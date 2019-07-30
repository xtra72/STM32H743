#include "target.h"
#include "config.h"
#include "rf.h"
#include "crc16.h"
#include "time2.h"
#include "scan.h"
#include "utils.h"
#include "shell.h"
#include "gpio.h"
#include "spi.h"

#ifndef RF_BUFFER_SIZE_MAX
#define RF_BUFFER_SIZE_MAX  RF_SPI_FRAME_SIZE
#endif

#define RF_FRAME_OVERHEAD_SIZE  11

#ifndef RF_PAYLOAD_SIZE_MAX
#define RF_PAYLOAD_SIZE_MAX  (RF_BUFFER_SIZE_MAX - RF_FRAME_OVERHEAD_SIZE)
#endif

#define RF_RX_QUEUE_SIZE    16
#define RF_TX_QUEUE_SIZE    16

#define __MODULE_NAME__ "RF"
#include "trace.h"

#define RF_RECEIVE_QUEUE_SIZE   16

RET_VALUE   RF_QUEUE_push(QueueHandle_t _queue, uint8_t *_data);
RET_VALUE   RF_QUEUE_pop(QueueHandle_t _queue, uint8_t *_buffer, uint32_t _timeout);

static  void        RF_taskMain(void const * argument);
static  RET_VALUE   RF_IO_start(void);
static  void        RF_IO_taskMain(void const * argument);

static  RET_VALUE   RF_commandProcessing(uint8_t* _data, uint32_t _length);
RET_VALUE           RF_send(uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _buffer, uint32_t _size, uint8_t _options, uint32_t _timeout);
RET_VALUE           RF_sendFrame(RF_IO_FRAME* frame, uint32_t timeout);

static  RET_VALUE   RF_lockAPI(uint32_t _timeout);
static  RET_VALUE   RF_unlockAPI(void);

RET_VALUE   RF_waitingForSlave(bool _ready, uint32_t _timeout);

extern  CONFIG           config_;

static  osThreadId          threadId_ = NULL;
static  osThreadId          rfIOTaskId_ = NULL;
static  bool                stop_ = true;
static  SemaphoreHandle_t   semaphore_ = NULL;
static  SemaphoreHandle_t   apiSemaphore_ = NULL;

static  uint32_t            timeout_ = 1000;
static  uint32_t            spiTimeout_ = 100;
static  RF_STATUS           status_ = RF_STATUS_STOPPED;
static  uint16_t            upCount_ = 0;
static  uint8_t             state_ = RF_CC1310_STOP;
static  QueueHandle_t       rxQueue_ = NULL;
static  QueueHandle_t       txQueue_ = NULL;
static  RF_CC1310_CONFIG    resultSetCC1310Config_;
static  RET_VALUE           retSetConfig_ = RET_TIMEOUT;
static  RF_CC1310_CONFIG    resultGetCC1310Config_;
static  RET_VALUE           retGetConfig_= RET_TIMEOUT;
static  uint32_t            sleepTime_ = 0;
static  uint32_t            sleepStartTime_ = 0;

static  RF_STATISTICS       statistics_ =
{
    .Rx =
    {
        .bytes = 0,
        .packets = 0
    },
    .Tx =
    {
        .bytes = 0,
        .packets = 0
    }
};

static  const char*   statusStrings_[] =
{
    "Stopped",
    "Init",
    "Waiting for contract",
    "Ready",
    "Motion Detection",
    "Motion Detected",
    "Scan",
    "Slee"
};

void RF_transferScanData(void const * argument);

static  osTimerId   timerLoopFinishedHandler = 0;
static  uint32_t    motionDetectedNotificationCount_ = 0;
static  uint32_t    motionDetectedNotificationCountMax_ = 10;
static  uint32_t    motionDetectedNotificationInterval_ = 10000;
static  uint32_t    motionDetectedTime_ = 0;




RET_VALUE   RF_init(SPI_HandleTypeDef* _spi)
{
    semaphore_ = xSemaphoreCreateMutex();

    apiSemaphore_ = xSemaphoreCreateBinary();
    xSemaphoreGive( apiSemaphore_ );

    rxQueue_ = xQueueCreate( RF_RX_QUEUE_SIZE, RF_BUFFER_SIZE_MAX);
    txQueue_ = xQueueCreate( RF_TX_QUEUE_SIZE, RF_BUFFER_SIZE_MAX);
    SPI_init(_spi);

    if ((rxQueue_ == NULL) || (txQueue_ == NULL))
    {
        return  RET_NOT_ENOUGH_MEMORY;
    }

    osTimerDef(timerLoopFinished, RF_transferScanData);
    timerLoopFinishedHandler = osTimerCreate(osTimer(timerLoopFinished), osTimerPeriodic, NULL);

    return  RET_OK;
}

RET_VALUE   RF_start(void)
{
    RET_VALUE   ret = RET_OK;

    if (threadId_ == NULL)
    {
        osThreadDef(rfTask, RF_taskMain, osPriorityIdle, 0, 512);
        threadId_ = osThreadCreate(osThread(rfTask), NULL);
        if (threadId_ == NULL)
        {
            ret = RET_ERROR;
        }
    }

    return  ret;
}

RET_VALUE   RF_stop(void)
{
    RET_VALUE   ret = RET_OK;

    while(!stop_)
    {
        osDelay(1);
    }

    osThreadTerminate(threadId_);
    threadId_ = NULL;

    return  ret;
}

void RF_taskMain(void const * argument)
{
    DEBUG("RF task start!\n");
    uint32_t    serverRequestTime = 0;

    uint8_t     buffer[RF_BUFFER_SIZE_MAX];

    RF_reset();

    status_ = RF_STATUS_INIT;
    stop_ = false;

    RF_IO_start();

    while(!stop_)
    {
        if (RF_QUEUE_pop(rxQueue_, buffer, 10) == RET_OK)
        {
            RF_commandProcessing(buffer, RF_BUFFER_SIZE_MAX);
        }

        switch(status_)
        {
        case    RF_STATUS_INIT:
            {
                RF_setStatus(RF_STATUS_WAITING_FOR_CONTRACT);
            }
            break;


        case    RF_STATUS_WAITING_FOR_CONTRACT:
            {
                if (TICK_remainTime(serverRequestTime, 10000) == 0)
                {
                    RF_sendContract(0, config_.deviceId, config_.adc.channelCount, 10);
                    RF_setStatus(RF_STATUS_WAITING_FOR_CONTRACT);
                    serverRequestTime =  TICK_get();;
                }
            }
            break;
        case    RF_STATUS_READY:
            {
            }
            break;

        case    RF_STATUS_MOTION_DETECTED:
            {
                if (motionDetectedNotificationCount_ < motionDetectedNotificationCountMax_)
                {
                    if (TICK_remainTime(motionDetectedTime_ + motionDetectedNotificationCount_ * motionDetectedNotificationInterval_, motionDetectedNotificationInterval_) == 0)
                    {
                        RF_sendMotionDetected(0, 1000);
                        motionDetectedNotificationCount_++;
                    }
                }
                else
                {
                    RF_setStatus(RF_STATUS_READY);
                }
            }
            break;

        case    RF_STATUS_SCAN:
            {
            }
            break;

        case    RF_STATUS_SLEEP:
            {
                if (TICK_remainTime(sleepStartTime_, sleepTime_) > 0)
                {
                    osDelay(1000);
                }
            }
            break;
        }
        osDelay(1);
    }

    stop_ = true;
    RF_setStatus(RF_STATUS_STOPPED);

}


RET_VALUE    RF_IO_start(void)
{
    RET_VALUE   ret = RET_OK;

    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);

    osThreadDef(rfIOTask, RF_IO_taskMain, osPriorityIdle, 0, 512);
    rfIOTaskId_ = osThreadCreate(osThread(rfIOTask), NULL);
    if (rfIOTaskId_ == NULL)
    {
        ret = RET_ERROR;
    }

    return  ret;
}

void RF_IO_taskMain(void const * argument)
{
    DEBUG("RF IO start!\n");

    while(true)
    {
        RET_VALUE   ret;
        static  RF_IO_FRAME    rxFrame;
        static  RF_IO_FRAME    txFrame;

        if (RF_QUEUE_pop(txQueue_, (uint8_t *)&txFrame, config_.rf.keepAlive) != RET_OK)
        {
            RF_makeFrame(&txFrame, RF_IO_CMD_TX_DATA, 0, RF_REQ_PING, NULL, 0, RF_OPTIONS_ACK);
        }
        else
        {
            statistics_.Tx.packets++;
            statistics_.Tx.bytes += txFrame.length;
        }

        if (txFrame.cmd != 0x0)
        {
            GPIO_MASTER_setStatus(true);
            if (RF_waitingForSlave(true, 1000) == RET_OK)
            {
                uint32_t    startTick = TICK_get();

                GPIO_MASTER_setStatus(false);

                //DEBUG("SPI_transmitReceive(%02x, %02x, %04x)\n", txFrame.cmd, txFrame.length, txFrame.crc);
                ret = SPI_transmitReceive((uint8_t *)&txFrame, (uint8_t*)&rxFrame, sizeof(RF_IO_FRAME), spiTimeout_);
                if (ret == RET_OK)
                {
                    if (rxFrame.qsize < 10)
                    {
                        DEBUG("SPI Queue Size : %d\n", rxFrame.qsize);
                    }
                    //if (RF_waitingForSlave(false, TICK_remainTime(startTick, spiTimeout_)) == RET_OK)
                    {
                        if (rxFrame.cmd)
                        {
                            statistics_.Rx.packets++;
                            if ((rxFrame.length < sizeof(rxFrame.payload)) && (rxFrame.crc = CRC16_calc(rxFrame.payload, rxFrame.length)))
                            {
                                statistics_.Rx.packets++;
                                statistics_.Rx.bytes += rxFrame.length;

                                switch(rxFrame.cmd)
                                {
                                case    RF_IO_REP_SET_CONFIG:
                                    {
                                        DEBUG("Receive setting Get response.\n");
                                        if (rxFrame.length != sizeof(RF_CC1310_CONFIG))
                                        {
                                            retSetConfig_ = RET_ERROR;
                                            break;
                                        }
                                        retSetConfig_ = RET_OK;
                                        memcpy(&resultSetCC1310Config_, rxFrame.payload, sizeof(RF_CC1310_CONFIG));
                                    }
                                    break;

                                case    RF_IO_REP_GET_CONFIG:
                                    {
                                        DEBUG("Receive setting Set response.\n");
                                        if (rxFrame.length != sizeof(RF_CC1310_CONFIG))
                                        {
                                            retGetConfig_ = RET_ERROR;
                                            break;
                                        }
                                        retGetConfig_ = RET_OK;
                                        memcpy(&resultGetCC1310Config_, rxFrame.payload, sizeof(RF_CC1310_CONFIG));
                                    }
                                    break;

                                default:
                                    if (RF_QUEUE_push(rxQueue_, (uint8_t*)&rxFrame) != RET_OK)
                                    {
                                        DEBUG("Rx Queue is full!\n");
                                    }
                                }
                            }
                            else
                            {
                                statistics_.Rx.errors++;
                            }
                        }
                    }
#if 0
                    else
                    {
                        DEBUG("RF response error!");
                    }
#endif
                }
                else
                {
                    DEBUG("Tx Failed!");
                }
                osDelay(10);
            }
        }
    }
}

RET_VALUE   RF_getConfig(RF_CONFIG* _config)
{
    memcpy(_config, &config_, sizeof(RF_CONFIG));

    return  RET_OK;
}

RET_VALUE   RF_setConfig(RF_CONFIG* _config)
{
    memcpy(&config_, _config, sizeof(RF_CONFIG));

    return  RET_OK;
}

RET_VALUE   RF_CC1310_readConfig(uint32_t _timeout)
{
    RET_VALUE       ret;
    RF_IO_FRAME    frame;

    frame.cmd = RF_SPI_CMD_REQUEST_GET_CONFIG;
    frame.length = 0;
    frame.crc = CRC16_calc(frame.payload, frame.length);

    retGetConfig_ = RET_TIMEOUT;

    ret = RF_sendFrame(&frame, _timeout);
    if (ret != RET_OK)
    {
        return  ret;
    }

    RF_sendDummy();
    RF_sendDummy();

    uint32_t    startTime = TICK_get();
    while(TICK_remainTime(startTime, _timeout))
    {
        if (retGetConfig_ == RET_OK)
        {
            memcpy(&config_.rf.cc1310, &resultGetCC1310Config_, sizeof(RF_CC1310_CONFIG));
            break;
        }
        osDelay(1);
    }

    ret = retGetConfig_;

    return  ret;
}

RET_VALUE   RF_CC1310_writeConfig(uint32_t _timeout)
{
    RET_VALUE   ret;
    RF_IO_FRAME    frame;

    frame.cmd = RF_SPI_CMD_REQUEST_SET_CONFIG;
    frame.length = sizeof(RF_CC1310_CONFIG);
    memcpy(frame.payload, &config_.rf.cc1310, sizeof(RF_CC1310_CONFIG));
    frame.crc = CRC16_calc(frame.payload, frame.length);

    retSetConfig_ = RET_TIMEOUT;

    ret = RF_sendFrame(&frame, _timeout);
    if (ret != RET_OK)
    {
        return  ret;
    }

    RF_sendDummy();
    RF_sendDummy();

    uint32_t    startTime = TICK_get();
    while(TICK_remainTime(startTime, _timeout))
    {
        if (retSetConfig_ == RET_OK)
        {
            break;
        }
        osDelay(1);
    }

    ret = retSetConfig_;

    return  ret;
}

RET_VALUE   RF_commandProcessing(uint8_t* _data, uint32_t _length)
{
    if (_length < sizeof(RF_FRAME_HEADER))
    {
        DEBUG("Invalid frame!\n");
        return  RET_ERROR;
    }

    RF_IO_FRAME*   io_frame = (RF_IO_FRAME*)_data;
    switch(io_frame->cmd)
    {
    case    RF_IO_REP_MOTION_DETECT_STARTED:
        {
            DEBUG("Motion detection started!\n");
            RF_setStatus(RF_STATUS_READY);
            RF_sendMotionDetectionStart(0, 100);
        }
        break;

    case    RF_IO_REP_MOTION_DETECT_STOPPED:
        {
            DEBUG("Motion detection stopped!\n");
            if (status_ == RF_STATUS_READY)
            {
                RF_setStatus(RF_STATUS_INIT);
            }
            RF_sendMotionDetectionStop(0, 100);
        }
        break;

    case    RF_IO_NOTI_MOTION_DETECTED:
        {
            if (status_ == RF_STATUS_READY)
            {
                DEBUG("Motion detected!\n");
                RF_motionDetectionStop(0);
                motionDetectedTime_ = TICK_get();
                motionDetectedNotificationCount_ = 0;
                RF_setStatus(RF_STATUS_MOTION_DETECTED);
            }
        }
        break;

    case    RF_IO_NOTI_FROM_SERVER:
        {
            DEBUG("Receive slave command : %d\n", ((RF_IO_REQUEST_FROM_SLAVE_PARAMS*)io_frame->payload)->cmd);
            switch(((RF_IO_REQUEST_FROM_SLAVE_PARAMS*)io_frame->payload)->cmd)
            {
            case    RF_REQ_SRV_SCAN_START:
                {
                    SCAN_DATA_reset();
                    SCAN_start();
                    RF_startTransferScanData();
                    RF_setStatus(RF_STATUS_SCAN);
                }
                break;

            case    RF_REQ_SRV_SCAN_STOP:
                {
                    SCAN_stop();
                }
                break;

            case    RF_REQ_SRV_TRANSFER_START:
                {
                    RF_startTransferScanData();
                }
                break;

            case    RF_REQ_SRV_TRANSFER_STOP:
                {
                    RF_stopTransferScanData();
                }
                break;

            case    RF_REQ_SRV_MOTION_DETECTION_START:
                {
                    RF_motionDetectionStart(0);
                    RF_setStatus(RF_STATUS_MOTION_DETECTION);}
                break;

            case    RF_REQ_SRV_MOTION_DETECTION_STOP:
                {
                    RF_motionDetectionStop(0);
                }
                break;

            case    RF_REQ_SRV_SLEEP:
                {
                    if (status_ == RF_STATUS_SCAN)
                    {
                        SCAN_stop();
                        RF_stopTransferScanData();
                    }
                    RF_setStatus(RF_STATUS_READY);
                }
                break;

            default:
                DEBUG("Invalid RF Command : %d\n", ((RF_IO_REQUEST_FROM_SLAVE_PARAMS*)io_frame->payload)->cmd);
            }
        }
        break;

    case    RF_IO_STATUS:
        {
        }
        break;

    case    RF_IO_DOWNLINK:
        {
            RF_FRAME*   rf_frame = (RF_FRAME*)io_frame->payload;
            if ((sizeof(RF_FRAME_HEADER) <= io_frame->length) && (sizeof(RF_FRAME_HEADER) + rf_frame->header.length <= io_frame->length))
            {
                switch(rf_frame->header.port)
                {
                case    RF_MSG_CONTRACT_CONFIRM:
                    {
                        if (status_ == RF_STATUS_WAITING_FOR_CONTRACT)
                        {
                            if (rf_frame->header.length == sizeof(RF_RESPONSE_CONTRACT))
                            {
                                RF_RESPONSE_CONTRACT*   response_contract = (RF_RESPONSE_CONTRACT*)rf_frame->payload;
                                uint32_t    timeStamp = 0;

                                timeStamp |= (response_contract->timeStamp >> 24) & 0x000000FF;
                                timeStamp |= (response_contract->timeStamp >> 8) & 0x0000FF00;
                                timeStamp |= (response_contract->timeStamp << 8) & 0x00FF0000;
                                timeStamp |= (response_contract->timeStamp << 24) & 0xFF000000;
                                DEBUG("Contract Received : %d", timeStamp);
                                RF_setStatus(RF_STATUS_READY);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("Contract is not waiting.");
                        }
                    }
                    break;

                case    RF_MSG_MOTION_DETECTION_START:
                    {
                        if (status_ == RF_STATUS_READY)
                        {
                            if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                            {
                                RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                RF_motionDetectionStart(0);
                                RF_setStatus(RF_STATUS_MOTION_DETECTION);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }

                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", RF_getStatusString(status_));
                        }
                    }
                    break;

                case    RF_MSG_MOTION_DETECTION_STOP:
                    {
                        if (status_ == RF_STATUS_MOTION_DETECTION)
                        {
                            if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                            {
                                RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                RF_motionDetectionStop(0);
                                RF_setStatus(RF_STATUS_READY);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", RF_getStatusString(status_));
                        }
                    }
                    break;

                case    RF_MSG_SCAN_START:
                    {
                        if (status_ == RF_STATUS_MOTION_DETECTED)
                        {
                            if (rf_frame->header.length == sizeof(RF_REQ_SCAN_PARAMS))
                            {
                                RF_REQ_SCAN_PARAMS*   params = (RF_REQ_SCAN_PARAMS*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                if (SCAN_start() == RET_OK)
                                {
                                    RF_motionDetectionStop(0);
                                    SCAN_DATA_reset();
                                    RF_startTransferScanData();
                                    RF_sendScanStart(0, 100);
                                    RF_setStatus(RF_STATUS_SCAN);
                                }
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", RF_getStatusString(status_));
                        }
                    }
                    break;

                case    RF_MSG_SCAN_STOP:
                    {
                        if (status_ == RF_STATUS_SCAN)
                        {
                            if (rf_frame->header.length == sizeof(RF_REQ_SCAN_PARAMS))
                            {
                                RF_REQ_SCAN_PARAMS*   params = (RF_REQ_SCAN_PARAMS*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                SCAN_stop();
                                RF_sendScanStop(0, 100);
                                RF_setStatus(RF_STATUS_READY);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", RF_getStatusString(status_));
                        }
                    }
                    break;

                case    RF_MSG_TRANS_START:
                    {
                        if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                        {
                            RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                            uint32_t    mid = 0;

                            mid |= (params->mid >> 24) & 0x000000FF;
                            mid |= (params->mid >> 8) & 0x0000FF00;
                            mid |= (params->mid << 8) & 0x00FF0000;
                            mid |= (params->mid << 24) & 0xFF000000;

                            RF_startTransferScanData();
                            RF_sendTransferStart(0, 100);
                        }
                        else
                        {
                            DEBUG("Payload does not conform to Contract response format.");
                        }
                    }
                    break;

                case    RF_MSG_TRANS_STOP:
                    {
                        if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                        {
                            RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                            uint32_t    mid = 0;

                            mid |= (params->mid >> 24) & 0x000000FF;
                            mid |= (params->mid >> 8) & 0x0000FF00;
                            mid |= (params->mid << 8) & 0x00FF0000;
                            mid |= (params->mid << 24) & 0xFF000000;

                            RF_stopTransferScanData();
                            RF_sendTransferStop(0, 100);
                        }
                        else
                        {
                            DEBUG("Payload does not conform to Contract response format.");
                        }
                    }
                break;

                case    RF_MSG_SLEEP:
                    {
                        if (rf_frame->header.length == sizeof(RF_SLEEP))
                        {
                            RF_SLEEP*   params = (RF_SLEEP *)rf_frame->payload;
                            uint32_t    sleepTime =0;

                            sleepTime |= (params->time >> 24) & 0x000000FF;
                            sleepTime |= (params->time >>  8) & 0x0000FF00;
                            sleepTime |= (params->time <<  8) & 0x00FF0000;
                            sleepTime |= (params->time << 24) & 0xFF000000;

                            RF_motionDetectionStop(0);
                            SCAN_stop();
                            RF_stopTransferScanData();
                            sleepTime_ = sleepTime;
                            sleepStartTime_ = TICK_get();

                            RF_setStatus(RF_STATUS_SLEEP);
                        }
                    }
                break;

                case    RF_MSG_STOP:
                    {
                        if (status_ == RF_STATUS_MOTION_DETECTION)
                        {
                            RF_motionDetectionStop(0);
                            RF_setStatus(RF_STATUS_READY);
                        }
                        else if (status_ == RF_STATUS_SCAN)
                        {
                            SCAN_stop();
                            RF_sendScanStop(0, 100);
                            RF_setStatus(RF_STATUS_READY);
                        }
                    }
                break;

                default:
                    DEBUG("Receive Data : %d\n", io_frame->length);
                    DEBUG_DUMP("DL", io_frame->payload, io_frame->length, 16);
                }
            }
            else
            {
                DEBUG("Invalid downlink data\n");
            }
        }
        break;


    default:
        DEBUG("Unknown CMD : %02x\n", io_frame->cmd);
    }

    return  RET_OK;
}

uint32_t    RF_getTimeout(void)
{
    return  config_.rf.cc1310.timeout;
}

bool        RF_setStatus(RF_STATUS _status)
{
    status_ = _status;

    return  true;
}

RF_STATUS   RF_getStatus(void)
{
    return  status_;
}

const char*       RF_getStatusString(RF_STATUS status)
{
    if (status < RF_STATUS_MAX)
    {
        return  statusStrings_[status];
    }

    return  "Unknown";
}

uint16_t    RF_getShortAddress(void)
{
    return  config_.rf.cc1310.shortAddress;
}

RET_VALUE   RF_setShortAddress(uint16_t _shortAddress)
{
    config_.rf.cc1310.shortAddress = _shortAddress;

    return  RET_OK;
}

RET_VALUE   RF_setFrequency(uint32_t _frequency)
{
    config_.rf.cc1310.frequency = _frequency;

    return  RET_OK;
}

uint32_t    RF_getFrequency(void)
{
    return  config_.rf.cc1310.frequency;
}

RET_VALUE   RF_setPower(int16_t _power)
{
    config_.rf.cc1310.power = _power;

    return  RET_OK;
}

int16_t    RF_getPower(void)
{
    return  config_.rf.cc1310.power;
}

RET_VALUE   RF_setMaxPayloadLength(uint32_t _maxPayloadLength)
{
    if (_maxPayloadLength < RF_PAYLOAD_SIZE_MAX)
    {
        config_.rf.cc1310.maxPayloadLength = _maxPayloadLength;

        return  RET_OK;
    }

    return  RET_ERROR;
}

uint32_t    RF_getMaxPayloadLength(void)
{
    return  config_.rf.cc1310.maxPayloadLength;
}

void    RF_reset(void)
{
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);
    osDelay(5);
}


RET_VALUE   RF_makeFrame(RF_IO_FRAME* _frame, uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options)
{
    if (config_.rf.cc1310.maxPayloadLength < _dataSize)
    {
        DEBUG("Data is too long[%d < %d]\n", config_.rf.cc1310.maxPayloadLength, _dataSize);
        return  RET_OUT_OF_RANGE;
    }

    _frame->cmd = cmd;
    _frame->length = 0;

    _frame->payload[_frame->length++] = (_destAddress >> 8) & 0xFF;
    _frame->payload[_frame->length++] = (_destAddress     ) & 0xFF;
    _frame->payload[_frame->length++] = (config_.rf.cc1310.shortAddress >> 8) & 0xFF;
    _frame->payload[_frame->length++] = (config_.rf.cc1310.shortAddress     ) & 0xFF;
    _frame->payload[_frame->length++] = _port;
    _frame->payload[_frame->length++] = _options;
    _frame->payload[_frame->length++] = (upCount_     ) & 0xFF;
    _frame->payload[_frame->length++] = _dataSize;
    if (_dataSize != 0)
    {
        memcpy(&_frame->payload[_frame->length], _data, _dataSize);
    }
    _frame->length += _dataSize;
    _frame->crc = CRC16_calc(_frame->payload, _frame->length);

    return  RET_OK;
}

RET_VALUE   RF_send(uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t timeout)
{
    uint8_t ret = RET_ERROR;
    TickType_t      tickStart  = xTaskGetTickCount();
    TickType_t      tickTimeout = timeout / portTICK_PERIOD_MS;
    RF_IO_FRAME     frame;

    if (tickTimeout == 0)
    {
        tickTimeout = config_.rf.cc1310.timeout  / portTICK_PERIOD_MS;
    }

    if (config_.rf.cc1310.maxPayloadLength < _dataSize)
    {
        DEBUG("Data is too long[%d < %d]\n", config_.rf.cc1310.maxPayloadLength, _dataSize);
        return  RET_OUT_OF_RANGE;
    }

    while( xSemaphoreTake( semaphore_, 0) != pdTRUE )
    {
        if (tickTimeout < xTaskGetTickCount() - tickStart)
        {
            DEBUG("RF send failed!\n");
            return  RET_ERROR;
        }

        osDelay(1);
    }

    if (tickTimeout <= (TICK_get() - tickStart))
    {
        tickTimeout = 1;
    }
    else
    {
        tickTimeout = tickTimeout - (TICK_get() - tickStart);
    }

    upCount_++;

    frame.cmd = cmd;
    frame.length = 0;

    frame.payload[frame.length++] = (_destAddress >> 8) & 0xFF;
    frame.payload[frame.length++] = (_destAddress     ) & 0xFF;
    frame.payload[frame.length++] = (config_.rf.cc1310.shortAddress >> 8) & 0xFF;
    frame.payload[frame.length++] = (config_.rf.cc1310.shortAddress     ) & 0xFF;
    frame.payload[frame.length++] = _port;
    frame.payload[frame.length++] = _options;
    frame.payload[frame.length++] = (upCount_     ) & 0xFF;
    frame.payload[frame.length++] = _dataSize;
    if (_dataSize != 0)
    {
        memcpy(&frame.payload[frame.length], _data, _dataSize);
    }
    frame.length += _dataSize;
    frame.crc = CRC16_calc(frame.payload, frame.length);

    ret = RF_sendFrame(&frame, tickTimeout);
    xSemaphoreGive( semaphore_ );
    if (ret != RET_OK)
    {
        DEBUG("send failed!\n");
    }

    return  ret;
}

RET_VALUE   RF_sendData(uint16_t _destAddress, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t _timeout)
{
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_DATA, _data, _dataSize, _options, _timeout);
}

RET_VALUE   RF_sendDataCount(uint16_t _destAddress, uint32_t _timeout)
{
    uint32_t    count = SCAN_getCurrentLoop();

    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_REP_SRV_DATA_COUNT, (uint8_t *)&count, sizeof(count), 0, _timeout);
}

RET_VALUE   RF_sendACK(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_REP_ACK, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendNAK(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_REP_NACK, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendPing(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_REQ_PING, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendContract(uint16_t _destAddress, char* _deviceId, uint8_t _channelCount, uint32_t _timeout)
{
    RF_REQUEST_CONTRACT contract;

    strncpy(contract.deviceId, _deviceId, TARGET_DEVICE_ID_LEN);
    contract.channelCuont = _channelCount;

    DEBUG("RF_sendContract : %s, %d", _deviceId, _channelCount);
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_REQ_CONTRACT, (uint8_t *)&contract, sizeof(contract), RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendMotionDetectionStart(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("RF_sendMotionDetectionStart");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_MOTION_DETECTION_STARTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendMotionDetectionStop(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("RF_sendMotionDetectionStop");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_MOTION_DETECTION_STOPPED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendMotionDetected(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_MOTION_DETECTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendScanStart(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("RF_sendScanStart");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_SCAN_STARTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendScanStop(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("RF_sendScanStop");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_SCAN_STOPPED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendTransferStart(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("RF_sendTransferStart");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_TRANS_STARTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendTransferStop(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("RF_sendTransferStop");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_TRANS_STOPPED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_recv(uint8_t* buffer, uint32_t bufferSize, uint32_t* receivedLength,  uint16_t* _srcAddress, uint8_t* _port, uint8_t* _options, uint32_t timeout)
{
    TickType_t  tickStart  = TICK_get();
    TickType_t  tickTimeout = timeout / portTICK_PERIOD_MS;

    if( xSemaphoreTake( semaphore_, tickTimeout) != pdTRUE )
    {
        DEBUG("RF recv failed!\n");
        return  RET_ERROR;
    }

    if (tickTimeout <= (TICK_get() - tickStart))
    {
        tickTimeout = 1;
    }
    else
    {
        tickTimeout = tickTimeout - (TICK_get() - tickStart);
    }

    xSemaphoreGive( semaphore_ );

    return  RET_OK;
}

RET_VALUE   RF_waitingForSlave(bool _ready, uint32_t _timeout)
{
    TickType_t  tickStart  = TICK_get();

    while (GPIO_SLAVE_isReady() != _ready)
    {
        if (!TICK_remainTime(tickStart, _timeout))
        {
            return  RET_TIMEOUT;
        }

        osDelay(1);
    }

    return  RET_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

RET_VALUE   RF_getStatistics(RF_STATISTICS* _statistics)
{
    ASSERT(_statistics);

    memcpy(_statistics, &statistics_, sizeof(RF_STATISTICS));

    return  RET_OK;
}

RET_VALUE   RF_clearStatistics(void)
{
    memset(&statistics_, 0, sizeof(RF_STATISTICS));

    return  RET_OK;
}

RET_VALUE   RF_motionDetectionStart(uint32_t _timeout)
{
    if (status_ == RF_STATUS_READY)
    {
        RF_IO_FRAME    frame = {    .cmd = RF_IO_REQ_MOTION_DETECT_START, .length = 0, .crc = 0  };

        return  RF_sendFrame(&frame, _timeout);
    }

    return  RET_ERROR;
}

RET_VALUE   RF_motionDetectionStop(uint32_t _timeout)
{
    RF_IO_FRAME    frame = {    .cmd = RF_IO_REQ_MOTION_DETECT_STOP, .length = 0, .crc = 0  };

    return  RF_sendFrame(&frame, _timeout);
}

/*******************************************************************
** RF_sendDummy : Sends dummy frame                                 **
********************************************************************
** In  :
** Out :
*******************************************************************/
RET_VALUE   RF_sendDummy(void)
{
    RF_IO_FRAME frame;

    frame.cmd = 0x5A;
    frame.length = 1;
    frame.payload[0] = 0x5A;
    frame.crc = 0;

    return  RF_sendFrame(&frame, 10);
}

/*******************************************************************
** RF_sendFrame : Sends a RF frame                                 **
********************************************************************
** In  : *buffer, size                                            **
** Out : *pReturnCode                                             **
*******************************************************************/
RET_VALUE   RF_sendFrame(RF_IO_FRAME *frame, uint32_t _timeout)
{
    RET_VALUE       ret;

    if (_timeout == 0)
    {
        _timeout = timeout_;
    }

    ret = RF_lockAPI(_timeout);
    if (ret != RET_OK)
    {
        DEBUG("API is locked\n");
        return  ret;
    }

    if (frame->crc == 0)
    {
        frame->crc =  CRC16_calc(frame->payload, frame->length);
    }

    state_ |= RF_CC1310_TX | RF_CC1310_BUSY;
    state_ &= ~RF_CC1310_STOP;

    ret = RF_QUEUE_push(txQueue_, (uint8_t *)frame);
    if (ret != RET_OK)
    {
        DEBUG("TX Queue push failed!\n");
    }
    state_ |= RF_CC1310_STOP | RF_CC1310_TX_DONE;
    state_ &= ~(RF_CC1310_BUSY | RF_CC1310_TX);

    RF_unlockAPI();

    return  ret;
}


uint32_t    transfer_index = 0;
bool        doing_transfer = false;

RET_VALUE   RF_startTransferScanData()
{
    if (!doing_transfer)
    {
        transfer_index = 0;
        doing_transfer = true;
        osTimerStart(timerLoopFinishedHandler, config_.rf.transferInterval);
    }

    return  RET_OK;
}

RET_VALUE   RF_stopTransferScanData()
{
    if (doing_transfer)
    {
        doing_transfer = false;
        osTimerStop(timerLoopFinishedHandler);
    }

    return  RET_OK;
}


void    RF_transferScanData(void const * argument)
{
    uint8_t    buffer[RF_PAYLOAD_SIZE_MAX];
    uint32_t    length = 0;

    if (transfer_index + config_.rf.nop <= SCAN_getCurrentLoop())
    {
        for(uint32_t i =  0 ; i <  config_.rf.nop ; i++)
        {
            SCAN_LOOP_DATA* data = SCAN_getLoopData(transfer_index + i);

            if (i == 0)
            {
                uint32_t    time = transfer_index * config_.rf.nop;
                buffer[length++] = (time >> 24) & 0xFF;
                buffer[length++] = (time >> 16) & 0xFF;
                buffer[length++] = (time >>  8) & 0xFF;
                buffer[length++] = (time      ) & 0xFF;
            }

            for(uint32_t j =  0 ; j <  ADC_CHANNEL_getCount() ; j++)
            {
                buffer[length++] = (data->data[j] >>  8) & 0xFF;
                buffer[length++] = (data->data[j]      ) & 0xFF;
            }
        }

        RET_VALUE ret = RF_sendData(0, buffer, length, 0, 1000);
        if (ret == RET_OK)
        {
            transfer_index += config_.rf.nop;
        }
        else
        {
            DEBUG("Data send failed[%d/%d]\n", transfer_index, SCAN_getCurrentLoop());
        }
    }
}

uint32_t    RF_getKeepAlive()
{
    return  config_.rf.keepAlive;
}

RET_VALUE   RF_setKeepAlive(uint32_t _interval)
{
    config_.rf.keepAlive = _interval;

    return  RET_OK;
}

uint32_t    RF_getTransferInterval()
{
    return  config_.rf.transferInterval;
}

RET_VALUE   RF_setTransferInterval(uint32_t _interval)
{
    config_.rf.transferInterval = _interval;

    return  RET_OK;
}

uint32_t    RF_getTransferNOP()
{
    return  config_.rf.nop;
}

RET_VALUE   RF_setTransferNOP(uint32_t _nop)
{
    if ((TARGET_RF_TRANSFER_NOP_MIN <= _nop) && (_nop <= TARGET_RF_TRANSFER_NOP_MAX))
    {
        config_.rf.nop = _nop;
        return  RET_OK;
    }

    return  RET_INVALID_ARGUMENT;
}

RET_VALUE   RF_lockAPI(uint32_t _timeout)
{
    TickType_t  tickTimeout = _timeout / portTICK_PERIOD_MS;

    if (xSemaphoreTake( apiSemaphore_, tickTimeout )  != pdTRUE)
    {
        DEBUG("Take failed\n");
        return  RET_TIMEOUT;
    }

    return  RET_OK;
}


RET_VALUE   RF_unlockAPI(void)
{
    xSemaphoreGive( apiSemaphore_ );

    return  RET_OK;
}

bool    RF_QUEUE_isEmpty(QueueHandle_t _queue)
{
    return uxQueueSpacesAvailable(_queue) == RF_RECEIVE_QUEUE_SIZE;
}

RET_VALUE   RF_QUEUE_push(QueueHandle_t _queue, uint8_t *_data)
{
    if (xQueueSend(_queue, _data, 10)  != pdPASS)
    {
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   RF_QUEUE_pop(QueueHandle_t _queue, uint8_t *_buffer, uint32_t _timeout)
{
    if (xQueueReceive(_queue, _buffer, _timeout)  != pdPASS)
    {
        return  RET_ERROR;
    }

    return  RET_OK;

}

