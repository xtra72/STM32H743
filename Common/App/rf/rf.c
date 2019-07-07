#include "target.h"
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
static  RET_VALUE   RF_waitingForAck(uint32_t _timeout);
RET_VALUE           RF_send(uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _buffer, uint32_t _size, uint8_t _options, uint32_t _timeout);
RET_VALUE           RF_sendFrame(RF_SPI_FRAME* frame, uint32_t timeout);

static  RET_VALUE   RF_lockAPI(uint32_t _timeout);
static  RET_VALUE   RF_unlockAPI(void);

RET_VALUE   RF_waitingForSlave(bool _ready, uint32_t _timeout);

static  RF_CONFIG           config_ =
{
    .shortAddress = 0x0000,
    .maxPayloadLength = RF_PAYLOAD_SIZE_MAX,
    .timeout = 100
};

static  osThreadId          threadId_ = NULL;
static  osThreadId          rfIOTaskId_ = NULL;
static  bool                stop_ = true;
static  SemaphoreHandle_t   semaphore_ = NULL;
static  SemaphoreHandle_t   apiSemaphore_ = NULL;

static  uint32_t            spiTimeout_ = 100;
static  RF_STATUS           status_ = RF_STATUS_STOPPED;
static  uint16_t            upCount_ = 0;
static  bool                ackReceived_ = false;
static  uint8_t             state_ = RF_CC1310_STOP;
static  QueueHandle_t       rxQueue = NULL;
static  QueueHandle_t       txQueue_ = NULL;
static  RF_CONFIG           resultSetConfig_;
static  RET_VALUE           retSetConfig_ = RET_TIMEOUT;
static  RF_CONFIG           resultGetConfig_;
static  RET_VALUE           retGetConfig_= RET_TIMEOUT;

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
    "Stopped",      //RF_CC1310_STOPPED
    "Ready",        //RF_STATUS_READY,
    "Send",         //RF_STATUS_SEND,
    "Recv",         //RF_STATUS_RECV,
};

RET_VALUE   RF_init(SPI_HandleTypeDef* _spi)
{
    semaphore_ = xSemaphoreCreateMutex();

    apiSemaphore_ = xSemaphoreCreateBinary();
    xSemaphoreGive( apiSemaphore_ );

    rxQueue = xQueueCreate( RF_RX_QUEUE_SIZE, RF_BUFFER_SIZE_MAX);
    if (rxQueue == NULL)
    {
        while(1);
    }

    txQueue_ = xQueueCreate( RF_TX_QUEUE_SIZE, RF_BUFFER_SIZE_MAX);
    if (txQueue_ == NULL)
    {
        while(1);
    }

    SPI_init(_spi);

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

    uint8_t     buffer[RF_BUFFER_SIZE_MAX];

    RF_reset();

    status_ = RF_STATUS_READY;
    stop_ = false;

    RF_IO_start();

    while(!stop_)
    {
        if (RF_QUEUE_pop(rxQueue, buffer, 0) == RET_OK)
        {
            RF_commandProcessing(buffer, RF_BUFFER_SIZE_MAX);
        }

        osDelay(1);
    }

    stop_ = true;
    status_ = RF_STATUS_STOPPED;

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
        static  RF_SPI_FRAME    rxFrame;
        static  RF_SPI_FRAME    txFrame;

        if (RF_QUEUE_pop(txQueue_, (uint8_t *)&txFrame, 10) != RET_OK)
        {
            txFrame.cmd = 0x00;
            txFrame.length = 0;
            txFrame.crc = CRC16_calc(txFrame.payload, txFrame.length);
        }
        else
        {
            statistics_.Tx.packets++;
            statistics_.Tx.bytes += txFrame.length;
        }
        osDelay(1);

        if (txFrame.cmd != 0x0)
        {
            GPIO_MASTER_setStatus(true);
            if (RF_waitingForSlave(true, 1000) == RET_OK)
            {
                uint32_t    startTick = TICK_get();

                GPIO_MASTER_setStatus(false);

                //DEBUG("SPI_transmitReceive(%02x, %02x, %04x)\n", txFrame.cmd, txFrame.length, txFrame.crc);
                ret = SPI_transmitReceive((uint8_t *)&txFrame, (uint8_t*)&rxFrame, sizeof(RF_SPI_FRAME), TICK_remainTime(startTick, spiTimeout_));
                if (ret == RET_OK)
                {
                    DEBUG("Wait for response : %d\n", TICK_remainTime(startTick, spiTimeout_));
//                    if (RF_waitingForSlave(false, TICK_remainTime(startTick, spiTimeout_)) == RET_OK)
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
                                case    RF_SPI_CMD_SET_CONFIG:
                                    {
                                        if (rxFrame.length != sizeof(RF_CONFIG))
                                        {
                                            retSetConfig_ = RET_ERROR;
                                            break;
                                        }
                                        retSetConfig_ = RET_OK;
                                        memcpy(&resultSetConfig_, rxFrame.payload, sizeof(RF_CONFIG));
                                    }
                                    break;

                                case    RF_SPI_CMD_GET_CONFIG:
                                    {
                                        if (rxFrame.length != sizeof(RF_CONFIG))
                                        {
                                            retGetConfig_ = RET_ERROR;
                                            break;
                                        }
                                        retGetConfig_ = RET_OK;
                                        memcpy(&resultGetConfig_, rxFrame.payload, sizeof(RF_CONFIG));
                                    }
                                    break;

                                default:
                                    if (RF_QUEUE_push(rxQueue, (uint8_t*)&rxFrame) != RET_OK)
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
            }
        }
    }
}

RET_VALUE   RF_getConfig(RF_CONFIG* _config, uint32_t _timeout)
{
    RET_VALUE   ret;
    RF_SPI_FRAME    frame;

    frame.cmd = RF_SPI_CMD_GET_CONFIG;
    frame.length = 1;
    frame.payload[0] = RF_SPI_CMD_GET_CONFIG;
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
            memcpy(_config, &resultGetConfig_, sizeof(RF_CONFIG));
            break;
        }
        osDelay(1);
    }

    ret = retGetConfig_;

    return  ret;
}

RET_VALUE   RF_setConfig(RF_CONFIG* _config, uint32_t _timeout)
{
    RET_VALUE   ret;
    RF_SPI_FRAME    frame;

    frame.cmd = RF_SPI_CMD_SET_CONFIG;
    frame.length = sizeof(RF_CONFIG);
    memcpy(frame.payload, _config, sizeof(RF_CONFIG));
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
            memcpy(_config, &resultSetConfig_, sizeof(RF_CONFIG));
            break;
        }
        osDelay(1);
    }

    ret = retSetConfig_;

    return  ret;
}

RET_VALUE   RF_commandProcessing(uint8_t* _data, uint32_t _length)
{
#if 0
    DEBUG_DUMP("RCVD", _data, _length, 0);
#endif

    if (_length < sizeof(RF_FRAME_HEADER))
    {
        DEBUG("Invalid frame!\n");
        return  RET_ERROR;
    }

    RF_FRAME*   frame = (RF_FRAME*)_data;

    DEBUG("Port : %d\n", frame->header.port);
    switch(frame->header.port)
    {
    case    RF_CMD_ACK:
        {
            ackReceived_ = true;
        }
        break;

#if SUPPORT_DRAM
    case    RF_CMD_START_SCAN:
        {
            bool    reset = false;

            if (frame->header.length == 1)
            {
                reset = (frame->payload[0] != 0);
            }

            if (SCAN_start(reset) == RET_OK)
            {
                RF_sendACK(frame->header.srcAddress, config_.timeout);
            }

        }
        break;

    case    RF_CMD_STOP_SCAN:
        {
            if (SCAN_stop() == RET_OK)
            {
                RF_sendACK(frame->header.srcAddress, config_.timeout);
            }
        }
        break;
#endif
    case    RF_CMD_REP_DATA_COUNT:
        {
            DEBUG("Data Count : %d\n", ntohUint32((*(uint32_t *)frame->payload)));
            ackReceived_ = true;
        }
        break;

    case    RF_CMD_REQ_DATA_COUNT:
        {
            RET_VALUE   ret;

            ret = RF_sendResponseDataCount(frame->header.srcAddress, SCAN_getCurrentLoop(), config_.timeout);
            if (ret != RET_OK)
            {
                DEBUG("Data count response failed!\n");
            }
        }
        break;

    case    RF_CMD_REQ_DATA:
        {
            RET_VALUE   ret;
            uint32_t    offset = 0;
            uint32_t    count = 0;

            RF_sendACK(frame->header.srcAddress, config_.timeout);
            osDelay(config_.timeout);

            offset = ntohUint32(((uint32_t*)frame->payload)[0]);
            count = ntohUint32(((uint32_t*)frame->payload)[1]);

            DEBUG("Data Get : %d, %d\n", offset, count);
            if (offset + count < SCAN_getCurrentLoop())
            {
                DEBUG("Out of index : %d\n", SCAN_getCurrentLoop());
            }

            for(uint32_t i = 0 ; i < count / 2 ; i++)
            {
                uint8_t    buffer[RF_PAYLOAD_SIZE_MAX];
                uint32_t    length = 0;

                SCAN_LOOP_DATA*   data;

                buffer[length++] = ((offset + i*2) >> 24) & 0xFF;
                buffer[length++] = ((offset + i*2) >> 16) & 0xFF;
                buffer[length++] = ((offset + i*2) >>  8) & 0xFF;
                buffer[length++] = ((offset + i*2)      ) & 0xFF;

                data = SCAN_getLoopData(offset + i*2);

                buffer[length++] = (data->time >> 24) & 0xFF;
                buffer[length++] = (data->time >> 16) & 0xFF;
                buffer[length++] = (data->time >>  8) & 0xFF;
                buffer[length++] = (data->time      ) & 0xFF;

                for(uint32_t j =  0 ; j <  ADC_CHANNEL_getCount() ; j++)
                {
                    buffer[length++] = (data->data[j] >>  8) & 0xFF;
                    buffer[length++] = (data->data[j]      ) & 0xFF;
                }

                data = SCAN_getLoopData(offset + i*2+1);
                for(uint32_t j =  0 ; j <  ADC_CHANNEL_getCount() ; j++)
                {
                    buffer[length++] = (data->data[j] >>  8) & 0xFF;
                    buffer[length++] = (data->data[j]      ) & 0xFF;
                }

                ret = RF_sendData(frame->header.srcAddress, RF_CMD_REP_DATA, buffer, length, 0, 1000);
                if (ret != RET_OK)
                {
                    DEBUG("Data send failed[%d/%d]\n", i, count);
                }
            }
        }
        break;

    default:
        DEBUG("Unknown CMD : %02x\n", frame->header.port);
    }

    return  RET_OK;
}

uint32_t    RF_getTimeout(void)
{
    return  config_.timeout;
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
    return  config_.shortAddress;
}

RET_VALUE   RF_setShortAddress(uint16_t _shortAddress)
{
    config_.shortAddress = _shortAddress;

    return  RET_OK;
}

RET_VALUE   RF_setMaxPayloadLength(uint32_t _maxPayloadLength)
{
    if (_maxPayloadLength < RF_PAYLOAD_SIZE_MAX)
    {
        config_.maxPayloadLength = _maxPayloadLength;

        return  RET_OK;
    }

    return  RET_ERROR;
}

uint32_t    RF_getMaxPayloadLength(void)
{
    return  config_.maxPayloadLength;
}

void    RF_reset(void)
{
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);
    osDelay(5);
}

RET_VALUE   RF_send(uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t timeout)
{
    uint8_t ret = RET_ERROR;
    TickType_t      tickStart  = xTaskGetTickCount();
    TickType_t      tickTimeout = timeout / portTICK_PERIOD_MS;
    RF_SPI_FRAME    frame;

    if (tickTimeout == 0)
    {
        tickTimeout = config_.timeout  / portTICK_PERIOD_MS;
    }

    if (config_.maxPayloadLength < _dataSize)
    {
        DEBUG("Data is too long[%d < %d]\n", config_.maxPayloadLength, _dataSize);
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
    frame.payload[frame.length++] = (config_.shortAddress >> 8) & 0xFF;
    frame.payload[frame.length++] = (config_.shortAddress     ) & 0xFF;
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

RET_VALUE   RF_sendData(uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t _timeout)
{
    return  RF_send(RF_SPI_CMD_TX_DATA, _destAddress, _port, _data, _dataSize, _options, _timeout);
}

RET_VALUE   RF_sendStartScan(uint16_t _destAddress, bool _reset, uint32_t _timeout)
{
    RET_VALUE   ret;
    uint32_t    expireTick;

    expireTick = TICK_get() + _timeout;

    TRACE("Send Start Scan\n");
    ret = RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_START_SCAN, (uint8_t*)&_reset, 1, RF_OPTIONS_ACK, _timeout);
    if (ret == RET_OK)
    {
        if (expireTick <= TICK_get())
        {
            ret = RET_TIMEOUT;
        }
        else
        {
            ret = RF_waitingForAck(expireTick - TICK_get());
        }
    }

    return  ret;
}

RET_VALUE   RF_sendStopScan(uint16_t _destAddress, uint32_t _timeout)
{
    RET_VALUE   ret;
    uint32_t    expireTick;

    expireTick = TICK_get() + _timeout;

    TRACE("Send Start Scan\n");
    ret = RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_STOP_SCAN, NULL, 0, RF_OPTIONS_ACK, _timeout);
    if (ret == RET_OK)
    {
        if (expireTick <= TICK_get())
        {
            ret = RET_TIMEOUT;
        }
        else
        {
            ret = RF_waitingForAck(expireTick - TICK_get());
        }
    }

    return  ret;
}

RET_VALUE   RF_sendRequestDataCount(uint16_t _destAddress, uint32_t _timeout)
{
    RET_VALUE   ret;
    uint32_t    expireTick;

    expireTick = TICK_get() + _timeout;

    TRACE("Send Start Scan\n");
    ret = RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_REQ_DATA_COUNT, NULL, 0, RF_OPTIONS_ACK, _timeout);
    if (ret == RET_OK)
    {
        if (expireTick <= TICK_get())
        {
            ret = RET_TIMEOUT;
        }
        else
        {
            ret = RF_waitingForAck(expireTick - TICK_get());
        }
    }

    return  ret;
}

RET_VALUE   RF_sendResponseDataCount(uint16_t _destAddress, uint32_t _count, uint32_t _timeout)
{
    uint8_t buffer[RF_PAYLOAD_SIZE_MAX];
    uint8_t length = 0;

    buffer[length++] = (_count >> 24) & 0xFF;
    buffer[length++] = (_count >> 16) & 0xFF;
    buffer[length++] = (_count >>  8) & 0xFF;
    buffer[length++] = (_count      ) & 0xFF;


    return  RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_REP_DATA_COUNT, buffer, length, 0, _timeout);
}

RET_VALUE   RF_recvResponseDataCount(uint16_t* _srcAddress, uint32_t* _count, uint32_t _timeout)
{
    return  RET_ERROR;
}


RET_VALUE   RF_sendRequestData(uint16_t _destAddress, uint32_t _offset, uint32_t _count, uint32_t _timeout)
{
    RET_VALUE   ret;
    uint8_t buffer[RF_PAYLOAD_SIZE_MAX];
    uint8_t length = 0;
    uint32_t    expireTick;

    expireTick = TICK_get() + _timeout;

    buffer[length++] = (_offset >> 24) & 0xFF;
    buffer[length++] = (_offset >> 16) & 0xFF;
    buffer[length++] = (_offset >>  8) & 0xFF;
    buffer[length++] = (_offset      ) & 0xFF;

    buffer[length++] = (_count >> 24) & 0xFF;
    buffer[length++] = (_count >> 16) & 0xFF;
    buffer[length++] = (_count >>  8) & 0xFF;
    buffer[length++] = (_count      ) & 0xFF;

    ret = RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_REQ_DATA, buffer, length, RF_OPTIONS_REQ_ACK, _timeout);
    if (ret == RET_OK)
    {
        if (expireTick <= TICK_get())
        {
            ret = RET_TIMEOUT;
        }
        else
        {
            ret = RF_waitingForAck(expireTick - TICK_get());
        }
    }

    return  ret;

}

RET_VALUE   RF_sendACK(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_ACK, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendNAK(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(RF_SPI_CMD_TX_DATA, _destAddress, RF_CMD_NAK, NULL, 0, RF_OPTIONS_ACK, _timeout);
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

RET_VALUE   RF_waitingForAck(uint32_t _timeout)
{
    TRACE("Wait for %d ms to wait for an ACK.\n", _timeout);

    uint32_t    expireTick = TICK_get() + _timeout;
    ackReceived_ = false;
    while(TICK_get() < expireTick)
    {
        if (ackReceived_)
        {
            TRACE("Ack received\n");
            return  RET_OK;
        }

        osDelay(1);
    }

    TRACE("The ACK wait time[%d ms] has expired.\n", _timeout);
    return  RET_TIMEOUT;
}

RET_VALUE   RF_startAutoTransfer(uint32_t timeout)
{
    RF_SPI_FRAME    frame;

    frame.cmd = RF_SPI_CMD_START_AUTO_TRANSFER;
    frame.length = 4;
    *(uint32_t *)frame.payload = 1000;
    frame.crc = CRC16_calc(frame.payload, frame.length);

    return  RF_sendFrame(&frame, timeout);
}

RET_VALUE   RF_stopAutoTransfer(uint32_t timeout)
{
    RF_SPI_FRAME    frame;

    frame.cmd = RF_SPI_CMD_STOP_AUTO_TRANSFER;
    frame.length = 1;
    frame.payload[0] = 0;
    frame.crc = CRC16_calc(frame.payload, frame.length);

    return  RF_sendFrame(&frame, timeout);
}

RET_VALUE   RF_startMotionDetection(uint32_t timeout)
{
    RF_SPI_FRAME    frame;

    frame.cmd = RF_SPI_CMD_MOTION_DETECTION;
    frame.length = 4;
    *(uint32_t *)frame.payload = 1000;
    frame.crc = CRC16_calc(frame.payload, frame.length);

    return  RF_sendFrame(&frame, timeout);
}

/*******************************************************************
** RF_sendDummy : Sends dummy frame                                 **
********************************************************************
** In  :
** Out :
*******************************************************************/
RET_VALUE   RF_sendDummy(void)
{
    RF_SPI_FRAME frame;

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
RET_VALUE   RF_sendFrame(RF_SPI_FRAME *frame, uint32_t timeout)
{
    RET_VALUE       ret;

    ret = RF_lockAPI(timeout);
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



RET_VALUE   RF_lockAPI(uint32_t _timeout)
{
    TickType_t  tickTimeout = _timeout / portTICK_PERIOD_MS;

    if (xSemaphoreTake( apiSemaphore_, tickTimeout )  != pdTRUE)
    {
        DEBUG("Take failed\n");
        return  RET_TIMEOUT;
    }

//    apiLocked = true;


    return  RET_OK;
}


RET_VALUE   RF_unlockAPI(void)
{
    xSemaphoreGive( apiSemaphore_ );
//    apiLocked = false;

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