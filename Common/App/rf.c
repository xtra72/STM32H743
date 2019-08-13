#include "target.h"
#include "config.h"
#include "wireguard.h"
#include "rf.h"
#include "crc16.h"
#include "time2.h"
#include "scan.h"
#include "utils.h"
#include "shell.h"
#include "gpio.h"
#include "spi.h"
#include "max17043.h"
#include "system.h"

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

RET_VALUE           RF_send(uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _buffer, uint32_t _size, uint8_t _options, uint32_t _timeout);
RET_VALUE           RF_sendFrame(RF_IO_FRAME* frame, uint32_t timeout);

static  RET_VALUE   RF_lockAPI(uint32_t _timeout);
static  RET_VALUE   RF_unlockAPI(void);

RET_VALUE   RF_waitingForSlave(bool _ready, uint32_t _timeout);

extern  CONFIG           config_;

static  osThreadId          threadId_ = NULL;
static  SemaphoreHandle_t   semaphore_ = NULL;
static  SemaphoreHandle_t   apiSemaphore_ = NULL;

static  uint32_t            timeout_ = 1000;
static  uint32_t            spiTimeout_ = 100;
static  uint16_t            upCount_ = 0;
static  uint8_t             state_ = RF_CC1310_STOP;
static  QueueHandle_t       txQueue_ = NULL;
static  RF_CONFIG    resultSetRFConfig_;
static  RET_VALUE           retSetConfig_ = RET_TIMEOUT;
static  RF_CONFIG    resultGetCC1310Config_;
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

void    RF_readyTimeout(void const * argument);


RET_VALUE   RF_init(SPI_HandleTypeDef* _spi)
{
    semaphore_ = xSemaphoreCreateMutex();

    apiSemaphore_ = xSemaphoreCreateBinary();
    xSemaphoreGive( apiSemaphore_ );

    txQueue_ = xQueueCreate( RF_TX_QUEUE_SIZE, RF_BUFFER_SIZE_MAX);
    SPI_init(_spi);

    if (txQueue_ == NULL)
    {
        return  RET_NOT_ENOUGH_MEMORY;
    }

    return  RET_OK;
}

RET_VALUE    RF_start(void)
{
    RET_VALUE   ret = RET_OK;

    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);

    osThreadDef(taskMain, RF_taskMain, osPriorityIdle, 0, 512);
    threadId_ = osThreadCreate(osThread(taskMain), NULL);
    if (threadId_ == NULL)
    {
        ret = RET_ERROR;
    }

    return  ret;
}

RET_VALUE    RF_stop(void)
{
    return  RET_OK;
}

void RF_taskMain(void const * argument)
{
    DEBUG("RF IO start!\n");

    while(true)
    {
        RET_VALUE   ret;
        static  RF_IO_FRAME    rxFrame;
        static  RF_IO_FRAME    txFrame;

        txFrame.cmd = 0x00;
        if (RF_QUEUE_pop(txQueue_, (uint8_t *)&txFrame, 1000) != RET_OK)
        {
            if (WG_getStatus() != WG_STATUS_MOTION_DETECTION)
            {
                RF_makeFrame(&txFrame, RF_IO_CMD_TX_DATA, 0, RF_REQ_PING, NULL, 0, RF_OPTIONS_ACK);
            }
            else
            {
                RF_makeFrame(&txFrame, RF_IO_CMD_PING, 0, RF_REQ_PING, NULL, 0, RF_OPTIONS_ACK);
            }
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
                uint32_t    retransmit = 0;
                uint32_t    startTick = TICK_get();

                GPIO_MASTER_setStatus(false);

                while(retransmit < 2)
                {
                    //DEBUG("SPI_transmitReceive(%02x, %02x, %04x)\n", txFrame.cmd, txFrame.length, txFrame.crc);
                    ret = SPI_transmitReceive((uint8_t *)&txFrame, (uint8_t*)&rxFrame, sizeof(RF_IO_FRAME), spiTimeout_);
                    if (ret == RET_OK)
                    {
                        break;
                    }

                    retransmit ++;
                }
                if (ret == RET_OK)
                {
                    if (rxFrame.qsize < 10)
                    {
                        DEBUG("SPI Queue Size : %d\n", rxFrame.qsize);
                        osDelay(50);
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
                                        if (rxFrame.length != sizeof(RF_CONFIG))
                                        {
                                            retSetConfig_ = RET_ERROR;
                                            break;
                                        }
                                        retSetConfig_ = RET_OK;
                                        memcpy(&resultSetRFConfig_, rxFrame.payload, sizeof(RF_CONFIG));
                                    }
                                    break;

                                case    RF_IO_REP_GET_CONFIG:
                                    {
                                        DEBUG("Receive setting Set response.\n");
                                        if (rxFrame.length != sizeof(RF_CONFIG))
                                        {
                                            retGetConfig_ = RET_ERROR;
                                            break;
                                        }
                                        retGetConfig_ = RET_OK;
                                        memcpy(&resultGetCC1310Config_, rxFrame.payload, sizeof(RF_CONFIG));
                                    }
                                    break;

                                default:
                                    if (WG_onData((uint8_t*)&rxFrame) != RET_OK)
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

uint32_t    RF_getBufferSize(void)
{
    return  RF_BUFFER_SIZE_MAX;
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
            memcpy(&config_.rf, &resultGetCC1310Config_, sizeof(RF_CONFIG));
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
    frame.length = sizeof(RF_CONFIG);
    memcpy(frame.payload, &config_.rf, sizeof(RF_CONFIG));
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

uint32_t    RF_getTimeout(void)
{
    return  config_.rf.timeout;
}

uint16_t    RF_getShortAddress(void)
{
    return  config_.rf.shortAddress;
}

RET_VALUE   RF_setShortAddress(uint16_t _shortAddress)
{
    config_.rf.shortAddress = _shortAddress;

    return  RET_OK;
}

RET_VALUE   RF_setFrequency(uint32_t _frequency)
{
    config_.rf.frequency = _frequency;

    return  RET_OK;
}

uint32_t    RF_getFrequency(void)
{
    return  config_.rf.frequency;
}

RET_VALUE   RF_setPower(int16_t _power)
{
    config_.rf.power = _power;

    return  RET_OK;
}

int16_t    RF_getPower(void)
{
    return  config_.rf.power;
}

RET_VALUE   RF_setMaxPayloadLength(uint32_t _maxPayloadLength)
{
    if (_maxPayloadLength < RF_PAYLOAD_SIZE_MAX)
    {
        config_.rf.maxPayloadLength = _maxPayloadLength;

        return  RET_OK;
    }

    return  RET_ERROR;
}

uint32_t    RF_getMaxPayloadLength(void)
{
    return  config_.rf.maxPayloadLength;
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
    if (config_.rf.maxPayloadLength < _dataSize)
    {
        DEBUG("Data is too long[%d < %d]\n", config_.rf.maxPayloadLength, _dataSize);
        return  RET_OUT_OF_RANGE;
    }

    _frame->cmd = cmd;
    _frame->length = 0;

    _frame->payload[_frame->length++] = (_destAddress >> 8) & 0xFF;
    _frame->payload[_frame->length++] = (_destAddress     ) & 0xFF;
    _frame->payload[_frame->length++] = (config_.rf.shortAddress >> 8) & 0xFF;
    _frame->payload[_frame->length++] = (config_.rf.shortAddress     ) & 0xFF;
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
        tickTimeout = config_.rf.timeout  / portTICK_PERIOD_MS;
    }

    if (config_.rf.maxPayloadLength < _dataSize)
    {
        DEBUG("Data is too long[%d < %d]\n", config_.rf.maxPayloadLength, _dataSize);
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
    frame.payload[frame.length++] = (config_.rf.shortAddress >> 8) & 0xFF;
    frame.payload[frame.length++] = (config_.rf.shortAddress     ) & 0xFF;
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

RET_VALUE   RF_sendKeepAlive(uint16_t _destAddress, uint32_t _batt, uint32_t _timeout)
{
    RF_KEEP_ALIVE   params;

    ((uint8_t*)&params.battery)[0] = ((uint8_t*)&_batt)[3];
    ((uint8_t*)&params.battery)[1] = ((uint8_t*)&_batt)[2];
    ((uint8_t*)&params.battery)[2] = ((uint8_t*)&_batt)[1];
    ((uint8_t*)&params.battery)[3] = ((uint8_t*)&_batt)[0];

    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_KEEPALIVE, (uint8_t *)&params, sizeof(params), RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendRadioStart(uint16_t _destAddress, RF_CONFIG* _config, uint32_t _timeout)
{
    RF_send(RF_IO_CMD_RADIO_START, 0, 0, (uint8_t*)_config, sizeof(RF_CONFIG), RF_OPTIONS_ACK, _timeout);

    return  RET_OK;
}

RET_VALUE   RF_sendContract(uint16_t _destAddress, char* _deviceId, uint8_t _channelCount, uint32_t _timeout)
{
    RF_REQUEST_CONTRACT contract;

    strncpy(contract.deviceId, _deviceId, TARGET_DEVICE_ID_LEN);
    contract.channelCuont = _channelCount;

    DEBUG("[NOTI] Contract request : %s, %d", _deviceId, _channelCount);
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_REQ_CONTRACT, (uint8_t *)&contract, sizeof(contract), RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendMotionDetectionStart(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Motion detection started");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_MOTION_DETECTION_STARTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendMotionDetectionStop(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Motion detection stopped");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_MOTION_DETECTION_STOPPED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendMotionDetected(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Motion Detected");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_MOTION_DETECTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendScanStart(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Scan started");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_SCAN_STARTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendScanStop(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Scan stopped");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_SCAN_STOPPED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendTransferStart(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Data transfer started");
    return  RF_send(RF_IO_CMD_TX_DATA, _destAddress, RF_MSG_TRANS_STARTED, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendTransferStop(uint16_t _destAddress, uint32_t _timeout)
{
    DEBUG("[NOTI] Data transfer stopped");
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
//    if (WG_getStatus() == WG_STATUS_READY)
    {
        RF_IO_FRAME    frame = {    .cmd = RF_IO_REQ_MOTION_DETECT_START, .length = 0, .crc = 0  };

        return  RF_sendFrame(&frame, _timeout);
    }

    //return  RET_ERROR;
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

