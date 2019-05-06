#include "target.h"
#include "sx1231.h"
#include "rf.h"
#include "crc16.h"
#include "time2.h"
#include "scan.h"
#include "utils.h"

#ifndef RF_BUFFER_SIZE_MAX
#define RF_BUFFER_SIZE_MAX  SX1231_RF_BUFFER_SIZE_MAX
#endif

#define RF_FRAME_OVERHEAD_SIZE  11

#ifndef RF_PAYLOAD_SIZE_MAX
#define RF_PAYLOAD_SIZE_MAX  (RF_BUFFER_SIZE_MAX - RF_FRAME_OVERHEAD_SIZE)
#endif

#define __MODULE_NAME__ "RF"
#include "trace.h"

static  void        RF_taskMain(void const * argument);
static  RET_VALUE   RF_waitingForAck(uint32_t _timeout);

static  SPI_HandleTypeDef*  spi_ = NULL;

static  RF_CONFIG           config_ =
{
    .shortAddress = 0x0000,
    .confirmed = false,
    .bitrate = TARGET_RF_BITRATE,
    .maxPayloadLength = RF_PAYLOAD_SIZE_MAX,
    .timeout = TARGET_RF_TIMEOUT
};

static  osThreadId          threadId_ = NULL;
static  bool                stop_ = true;
static  SemaphoreHandle_t   semaphore_ = NULL;
static  RF_STATUS           status_ = RF_STATUS_STOPPED;
static  uint16_t            upCount_ = 0;
static  bool                ackReceived_ = false;

static  RF_STATISTICS       statistics_ =
{
    .Rx =
    {
        .count = 0,
        .ack = 0
    },
    .Tx =
    {
        .count = 0,
        .ack = 0
    }
};

static  const char*   statusStrings_[] =
{
    "Stopped",      //RF_STATUS_STOPPED
    "Ready",        //RF_STATUS_READY,
    "Send",         //RF_STATUS_SEND,
    "Recv",         //RF_STATUS_RECV,
};

RET_VALUE   RF_init(SPI_HandleTypeDef* spi)
{
    semaphore_ = xSemaphoreCreateMutex();

    spi_ = spi;

    SX1231_init();


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
    uint32_t    dataLength = 0;
    RF_reset();

    SX1231_initRFChip();

    RF_setConfig(&config_);

    status_ = RF_STATUS_READY;
    stop_ = false;

    while(!stop_)
    {
        uint16_t    srcAddress = 0;
        uint8_t     port = 0;
        uint8_t     options = 0;

        if (RF_recv(buffer, sizeof(buffer), &dataLength, &srcAddress, &port, &options, 1000) == RET_OK)
        {
#if 0
            DEBUG("RCVD : %d\n", dataLength);
#else
            DEBUG_DUMP("RCVD", buffer, dataLength, 0);
#endif

            DEBUG("Port : %d\n", port);
            switch(port)
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

                    if (dataLength == 1)
                    {
                        reset = (*buffer != 0);
                    }

                    if (SCAN_start(reset) == RET_OK)
                    {
                        RF_sendACK(srcAddress, config_.timeout);
                    }

                }
                break;

            case    RF_CMD_STOP_SCAN:
                {
                    if (SCAN_stop() == RET_OK)
                    {
                        RF_sendACK(srcAddress, config_.timeout);
                    }
                }
                break;
#endif
            case    RF_CMD_REP_DATA_COUNT:
                {
                    DEBUG("Data Count : %d\n", ntohUint32((*(uint32_t *)buffer)));
                    ackReceived_ = true;
                }
                break;

#if SWG_HOST
            case    RF_CMD_REP_DATA:
                {
                    char    output[128];
                    uint32_t    outputLength = 0;
                    uint32_t    offset = 0;
                    uint32_t    time  = 0;

                    offset  = ntohUint32((*(uint32_t *)&buffer[0]));
                    time    = ntohUint32((*(uint32_t *)&buffer[4]));
                    outputLength = snprintf(output, sizeof(output), "%04x,%d,%d", srcAddress, offset, time);

                    for(uint32_t j =  8 ; j <  dataLength ; j+=2)
                    {
                        uint16_t value = ntohUint16((*(uint16_t *)&buffer[j]));
                        outputLength += snprintf(&output[outputLength], sizeof(output) - outputLength, ",%d", value);
                    }

                    COM_printf("+DATA: %s\n", output);
                }
                break;
#endif
#if SWG_DEVICE
            case    RF_CMD_REQ_DATA_COUNT:
                {
                    RET_VALUE   ret;

                    ret = RF_sendResponseDataCount(srcAddress, SCAN_getCurrentLoop(), config_.timeout);
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

                    RF_sendACK(srcAddress, config_.timeout);
                    osDelay(config_.timeout);

                    offset = ntohUint32(((uint32_t*)buffer)[0]);
                    count = ntohUint32(((uint32_t*)buffer)[1]);
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
                        buffer[length++] = (data->time >> 24) & 0xFF;
                        buffer[length++] = (data->time >> 16) & 0xFF;
                        buffer[length++] = (data->time >>  8) & 0xFF;
                        buffer[length++] = (data->time      ) & 0xFF;

                        data = SCAN_getLoopData(offset + i*2);
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

                        ret = RF_send(srcAddress, RF_CMD_REP_DATA, buffer, length, 0, 1000);
                        if (ret != RET_OK)
                        {
                            DEBUG("Data send failed[%d/%d]\n", i, count);
                        }
                    }
                }
                break;
#endif
            default:
                DEBUG("Unknown CMD : %02x\n", port);
            }
        }

        osDelay(1);
    }

    stop_ = true;
    status_ = RF_STATUS_STOPPED;

}


typedef struct
{
    uint32_t    speed;
    struct
    {
        uint8_t     msb;
        uint8_t     lsb;
    }   bitrate;
    struct
    {
        uint8_t     msb;
        uint8_t     lsb;
    }   fdev;

}   RF_BIT_RATE;

RF_BIT_RATE bitrateTables [] =
{
    {   1200,   SX1231_RF_BITRATELSB_1200,  SX1231_RF_BITRATELSB_1200,  SX1231_RF_FDEVMSB_2000,     SX1231_RF_FDEVLSB_2000},
    {   2400,   SX1231_RF_BITRATEMSB_2400,  SX1231_RF_BITRATELSB_2400,  SX1231_RF_FDEVMSB_5000,     SX1231_RF_FDEVLSB_5000},
    {   4800,   SX1231_RF_BITRATEMSB_4800,  SX1231_RF_BITRATELSB_4800,  SX1231_RF_FDEVMSB_5000,     SX1231_RF_FDEVLSB_5000},
    {   9600,   SX1231_RF_BITRATEMSB_9600,  SX1231_RF_BITRATELSB_9600,  SX1231_RF_FDEVMSB_10000,    SX1231_RF_FDEVLSB_10000},
    {   12500,  SX1231_RF_BITRATEMSB_12500, SX1231_RF_BITRATELSB_12500, SX1231_RF_FDEVMSB_15000,    SX1231_RF_FDEVLSB_15000},
    {   19200,  SX1231_RF_BITRATEMSB_19200, SX1231_RF_BITRATELSB_19200, SX1231_RF_FDEVMSB_20000,    SX1231_RF_FDEVLSB_20000},
    {   25000,  SX1231_RF_BITRATEMSB_25000, SX1231_RF_BITRATELSB_25000, SX1231_RF_FDEVMSB_25000,    SX1231_RF_FDEVLSB_25000},
    {   32768,  SX1231_RF_BITRATEMSB_32768, SX1231_RF_BITRATELSB_32768, SX1231_RF_FDEVMSB_35000,    SX1231_RF_FDEVLSB_35000},
    {   38400,  SX1231_RF_BITRATEMSB_38400, SX1231_RF_BITRATELSB_38400, SX1231_RF_FDEVMSB_40000,    SX1231_RF_FDEVLSB_40000},
    {   50000,  SX1231_RF_BITRATEMSB_50000, SX1231_RF_BITRATELSB_50000, SX1231_RF_FDEVMSB_50000,    SX1231_RF_FDEVLSB_50000},
    {   57600,  SX1231_RF_BITRATEMSB_57600, SX1231_RF_BITRATELSB_57600, SX1231_RF_FDEVMSB_60000,    SX1231_RF_FDEVLSB_60000},
    {   76800,  SX1231_RF_BITRATEMSB_76800, SX1231_RF_BITRATELSB_76800, SX1231_RF_FDEVMSB_80000,    SX1231_RF_FDEVLSB_80000},
    {   100000, SX1231_RF_BITRATEMSB_100000,SX1231_RF_BITRATELSB_100000,SX1231_RF_FDEVMSB_100000 ,  SX1231_RF_FDEVLSB_100000},
    {   115200, SX1231_RF_BITRATEMSB_115200,SX1231_RF_BITRATELSB_115200,SX1231_RF_FDEVMSB_120000,   SX1231_RF_FDEVLSB_120000},
    {   150000, SX1231_RF_BITRATEMSB_150000,SX1231_RF_BITRATELSB_150000,SX1231_RF_FDEVMSB_150000,   SX1231_RF_FDEVLSB_150000},
    {   153600, SX1231_RF_BITRATEMSB_153600,SX1231_RF_BITRATELSB_153600,SX1231_RF_FDEVMSB_160000,   SX1231_RF_FDEVLSB_160000},
    {   200000, SX1231_RF_BITRATEMSB_200000,SX1231_RF_BITRATELSB_200000,SX1231_RF_FDEVMSB_200000,   SX1231_RF_FDEVLSB_200000},
    {   250000, SX1231_RF_BITRATEMSB_250000,SX1231_RF_BITRATELSB_250000,SX1231_RF_FDEVMSB_250000,   SX1231_RF_FDEVLSB_250000},
    {   300000, SX1231_RF_BITRATEMSB_300000,SX1231_RF_BITRATELSB_300000,SX1231_RF_FDEVMSB_300000,   SX1231_RF_FDEVLSB_300000},
    {   0,      0,                          0}
};

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

RET_VALUE   RF_setConfig(RF_CONFIG* config)
{
    ASSERT(config);

    RF_setConfirmed(config->confirmed);
    RF_setBitrate(config->bitrate);
    RF_setMaxPayloadLength(config->maxPayloadLength);

    memcpy(&config_, config, sizeof(RF_CONFIG));

    return  RET_OK;
}

RET_VALUE   RF_getConfig(RF_CONFIG* config)
{
    ASSERT(config);

    memcpy(config, &config_, sizeof(RF_CONFIG));

    return  RET_OK;
}

RET_VALUE   RF_setConfirmed(bool confirmed)
{
    config_.confirmed = confirmed;

    return  RET_OK;
}

bool    RF_getConfirmed(void)
{
    return  config_.confirmed;
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

RET_VALUE   RF_setBitrate(uint32_t bitrate)
{
    for(int i = 1 ; bitrateTables[i].speed != 0; i++)
    {
        if (bitrate < bitrateTables[i].speed)
        {
            if (threadId_)
            {
                SX1231_writeRegister(SX1231_REG_BITRATEMSB, bitrateTables[i-1].bitrate.msb);
                SX1231_writeRegister(SX1231_REG_BITRATELSB, bitrateTables[i-1].bitrate.lsb);
//                SX1231_writeRegister(SX1231_REG_FDEVMSB, bitrateTables[i-1].fdev.msb);
//                SX1231_writeRegister(SX1231_REG_FDEVLSB, bitrateTables[i-1].fdev.lsb);
            }
            config_.bitrate = bitrateTables[i - 1].speed;

            return  RET_OK;
        }
    }

    return  RET_OUT_OF_RANGE;
}

uint32_t    RF_getBitrate(void)
{
    return  config_.bitrate;
}

uint8_t RF_readRegister(uint8_t    address)
{
    uint8_t value = 0;

    value = SX1231_readRegister(address);

    return  value;
}

void    RF_writeRegister(uint8_t    address, uint8_t value)
{
    SX1231_writeRegister(address, value);
}

void    RF_reset(void)
{
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_RESET);
    osDelay(5);
}

RET_VALUE   RF_send(uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t timeout)
{
    uint8_t ret = RET_ERROR;
    TickType_t  tickStart  = xTaskGetTickCount();
    TickType_t  tickTimeout = timeout / portTICK_PERIOD_MS;
    uint8_t     frameBuffer[RF_BUFFER_SIZE_MAX];
    uint32_t    frameLength = 0;

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

        SX1231_receiveCancel();
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

    frameBuffer[frameLength++] = (_destAddress >> 8) & 0xFF;
    frameBuffer[frameLength++] = (_destAddress     ) & 0xFF;
    frameBuffer[frameLength++] = (config_.shortAddress >> 8) & 0xFF;
    frameBuffer[frameLength++] = (config_.shortAddress     ) & 0xFF;
    frameBuffer[frameLength++] = _port;
    frameBuffer[frameLength++] = _options;
    frameBuffer[frameLength++] = (upCount_ >> 8) & 0xFF;
    frameBuffer[frameLength++] = (upCount_     ) & 0xFF;
    frameBuffer[frameLength++] = _dataSize;
    if (_dataSize != 0)
    {
        memcpy(&frameBuffer[frameLength], _data, _dataSize);
    }
    frameLength += _dataSize;

    uint16_t    crc = CRC16_calc(frameBuffer, frameLength);
    frameBuffer[frameLength++] = (crc >> 8) & 0xFF;
    frameBuffer[frameLength++] = (crc     ) & 0xFF;

    ret = SX1231_sendFrame(frameBuffer, frameLength, tickTimeout);
    xSemaphoreGive( semaphore_ );
    if (ret == RET_OK)
    {
        statistics_.Tx.count++;

        if (frameLength != 0)
        {
            DEBUG_DUMP("SEND", frameBuffer, frameLength, 0);
        }
        else
        {
            statistics_.Tx.ack++;
            DEBUG("ACK");
        }
    }
    else
    {
        DEBUG("send failed!\n");
    }

    return  ret;
}

RET_VALUE   RF_sendStartScan(uint16_t _destAddress, bool _reset, uint32_t _timeout)
{
    RET_VALUE   ret;
    uint32_t    expireTick;

    expireTick = TICK_get() + _timeout;

    TRACE("Send Start Scan\n");
    ret = RF_send(_destAddress, RF_CMD_START_SCAN, (uint8_t*)&_reset, 1, RF_OPTIONS_ACK, _timeout);
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
    ret = RF_send(_destAddress, RF_CMD_STOP_SCAN, NULL, 0, RF_OPTIONS_ACK, _timeout);
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
    ret = RF_send(_destAddress, RF_CMD_REQ_DATA_COUNT, NULL, 0, RF_OPTIONS_ACK, _timeout);
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


    return  RF_send(_destAddress, RF_CMD_REP_DATA_COUNT, buffer, length, 0, _timeout);
}

RET_VALUE   RF_recvResponseDataCount(uint16_t* _srcAddress, uint32_t* _count, uint32_t _timeout)
{
    RET_VALUE   ret;
    uint8_t     buffer[RF_PAYLOAD_SIZE_MAX];
    uint32_t    receivedLength = 0;
    uint8_t     port;
    uint8_t     options;

    ret = RF_recv(buffer, sizeof(buffer), &receivedLength,  _srcAddress, &port, &options, _timeout);
    if (ret == RET_OK)
    {
        if (port == RF_CMD_REP_DATA_COUNT)
        {
            if (receivedLength != 4)
            {
                ret = RET_INVALID_FRAME;
            }
            else
            {
                *_count = ntohUint32(*(uint32_t *)buffer);
            }
        }
    }

    return  ret;
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

    ret = RF_send(_destAddress, RF_CMD_REQ_DATA, buffer, length, RF_OPTIONS_REQ_ACK, _timeout);
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
    return  RF_send(_destAddress, RF_CMD_ACK, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_sendNAK(uint16_t _destAddress, uint32_t _timeout)
{
    return  RF_send(_destAddress, RF_CMD_NAK, NULL, 0, RF_OPTIONS_ACK, _timeout);
}

RET_VALUE   RF_recv(uint8_t* buffer, uint32_t bufferSize, uint32_t* receivedLength,  uint16_t* _srcAddress, uint8_t* _port, uint8_t* _options, uint32_t timeout)
{
    uint8_t ret = ERROR;
    TickType_t  tickStart  = TICK_get();
    TickType_t  tickTimeout = timeout / portTICK_PERIOD_MS;
    uint8_t     frame[RF_BUFFER_SIZE_MAX];
    uint32_t    frameLength;


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

    ret  = SX1231_receiveFrame(frame, RF_BUFFER_SIZE_MAX, &frameLength, tickTimeout * portTICK_PERIOD_MS);
    xSemaphoreGive( semaphore_ );
    if (ret == RET_OK)
    {
        if (frameLength < RF_FRAME_OVERHEAD_SIZE)
        {
            DEBUG("Too short frame received[frameLength = %d].\n", frameLength);
            ret = RET_FRAME_TOO_SHORT;
        }
        else
        {
            uint16_t    destAddress = (((uint16_t)frame[RF_FRAME_OFFSET_DEST_ADDRESS] << 8) | frame[RF_FRAME_OFFSET_DEST_ADDRESS + 1]);
            if (config_.shortAddress != destAddress)
            {
                DEBUG("Invalid address [%04x != %04x].\n", config_.shortAddress, destAddress);
                ret = RET_IGNORE_FARME;
            }
            else
                {
                uint16_t    crc = CRC16_calc(frame, frameLength - 2);
                uint16_t    inFrameCrc = (((uint16_t)frame[frameLength - 2] << 8) + frame[frameLength - 1]);
                if (crc != inFrameCrc)
                {
                    DEBUG("CRC does not match.[ Calc : %04x, Pkt : %04x].\n", crc, inFrameCrc);
                    ret = RET_FRAME_INVALID;
                }
                else
                {

                    if (frame[RF_FRAME_OFFSET_PAYLOAD_LENGTH] != (frameLength - RF_FRAME_OVERHEAD_SIZE))
                    {
                        DEBUG("Invalid frame length.[ Received : %d, In Frame : %d].\n", frameLength - RF_FRAME_OVERHEAD_SIZE, (uint32_t)frame[3]);
                        ret = RET_FRAME_INVALID;
                    }
                    else
                    {

                        *receivedLength = frameLength - RF_FRAME_OVERHEAD_SIZE;
                        memcpy(buffer, &frame[RF_FRAME_OFFSET_PAYLOAD_BEGIN], frameLength - RF_FRAME_OVERHEAD_SIZE);

                        if (_srcAddress)
                        {
                            *_srcAddress = ((uint16_t)frame[RF_FRAME_OFFSET_SRC_ADDRESS] << 8) | frame[RF_FRAME_OFFSET_SRC_ADDRESS + 1];
                        }

                        if (_port)
                        {
                            *_port = frame[RF_FRAME_OFFSET_PORT];
                        }

                        if (_options)
                        {
                            *_options = frame[RF_FRAME_OFFSET_OPTIONS];
                        }

                        statistics_.Rx.count++;
                        ret = RET_OK;
                    }
                }
            }
        }
    }


    return  ret;
}

/////////////////////////////////////////////////////////////////////////////
// SX1231 Control Functions
/////////////////////////////////////////////////////////////////////////////
void SX1231_wait(uint32_t usecs)
{
    uint32_t    msecs = (usecs + 999) / 1000;

    osDelay(msecs);
}

bool SX1231_getDIO0(void)
{
    return  HAL_GPIO_ReadPin(RF_DIO0_GPIO_Port, RF_DIO0_Pin) == GPIO_PIN_SET;
}

void   SX1231_SPI_select(bool   select)
{
    if (select)
    {
        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
    }
}

bool    SX1231_SPI_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        return  (HAL_SPI_Transmit(spi_, buffer, size, timeout) == HAL_OK);
    }

    return  false;
}

bool    SX1231_SPI_receive(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        return  (HAL_SPI_Receive(spi_, buffer, size, timeout) == HAL_OK);
    }

    return  false;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(SX1231_getPreviousMode())
    {
    case    SX1231_RF_TRANSMITTER:
        {
        }
        break;

    case    SX1231_RF_RECEIVER:
        {
            SX1231_receiveCallback();
        }
        break;
    }
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