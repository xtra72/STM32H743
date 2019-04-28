#ifndef RF_H__
#define RF_H__

typedef struct
{
    uint16_t    shortAddress;
    bool        enable;
    bool        confirmed;
    uint32_t    bitrate;
    uint32_t    maxPayloadLength;
}   RF_CONFIG;

typedef struct
{
    struct
    {
        uint32_t    count;
        uint32_t    ack;
    }   Rx;

    struct
    {
        uint32_t    count;
        uint32_t    ack;
    }   Tx;
}   RF_STATISTICS;

typedef enum
{
    RF_STATUS_STOPPED,
    RF_STATUS_READY,
    RF_STATUS_SEND,
    RF_STATUS_RECV,
    RF_STATUS_MAX
}   RF_STATUS;

typedef enum
{
    RF_CMD_START_SCAN = 0x01,
    RF_CMD_STOP_SCAN,
    RF_CMD_GET_DATA_COUNT,
    RF_CMD_GET_DATA
}   RF_CMD;

#define RF_FRAME_OFFSET_DEST_ADDRESS 0
#define RF_FRAME_OFFSET_SRC_ADDRESS 2
#define RF_FRAME_OFFSET_PORT        4
#define RF_FRAME_OFFSET_OPTIONS 5
#define RF_FRAME_OFFSET_PAYLOAD_LENGTH  8
#define RF_FRAME_OFFSET_PAYLOAD_BEGIN   9

RET_VALUE   RF_init(SPI_HandleTypeDef* _spi);

RET_VALUE   RF_setConfig(RF_CONFIG* _config);
RET_VALUE   RF_getConfig(RF_CONFIG* _config);

RET_VALUE   RF_start(void);
RET_VALUE   RF_stop(void);
void        RF_reset(void);

RET_VALUE   RF_setConfirmed(bool confirmed);
bool        RF_getConfirmed(void);

uint16_t    RF_getShortAddress(void);
RET_VALUE   RF_setShortAddress(uint16_t _shortAddress);

RET_VALUE   RF_setBitrate(uint32_t _bitrate);
uint32_t    RF_getBitrate(void);

RET_VALUE   RF_setMaxPayloadLength(uint32_t _maxPayloadLength);
uint32_t    RF_getMaxPayloadLength(void);

uint8_t     RF_readRegister(uint8_t    _address);
void        RF_writeRegister(uint8_t    _address, uint8_t _value);

RF_STATUS   RF_getStatus(void);
const char* RF_getStatusString(RF_STATUS _status);

RET_VALUE   RF_send(uint16_t _destAddress, uint8_t _port, uint8_t* _buffer, uint32_t _size, uint8_t _options, uint32_t _timeout);
RET_VALUE   RF_recv(uint8_t* _buffer, uint32_t _bufferSize, uint32_t* _receivedLength, uint16_t* _srcAddress, uint8_t* _port, uint8_t* _options, uint32_t _timeout);

RET_VALUE   RF_testSend(uint8_t* _buffer, uint32_t _size, uint32_t _interval, uint32_t _count);
RET_VALUE   RF_testRecv(uint32_t _interval, uint32_t _timeout);
RET_VALUE   RF_testStop(void);

RET_VALUE   RF_getStatistics(RF_STATISTICS* _statistics);
RET_VALUE   RF_clearStatistics(void);

#endif