#ifndef RF_H__
#define RF_H__

typedef struct
{
    uint16_t    shortAddress;
    uint16_t    power;
    uint32_t    frequency;
    uint32_t    maxPayloadLength;
    uint32_t    timeout;
}   RF_CONFIG;

typedef struct
{
    struct
    {
        uint32_t    bytes;
        uint32_t    packets;
        uint32_t    errors;
    }   Rx;

    struct
    {
        uint32_t    bytes;
        uint32_t    packets;
        uint32_t    errors;
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
    RF_CMD_ACK = 0x01,
    RF_CMD_START_SCAN,
    RF_CMD_STOP_SCAN,
    RF_CMD_REQ_DATA_COUNT,
    RF_CMD_REP_DATA_COUNT,
    RF_CMD_REQ_DATA,
    RF_CMD_REP_DATA,
    RF_CMD_REP_SET_CONFIG,
    RF_CMD_REP_GET_CONFIG,
    RF_CMD_NAK
}   RF_CMD;

#define RF_FRAME_OFFSET_DEST_ADDRESS    0
#define RF_FRAME_OFFSET_SRC_ADDRESS     2
#define RF_FRAME_OFFSET_PORT            4
#define RF_FRAME_OFFSET_OPTIONS         5
#define RF_FRAME_OFFSET_SEQUENCE        6
#define RF_FRAME_OFFSET_PAYLOAD_LENGTH  8
#define RF_FRAME_OFFSET_PAYLOAD_BEGIN   9

#define RF_CC1310_STOP              0x01
#define RF_CC1310_BUSY              0x02
#define RF_CC1310_RX                0x04
#define RF_CC1310_RX_DONE           0x08
#define RF_CC1310_TX                0x10
#define RF_CC1310_TX_DONE           0x20
#define RF_CC1310_ERROR             0x40
#define RF_CC1310_TIMEOUT           0x80

#define RF_SPI_CMD_TX_DATA          0x01
#define RF_SPI_CMD_RX_DATA          0x02

#define RF_SPI_CMD_SET_CONFIG           0x42
#define RF_SPI_CMD_GET_CONFIG           0x41

#define RF_SPI_CMD_START_AUTO_TRANSFER  0x81
#define RF_SPI_CMD_STOP_AUTO_TRANSFER   0x82
#define RF_SPI_CMD_MOTION_DETECTION     0x83

#define RF_PAYLOAD_SIZE_MAX         48


#pragma pack(push, 1)
typedef struct
{
    uint16_t    destAddress;
    uint16_t    srcAddress;
    uint8_t     port;
    uint8_t     options;
    uint8_t     sequence;
    uint8_t     length;
}   RF_FRAME_HEADER;

typedef struct
{
    RF_FRAME_HEADER header;
    uint8_t         payload[RF_PAYLOAD_SIZE_MAX];
}   RF_FRAME;

#define RF_FRAME_SIZE_MAX           sizeof(RF_FRAME)

typedef struct
{
    uint8_t     cmd;
    uint8_t     length;
    uint16_t    crc;
    uint8_t     payload[RF_FRAME_SIZE_MAX];
}   RF_SPI_FRAME;
#pragma pack(pop)

#define RF_SPI_FRAME_SIZE       sizeof(RF_SPI_FRAME)

#define RF_OPTIONS_REQ_ACK      0x01
#define RF_OPTIONS_ACK          0x02

RET_VALUE   RF_init(SPI_HandleTypeDef* _spi);

RET_VALUE   RF_setConfig(RF_CONFIG* _config, uint32_t _timeout);
RET_VALUE   RF_getConfig(RF_CONFIG* _config, uint32_t _timeout);

RET_VALUE   RF_start(void);
RET_VALUE   RF_stop(void);
void        RF_reset(void);

uint32_t    RF_getTimeout(void);

uint16_t    RF_getShortAddress(void);
RET_VALUE   RF_setShortAddress(uint16_t _shortAddress);

RET_VALUE   RF_setMaxPayloadLength(uint32_t _maxPayloadLength);
uint32_t    RF_getMaxPayloadLength(void);


RF_STATUS   RF_getStatus(void);
const char* RF_getStatusString(RF_STATUS _status);


RET_VALUE   RF_sendStartScan(uint16_t _destAddress, bool _reset, uint32_t _timeout);
RET_VALUE   RF_sendStopScan(uint16_t _destAddress, uint32_t _timeout);

RET_VALUE   RF_sendData(uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t _timeout);
RET_VALUE   RF_sendRequestDataCount(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendResponseDataCount(uint16_t _destAddress, uint32_t _count, uint32_t _timeout);
RET_VALUE   RF_recvResponseDataCount(uint16_t* _srcAddress, uint32_t* _count, uint32_t _timeout);
RET_VALUE   RF_sendRequestData(uint16_t _destAddress, uint32_t _offset, uint32_t _count, uint32_t _timeout);
RET_VALUE   RF_sendDummy(void);

RET_VALUE   RF_sendACK(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendNAK(uint16_t _destAddress, uint32_t _timeout);

RET_VALUE   RF_testSend(uint8_t* _buffer, uint32_t _size, uint32_t _interval, uint32_t _count);
RET_VALUE   RF_testRecv(uint32_t _interval, uint32_t _timeout);
RET_VALUE   RF_testStop(void);

RET_VALUE   RF_getStatistics(RF_STATISTICS* _statistics);
RET_VALUE   RF_clearStatistics(void);

RET_VALUE   RF_startAutoTransfer(uint32_t timeout);
RET_VALUE   RF_stopAutoTransfer(uint32_t timeout);
RET_VALUE   RF_startMotionDetection(uint32_t timeout);

#endif