#ifndef RF_H__
#define RF_H__

#include "target.h"

typedef struct
{
    uint16_t    shortAddress;
     int16_t    power;
    uint32_t    frequency;
    uint32_t    maxPayloadLength;
    uint32_t    timeout;
}   RF_CC1310_CONFIG;

typedef struct
{
    RF_CC1310_CONFIG    cc1310;
    uint32_t            transferInterval;
    uint32_t            keepAlive;
    uint32_t            nop;
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
    RF_STATUS_INIT,
    RF_STATUS_WAITING_FOR_CONTRACT,
    RF_STATUS_READY,
    RF_STATUS_MOTION_DETECTION,
    RF_STATUS_MOTION_DETECTED,
    RF_STATUS_SCAN,
    RF_STATUS_SLEEP,
    RF_STATUS_MAX
}   RF_STATUS;

typedef struct
{
    char        deviceId[TARGET_DEVICE_ID_LEN];
    uint8_t     channelCuont;
}   RF_REQUEST_CONTRACT;

#define RF_MSG_DATA                         1
#define RF_MSG_ACK                          2
#define RF_MSG_CONTRACT_REQUEST             4
#define RF_MSG_CONTRACT_CONFIRM             5
#define RF_MSG_MOTION_DETECTION_START       6
#define RF_MSG_MOTION_DETECTION_STARTED     7
#define RF_MSG_MOTION_DETECTION_ALREADY_STARTED     8
#define RF_MSG_MOTION_DETECTION_STOP        8
#define RF_MSG_MOTION_DETECTION_STOPPED     10
#define RF_MSG_MOTION_DETECTION_ALREADY_STOPPED     11
#define RF_MSG_MOTION_DETECTED              12
#define RF_MSG_SCAN_START                   13
#define RF_MSG_SCAN_STARTED                 14
#define RF_MSG_SCAN_ALREADY_STARTED                 15
#define RF_MSG_SCAN_STOP                    16
#define RF_MSG_SCAN_STOPPED                 17
#define RF_MSG_SCAN_ALREADY_STOPPED                 18
#define RF_MSG_TRANS_START                  19
#define RF_MSG_TRANS_STARTED                20
#define RF_MSG_TRANS_ALREADY_STARTED                21
#define RF_MSG_TRANS_STOP                   22
#define RF_MSG_TRANS_STOPPED                23
#define RF_MSG_TRANS_ALREADY_STOPPED        24
#define RF_MSG_SLEEP                        25
#define RF_MSG_STOP                         26

#define RF_CMD                              0x00
#define RF_DATA                             (RF_CMD + 1)
#define RF_REQ_PING                         (RF_CMD + 3)
#define RF_REQ_CONTRACT                     (RF_CMD + 4)

#define RF_REPLY_TO_SERVER                  (0x20)
#define RF_REP_SRV_MOTION_DETECTION_START   (RF_REPLY_TO_SERVER + 1)
#define RF_REP_SRV_MOTION_DETECTION_STOP    (RF_REPLY_TO_SERVER + 2)
#define RF_REP_SRV_SCAN_START               (RF_REPLY_TO_SERVER + 3)
#define RF_REP_SRV_SCAN_STOP                (RF_REPLY_TO_SERVER + 4)
#define RF_REP_SRV_TRANSFER_START           (RF_REPLY_TO_SERVER + 5)
#define RF_REP_SRV_TRANSFER_STOP            (RF_REPLY_TO_SERVER + 6)
#define RF_REP_SRV_DATA_COUNT               (RF_REPLY_TO_SERVER + 7)

#define RF_REPLY                            (0x80)
#define RF_REP_ACK                          (RF_REPLY + 1)
#define RF_REP_NACK                         (RF_REPLY + 2)
#define RF_REP_PING                         (RF_REPLY + 3)
#define RF_REP_CONTRACT                     (RF_REPLY + 4)
#define RF_DOWNLINK                         (RF_REPLY + 5)

#define RF_NOTIFY                           (0x90)
#define RF_NOTI_MOTION_DETECTED             (RF_NOTIFY + 1)

#define RF_REQUEST_FROM_SERVER              (0xA0)
#define RF_REQ_SRV_MOTION_DETECTION_START   (RF_REQUEST_FROM_SERVER + 1)
#define RF_REQ_SRV_MOTION_DETECTION_STOP    (RF_REQUEST_FROM_SERVER + 2)
#define RF_REQ_SRV_SCAN_START               (RF_REQUEST_FROM_SERVER + 3)
#define RF_REQ_SRV_SCAN_STOP                (RF_REQUEST_FROM_SERVER + 4)
#define RF_REQ_SRV_TRANSFER_START           (RF_REQUEST_FROM_SERVER + 5)
#define RF_REQ_SRV_TRANSFER_STOP            (RF_REQUEST_FROM_SERVER + 6)
#define RF_REQ_SRV_REQ_DATA_COUNT           (RF_REQUEST_FROM_SERVER + 7)
#define RF_REQ_SRV_SLEEP                    (RF_REQUEST_FROM_SERVER + 8)

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

#define RF_IO_CMD_TX_DATA           0x01
#define RF_SPI_CMD_RX_DATA          0x02

#define RF_SPI_CMD_REQUEST_GET_CONFIG       0x11
#define RF_SPI_CMD_REQUEST_SET_CONFIG       0x12

#define RF_IO_REQ_MOTION_DETECT_START       0x21
#define RF_IO_REQ_MOTION_DETECT_STOP        0x22
#define RF_SPI_CMD_SLEEP                    0x23

#define RF_SPI_CMD_KEEP_ALIVE               0x6F

#define RF_IO_DOWNLINK                      0x84

#define RF_IO_REP_GET_CONFIG                0x91

#define RF_IO_REP_GET_CONFIG                0x91
#define RF_IO_REP_SET_CONFIG                0x92

#define RF_IO_REP_MOTION_DETECT_STARTED     0xA1
#define RF_IO_REP_MOTION_DETECT_STOPPED     0xA2
#define RF_IO_NOTI_MOTION_DETECTED          0xA3
#define RF_IO_NOTI_FROM_SERVER              0xA4

#define RF_IO_STATUS                        0xEF

#define RF_PAYLOAD_SIZE_MAX                 64


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

typedef struct
{
    uint32_t    mid;
    uint32_t    timeStamp;
}   RF_RESPONSE_CONTRACT;

typedef struct
{
    uint32_t    mid;
}   RF_MOTION_DETECTION;

typedef struct
{
    uint32_t    mid;
}   RF_MOTION_DETECTION_STARTED;

typedef struct
{
    uint32_t    mid;
}   RF_MOTION_DETECTION_STOPED;

typedef struct
{
    uint32_t    mid;
}   RF_REQ_SCAN_PARAMS;

typedef struct
{
    uint32_t    mid;
}   RF_SCAN_STARTED;

typedef struct
{
    uint32_t    mid;
}   RF_SCAN_STOPED;


typedef struct
{
    uint32_t    mid;
    uint32_t     time;
}   RF_SLEEP;

#define RF_FRAME_SIZE_MAX           sizeof(RF_FRAME)

typedef struct
{
    uint8_t     cmd;
    uint8_t     length;
    uint16_t    crc;
    uint8_t     result;
    uint8_t     status;
    uint8_t     qsize;
    uint8_t     reserved;
    uint8_t     payload[RF_FRAME_SIZE_MAX];
}   RF_IO_FRAME;


typedef struct
{
    uint32_t    time;
}   RF_IO_REQUEST_MOTION_DETECT_PARAMS;

typedef struct
{
    uint32_t    time;
}   RF_IO_NOTI_MOTION_DETECT_PARAMS;

typedef struct
{
    uint32_t    cmd;
}   RF_IO_REQUEST_FROM_SLAVE_PARAMS;


#pragma pack(pop)

#define RF_SPI_FRAME_SIZE       sizeof(RF_IO_FRAME)

#define RF_OPTIONS_REQ_ACK      0x01
#define RF_OPTIONS_ACK          0x02

RET_VALUE   RF_init(SPI_HandleTypeDef* _spi);

RET_VALUE   RF_start(void);
RET_VALUE   RF_stop(void);
void        RF_reset(void);

RET_VALUE   RF_setConfig(RF_CONFIG* _config);
RET_VALUE   RF_getConfig(RF_CONFIG* _config);

uint32_t    RF_getTimeout(void);

uint16_t    RF_getShortAddress(void);
RET_VALUE   RF_setShortAddress(uint16_t _shortAddress);

RET_VALUE   RF_setFrequency(uint32_t _frequency);
uint32_t    RF_getFrequency(void);

RET_VALUE   RF_setPower(int16_t _frequency);
int16_t     RF_getPower(void);

RET_VALUE   RF_setMaxPayloadLength(uint32_t _maxPayloadLength);
uint32_t    RF_getMaxPayloadLength(void);

bool        RF_setStatus(RF_STATUS _status);
RF_STATUS   RF_getStatus(void);
const char* RF_getStatusString(RF_STATUS _status);

RET_VALUE   RF_CC1310_writeConfig(uint32_t _timeout);
RET_VALUE   RF_CC1310_readConfig(uint32_t _timeout);

RET_VALUE   RF_sendStartScan(uint16_t _destAddress, bool _reset, uint32_t _timeout);
RET_VALUE   RF_sendStopScan(uint16_t _destAddress, uint32_t _timeout);

RET_VALUE   RF_sendData(uint16_t _destAddress, uint8_t* _data, uint32_t _dataSize, uint8_t _options, uint32_t _timeout);
RET_VALUE   RF_sendRequestDataCount(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendResponseDataCount(uint16_t _destAddress, uint32_t _count, uint32_t _timeout);
RET_VALUE   RF_recvResponseDataCount(uint16_t* _srcAddress, uint32_t* _count, uint32_t _timeout);
RET_VALUE   RF_sendRequestData(uint16_t _destAddress, uint32_t _offset, uint32_t _count, uint32_t _timeout);
RET_VALUE   RF_sendMotionDetectionStart(uint16_t _destAddress, uint32_t _timeout);;
RET_VALUE   RF_sendMotionDetectionStop(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendScanStart(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendScanStop(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendTransferStart(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendTransferStop(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendDummy(void);

RET_VALUE   RF_sendACK(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendNAK(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendPing(uint16_t _destAddress, uint32_t _timeout);
RET_VALUE   RF_sendContract(uint16_t _destAddress, char* _deviceId, uint8_t _channelCount, uint32_t _timeout);
RET_VALUE   RF_sendMotionDetected(uint16_t _destAddress, uint32_t _timeout);

RET_VALUE   RF_testSend(uint8_t* _buffer, uint32_t _size, uint32_t _interval, uint32_t _count);
RET_VALUE   RF_testRecv(uint32_t _interval, uint32_t _timeout);
RET_VALUE   RF_testStop(void);

RET_VALUE   RF_getStatistics(RF_STATISTICS* _statistics);
RET_VALUE   RF_clearStatistics(void);

RET_VALUE   RF_startAutoTransfer(uint32_t timeout);
RET_VALUE   RF_stopAutoTransfer(uint32_t timeout);
RET_VALUE   RF_motionDetectionStart(uint32_t timeout);
RET_VALUE   RF_motionDetectionStop(uint32_t timeout);

RET_VALUE   RF_startTransferScanData();
RET_VALUE   RF_stopTransferScanData();

uint32_t    RF_getKeepAlive();
RET_VALUE   RF_setKeepAlive(uint32_t _interval);

uint32_t    RF_getTransferInterval();
RET_VALUE   RF_setTransferInterval(uint32_t _interval);

uint32_t    RF_getTransferNOP();
RET_VALUE   RF_setTransferNOP(uint32_t _nop);

RET_VALUE   RF_makeFrame(RF_IO_FRAME* _frame, uint8_t cmd, uint16_t _destAddress, uint8_t _port, uint8_t* _data, uint32_t _dataSize, uint8_t _options);



#endif