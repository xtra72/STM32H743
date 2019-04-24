#ifndef RF_H__
#define RF_H__

typedef struct
{
    uint32_t    bitrate;
}       RF_CONFIG;

typedef enum
{
    RF_STATUS_STOPPED,
    RF_STATUS_READY,
    RF_STATUS_TEST_SEND,
    RF_STATUS_TEST_RECV,
    RF_STATUS_MAX
}   RF_STATUS;

RET_VALUE   RF_init(SPI_HandleTypeDef* spi);

RET_VALUE   RF_setConfig(RF_CONFIG* config);
RET_VALUE   RF_getConfig(RF_CONFIG* config);

RET_VALUE   RF_start(void);
RET_VALUE   RF_stop(void);
void        RF_reset(void);

RET_VALUE   RF_setBitrate(uint32_t bitrate);
uint32_t    RF_getBitrate(void);

uint8_t     RF_readRegister(uint8_t    address);
void        RF_writeRegister(uint8_t    address, uint8_t value);

RF_STATUS   RF_getStatus(void);
const char* RF_getStatusString(RF_STATUS status);

RET_VALUE   RF_send(uint8_t* buffer, uint32_t size, uint32_t timeout);
RET_VALUE   RF_recv(uint8_t* buffer, uint32_t bufferSize, uint32_t* receivedLength, uint32_t timeout);

RET_VALUE   RF_testSend(uint8_t* buffer, uint32_t size, uint32_t interval, uint32_t count);
RET_VALUE   RF_testRecv(uint32_t interval, uint32_t timeout);
RET_VALUE   RF_testStop(void);

#endif