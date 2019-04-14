#ifndef RF_H__
#define RF_H__

typedef struct
{
    uint32_t    bitrate;
}       RF_CONFIG;

RET_VALUE   RF_init(SPI_HandleTypeDef* spi);

RET_VALUE   RF_setConfig(RF_CONFIG* config);
RET_VALUE   RF_getConfig(RF_CONFIG* config);

RET_VALUE   RF_start(void);
void        RF_reset(void);

RET_VALUE   RF_setBitrate(uint32_t bitrate);

uint8_t     RF_readRegister(uint8_t    address);
void        RF_writeRegister(uint8_t    address, uint8_t value);

#endif