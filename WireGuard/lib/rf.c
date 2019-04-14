#include "target.h"
#include "sx1231.h"
#include "rf.h"

void    RF_reset(void);
void    RF_selectSignal(bool select);
void    RF_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout);
void    RF_receive(uint8_t* buffer, uint32_t size, uint32_t timeout);

static  SX1231_CONFIG   _config =
{
    .select = RF_selectSignal,
    .reset = RF_reset,
    .transmit = RF_transmit,
    .receive = RF_receive
};

static  SPI_HandleTypeDef*  spi_ = NULL;

static  uint32_t    bitrate_ = 300000;

RET_VALUE   RF_init(SPI_HandleTypeDef* spi)
{
    spi_ = spi;

    SX1231_init(&_config);

    return  RET_OK;
}

RET_VALUE   RF_setConfig(RF_CONFIG* config)
{
    ASSERT(config);

    bitrate_ = config->bitrate;

    return  RET_OK;
}

RET_VALUE   RF_getConfig(RF_CONFIG* config)
{
    ASSERT(config);

    config->bitrate = bitrate_;

    return  RET_OK;
}

RET_VALUE   RF_start(void)
{
    SX1231_initRFChip();

    return  RET_OK;
}

void    RF_reset(void)
{
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_RESET);
    osDelay(5);
}

typedef struct
{
    uint32_t    bitrate;
    uint8_t     msb;
    uint8_t     lsb;
}   RF_BIT_RATE;

RF_BIT_RATE bitrateTables [] =
{
    {   1200,   SX1231_RF_BITRATELSB_1200,  SX1231_RF_BITRATELSB_1200 },
    {   2400,   SX1231_RF_BITRATEMSB_2400,  SX1231_RF_BITRATELSB_2400 },
    {   4800,   SX1231_RF_BITRATEMSB_4800,  SX1231_RF_BITRATELSB_4800 },
    {   9600,   SX1231_RF_BITRATEMSB_9600,  SX1231_RF_BITRATELSB_9600 },
    {   12500,  SX1231_RF_BITRATEMSB_12500, SX1231_RF_BITRATELSB_12500 },
    {   19200,  SX1231_RF_BITRATEMSB_19200, SX1231_RF_BITRATELSB_19200 },
    {   25000,  SX1231_RF_BITRATEMSB_25000, SX1231_RF_BITRATELSB_25000 },
    {   32768,  SX1231_RF_BITRATEMSB_32768, SX1231_RF_BITRATELSB_32768 },
    {   38400,  SX1231_RF_BITRATEMSB_38400, SX1231_RF_BITRATELSB_38400 },
    {   50000,  SX1231_RF_BITRATEMSB_50000, SX1231_RF_BITRATELSB_50000 },
    {   57600,  SX1231_RF_BITRATEMSB_57600, SX1231_RF_BITRATELSB_57600 },
    {   76800,  SX1231_RF_BITRATEMSB_76800, SX1231_RF_BITRATELSB_76800 },
    {   100000, SX1231_RF_BITRATEMSB_100000,SX1231_RF_BITRATELSB_100000 },
    {   115200, SX1231_RF_BITRATEMSB_115200,SX1231_RF_BITRATELSB_115200 },
    {   150000, SX1231_RF_BITRATEMSB_150000,SX1231_RF_BITRATELSB_150000 },
    {   153600, SX1231_RF_BITRATEMSB_153600,SX1231_RF_BITRATELSB_153600 },
    {   200000, SX1231_RF_BITRATEMSB_200000,SX1231_RF_BITRATELSB_200000 },
    {   250000, SX1231_RF_BITRATEMSB_250000,SX1231_RF_BITRATELSB_250000 },
    {   300000, SX1231_RF_BITRATEMSB_300000,SX1231_RF_BITRATELSB_300000 },
    {   0,      0,                          0}
};

RET_VALUE   RF_setBitrate(uint32_t bitrate)
{
    for(int i = 1 ; bitrateTables[i].bitrate != 0; i++)
    {
        if (bitrate < bitrateTables[i].bitrate)
        {
            SX1231_writeRegister(SX1231_REG_BITRATEMSB, bitrateTables[i-1].msb);
            SX1231_writeRegister(SX1231_REG_BITRATELSB, bitrateTables[i-1].lsb);
            bitrate_ = bitrateTables[i].bitrate;

            return  RET_OK;
        }
    }

    SX1231_writeRegister(SX1231_REG_BITRATEMSB, SX1231_RF_BITRATEMSB_300000);
    SX1231_writeRegister(SX1231_REG_BITRATELSB, SX1231_RF_BITRATELSB_300000);
    bitrate_ = 300000;

    return  RET_OK;
}

uint32_t    RF_getBitrate(void)
{
    return  bitrate_;
}

uint8_t RF_readRegister(uint8_t    address)
{
    return  SX1231_readRegister(address);
}

void    RF_writeRegister(uint8_t    address, uint8_t value)
{
    SX1231_writeRegister(address, value);
}

void   RF_selectSignal(bool   select)
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

void    RF_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        HAL_SPI_Transmit(spi_, buffer, size, timeout);
    }
}

void    RF_receive(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        HAL_SPI_Receive(spi_, buffer, size, timeout);
    }
}
