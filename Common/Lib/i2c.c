#include "target.h"
#include "i2c.h"
#include "time2.h"

#define __MODULE_NAME__ "RF"
#include "trace.h"

static  I2C_HandleTypeDef*  i2c_ = NULL;
static  SemaphoreHandle_t   ioSemaphore_ = NULL;

RET_VALUE   I2C_init(I2C_HandleTypeDef* _i2c)
{
    i2c_ = _i2c;

    ioSemaphore_ = xSemaphoreCreateBinary();

    return  RET_OK;
}

RET_VALUE   I2C_receive(uint8_t address, uint8_t* _buffer, uint32_t _size, uint32_t _timeout)
{

    ASSERT(i2c_ != NULL);
    uint32_t    startTime = TICK_get();

    if (HAL_I2C_Master_Receive(i2c_, (address << 1) | 0x01, _buffer, _size, 1000) == HAL_OK)
    {
        return  RET_OK;
    }

    return  RET_ERROR;
}

RET_VALUE   I2C_transmit(uint8_t address, uint8_t* _buffer, uint32_t _size, uint32_t _timeout)
{
    ASSERT(i2c_ != NULL);
    uint32_t    startTime = TICK_get();

    if (HAL_I2C_Master_Transmit(i2c_, (address << 1), _buffer, _size, 1000) == HAL_OK)
    {
        return  RET_OK;
    }

    return  RET_ERROR;
}

