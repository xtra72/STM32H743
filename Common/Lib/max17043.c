#include "target.h"
#include "i2c.h"
#include "time2.h"

#define __MODULE_NAME__ "MAX17043"
#include "trace.h"

#define MAX17043_ADDRESS    0x36

RET_VALUE   MAX17043_readRegister(uint8_t address, uint16_t* value);
RET_VALUE   MAX17043_writeRegister(uint8_t address, uint16_t value);

RET_VALUE   MAX17043_sleep(void)
{
    uint16_t    value;

    if (MAX17043_readRegister(0x0C, &value) != RET_OK)
    {
        TRACE("Read failed!\n");
        return  RET_ERROR;
    }

    value |= 0x80;

    if (MAX17043_writeRegister(0x0C, value) != RET_OK)
    {
        TRACE("Write failed!\n");
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   MAX17043_wakeup(void)
{
    uint16_t    value;

    if (MAX17043_readRegister(0x0C, &value) != RET_OK)
    {
        TRACE("Read failed!\n");
        return  RET_ERROR;
    }

    value &= ~0x0080;

    if (MAX17043_writeRegister(0x0C, value) != RET_OK)
    {
        TRACE("Write failed!\n");
        return  RET_ERROR;
    }

    return  RET_OK;
}


RET_VALUE   MAX17043_getCell(uint16_t* cell)
{
    if (MAX17043_wakeup() != RET_OK)
    {
        TRACE("Wakeup failed!\n");
        return  RET_ERROR;
    }
    osDelay(1);
    if (MAX17043_readRegister(0x02, cell) != RET_OK)
    {
        TRACE("Read register failed!\n");
    }

    if (MAX17043_sleep() != RET_OK)
    {
        TRACE("Wakeup failed!\n");
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   MAX17043_readRegister(uint8_t address, uint16_t* value)
{
    uint8_t     buffer[2] = {0, };

    if (I2C_transmit(MAX17043_ADDRESS, &address, 1, 100) != RET_OK)
    {
        TRACE("I2C transmit failed!\n");
        return  RET_ERROR;
    }

    if (I2C_receive(MAX17043_ADDRESS, buffer, 2, 100) != RET_OK)
    {
        TRACE("I2C transmit failed!\n");
        return  RET_ERROR;
    }

    *value = (uint16_t)((((uint16_t)buffer[0] << 4) | ((uint16_t)buffer[1] >> 4)) * 1.25);

    return  RET_OK;
}

RET_VALUE   MAX17043_writeRegister(uint8_t address, uint16_t value)
{
    uint8_t     buffer[3] = {0, };

    buffer[0] = address;
    buffer[1] = ((value >> 8) & 0xFF);
    buffer[2] = (value & 0xFF);

    if (I2C_transmit(MAX17043_ADDRESS, buffer, 3, 100) != RET_OK)
    {
        TRACE("I2C transmit failed!\n");
        return  RET_ERROR;
    }

    return  RET_OK;
}
