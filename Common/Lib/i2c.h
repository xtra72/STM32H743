#ifndef I2C_H_
#define I2C_H_

RET_VALUE   I2C_init(I2C_HandleTypeDef* _spi);

RET_VALUE   I2C_receive(uint8_t address, uint8_t* buffer, uint32_t size, uint32_t timeout);
RET_VALUE   I2C_transmit(uint8_t address, uint8_t* buffer, uint32_t size, uint32_t _timeout);

#endif
