#ifndef SPI_H_
#define SPI_H_

RET_VALUE   SPI_init(SPI_HandleTypeDef* _spi);

RET_VALUE   SPI_receive(uint8_t* buffer, uint32_t size, uint32_t timeout);
RET_VALUE   SPI_transmit(uint8_t* buffer, uint32_t size, uint32_t _timeout);
RET_VALUE   SPI_transmitReceive(uint8_t* _txBuffer, uint8_t* _rxBuffer, uint32_t _size, uint32_t _timeout);

#endif
