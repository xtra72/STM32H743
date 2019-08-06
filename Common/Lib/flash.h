#ifndef FLASH_H_
#define FLASH_H_

RET_VALUE   FLASH_erase(uint32_t _startAddress, uint32_t _endAddress);
RET_VALUE   FLASH_write(uint32_t _address, uint8_t *_data, uint32_t _size);

#endif
