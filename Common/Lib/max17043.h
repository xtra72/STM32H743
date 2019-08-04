#ifndef MAX17043_H_
#define MAX17043_H_

void    MAX17043_init(void);

RET_VALUE   MAX17043_sleep(void);
RET_VALUE   MAX17043_wakeup(void);
RET_VALUE   MAX17043_getCell(uint16_t* cell);
#endif