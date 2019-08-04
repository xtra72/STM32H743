#ifndef SYS_H_
#define SYS_H_

#include "target.h"

void        SYS_init(void);
RET_VALUE   SYS_sleep(uint32_t sleepTime);
void        SYS_reset(void);
uint8_t     SYS_getModelID(void);
const char* SYS_getModelName(uint8_t xModel);

void        SYS_LED_init(void);
void        SYS_LED_on(bool bOn);

#endif