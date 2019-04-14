#include "target.h"

extern  RET_VALUE   STM32H7XX_SYS_reset(void);

RET_VALUE   SYS_reset(void)
{
    return  STM32H7XX_SYS_reset();
}
