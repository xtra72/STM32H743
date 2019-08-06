#include "target.h"

#ifdef STM32H743xx

RET_VALUE   STM32H7XX_SYS_reset(void)
{
    HAL_NVIC_SystemReset();

    return  RET_OK;
}

#endif
