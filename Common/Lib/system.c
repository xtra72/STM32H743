#include "target.h"
#include "sdram.h"
#include "system.h"

#define PWR_WAKEUP_PIN_FLAGS  (PWR_WAKEUP_FLAG1 | PWR_WAKEUP_FLAG2 | PWR_WAKEUP_FLAG3 | \
                               PWR_WAKEUP_FLAG4 | PWR_WAKEUP_FLAG5 | PWR_WAKEUP_FLAG6)

extern  RTC_HandleTypeDef hrtc;
extern  RET_VALUE   STM32H7XX_SYS_reset(void);

void   SYS_reset(void)
{
    STM32H7XX_SYS_reset();
}


RET_VALUE   SYS_sleep(uint32_t sleepTime)
{
    /* Disable all used wakeup sources*/
    if (sleepTime > 24*60*60)
    {
        return  RET_OUT_OF_RANGE;
    }

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    /* Clear all related wakeup flags */
    HAL_PWREx_ClearWakeupFlag(PWR_WAKEUP_PIN_FLAGS);

    uint32_t wakeUpCount = sleepTime * 2000;
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpCount, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

    HAL_PWR_EnterSTANDBYMode();

    return    RET_OK;
}

void        SYS_setSleepInfo(SYS_SLEEP_INFO* _info)
{
    SDRAM_STORAGE_write(0, (uint8_t*)_info, sizeof(SYS_SLEEP_INFO));
}

void        SYS_getSleepInfo(SYS_SLEEP_INFO* _info)
{
    SDRAM_STORAGE_read(0, (uint8_t*)_info, sizeof(SYS_SLEEP_INFO));
}
