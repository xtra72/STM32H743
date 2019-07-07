#ifndef TIME2_H_
#define TIME2_H_

#include <stdint.h>
#include "target.h"
#include <time.h>

typedef uint32_t    TIME2;
typedef uint32_t    FI_CLOCK;

#define SECONDS_OF_MINUTE   (60)
#define MINUTES_OF_HOUR     (60)
#define SECONDS_OF_HOUR     (MINUTES_OF_HOUR * SECONDS_OF_MINUTE)
#define HOURS_OF_DAY        (24)
#define SECONDS_OF_DAY      (HOURS_OF_DAY *SECONDS_OF_HOUR)

RET_VALUE   TIME2_init(void);
RET_VALUE   TIME2_get(TIME2* time);
RET_VALUE   TIME2_set(TIME2  time);
char*       TIME2_toString(TIME2 time, char* format);
RET_VALUE   TIME2_toRTCDateTime(TIME2 value, RTC_TimeTypeDef *rtcTime, RTC_DateTypeDef *rtcDate);
RET_VALUE   TIME2_fromRTCDateTime(RTC_TimeTypeDef *rtcTime, RTC_DateTypeDef *rtcDate, TIME2 *value);

RET_VALUE   TIME2_toRTCTime(TIME2 value, RTC_TimeTypeDef *rtcTime);
RET_VALUE   TIME2_fromRTCTime(RTC_TimeTypeDef *rtcTime, TIME2 *value);

RET_VALUE   TIME2_getAlarm(FI_CLOCK* time);
RET_VALUE   TIME2_setAlarm(FI_CLOCK time);

uint32_t    TIME2_getTimeZone(void);

uint32_t    TICK_get(void);
uint32_t    TICK_elapsedTime(uint32_t _base);
uint32_t    TICK_remainTime(uint32_t _base, uint32_t _timeout);


#endif