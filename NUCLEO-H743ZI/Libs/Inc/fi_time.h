#ifndef TIME_H_
#define TIME_H_

#include <stdint.h>
#include "target.h"
#include <time.h>

typedef uint32_t    FI_TIME;
typedef uint32_t    FI_CLOCK;

#define SECONDS_OF_MINUTE   (60)
#define MINUTES_OF_HOUR     (60)
#define SECONDS_OF_HOUR     (MINUTES_OF_HOUR * SECONDS_OF_MINUTE)
#define HOURS_OF_DAY        (24)
#define SECONDS_OF_DAY      (HOURS_OF_DAY *SECONDS_OF_HOUR)

RET_VALUE   FI_TIME_init(void);
RET_VALUE   FI_TIME_get(FI_TIME* time);
RET_VALUE   FI_TIME_set(FI_TIME  time);
char*       FI_TIME_toString(FI_TIME time, char* format);
RET_VALUE   FI_TIME_toRTCDateTime(FI_TIME value, RTC_TimeTypeDef *rtcTime, RTC_DateTypeDef *rtcDate);
RET_VALUE   FI_TIME_fromRTCDateTime(RTC_TimeTypeDef *rtcTime, RTC_DateTypeDef *rtcDate, FI_TIME *value);

RET_VALUE   FI_TIME_toRTCTime(FI_TIME value, RTC_TimeTypeDef *rtcTime);
RET_VALUE   FI_TIME_fromRTCTime(RTC_TimeTypeDef *rtcTime, FI_TIME *value);

RET_VALUE   FI_TIME_getAlarm(FI_CLOCK* time);
RET_VALUE   FI_TIME_setAlarm(FI_CLOCK time);

uint32_t    FI_TIME_getTimeZone(void);

RET_VALUE   FI_CLOCK_toRTCTime(FI_CLOCK value, RTC_TimeTypeDef *rtcTime);
RET_VALUE   FI_CLOCK_fromRTCTime(RTC_TimeTypeDef *rtcTime, FI_CLOCK *value);

uint32_t    FI_TICK_get(void);


#endif