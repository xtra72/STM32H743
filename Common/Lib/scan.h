#ifndef SCAN_H_
#define SCAN_H_

#include "target.h"
#include "adc.h"

typedef struct
{
    uint32_t    interval;
    uint32_t    count;
}   SCAN_CONFIG;

typedef struct
{
    TIME2       time;
    uint16_t    data[ADC_CHANNEL_COUNT_MAX];
}   SCAN_LOOP_DATA;

typedef struct
{
    TIME2           time;
    uint32_t        count;
    struct
    {
        uint32_t     min;
        uint32_t     max;
        double      average;
        uint64_t    total;
    }   statistics[ADC_CHANNEL_COUNT_MAX];
    SCAN_LOOP_DATA  loop[];
}   SCAN_DATA;

RET_VALUE   SCAN_init();

RET_VALUE   SCAN_setConfig(SCAN_CONFIG* config);
RET_VALUE   SCAN_getConfig(SCAN_CONFIG* config);

RET_VALUE   SCAN_start();
RET_VALUE   SCAN_stop();

bool        SCAN_isRun(void);

uint32_t    SCAN_getInterval(void);
bool        SCAN_setInterval(uint32_t interval);

uint32_t    SCAN_getCurrentLoop(void);
uint32_t    SCAN_getMaxLoop(void);
bool        SCAN_setMaxLoop(uint32_t count);

SCAN_DATA*      SCAN_getData(void);
SCAN_LOOP_DATA*   SCAN_getLoopData(uint32_t index);

#endif
