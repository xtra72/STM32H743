#ifndef ADC_H__
#define ADC_H__

#include "target.h"

#ifndef TARGET_ADC_COUNT_MAX
#define ADC_COUNT_MAX           2
#else
#define ADC_COUNT_MAX           TARGET_ADC_COUNT_MAX
#endif

#ifndef TARGET_ADC_CHANNEL_COUNT_MAX
#define ADC_CHANNEL_COUNT_MAX   8
#else
#define ADC_CHANNEL_COUNT_MAX   TARGET_ADC_CHANNEL_COUNT_MAX
#endif

typedef struct
{
    uint32_t        id;
    ADC_TypeDef*    instance;
    bool            multiMode;
}   ADC_DESCRIPTION;

typedef struct
{
    uint32_t    deviceId;
    uint32_t    channelId;
}   ADC_CHANNEL_CONFIG;

typedef struct
{
    uint32_t    count;
    uint32_t    min;
    uint32_t    max;
    double      average;
}   ADC_CHANNEL_STATISTICS;

typedef struct
{
    uint32_t            channelCount;
    ADC_CHANNEL_CONFIG  channels[ADC_CHANNEL_COUNT_MAX];
    uint32_t            dataCount;
}       ADC_CONFIG;

typedef void (*ADC_CALLBACK)(void*);

RET_VALUE   ADC_init(ADC_HandleTypeDef* adcList[], uint32_t count);
RET_VALUE   ADC_config(ADC_CONFIG* config);

RET_VALUE   ADC_start(void);
RET_VALUE   ADC_stop(void);

uint32_t    ADC_CHANNEL_getCount(void);
RET_VALUE   ADC_CHANNEL_start(uint32_t channelId);
RET_VALUE   ADC_CHANNEL_next(void);

uint32_t    ADC_CHANNEL_getValueCount(uint32_t channelId);
RET_VALUE   ADC_CHANNEL_getValue(uint32_t channelId, uint32_t index, uint16_t* value);
RET_VALUE   ADC_CHANNEL_getLastValue(uint32_t channelId, uint16_t* value);
RET_VALUE   ADC_CHANNEL_clearValue(uint32_t channelId);

RET_VALUE   ADC_CHANNEL_getStatistics(uint32_t channelId, ADC_CHANNEL_STATISTICS* statistics);

#endif