#include "target.h"
#include "adc.h"
#include "trace.h"

#define TRACE(...)  TRACE_printf("ADC", __VA_ARGS__)

/* ADC handle declaration */
extern  osTimerId timerStartADCHandle;
extern  osTimerId timerMgmtADCHandle;

ADC_CALLBACK    ADC_callback = NULL;
void*           ADC_options = NULL;

typedef struct
{
    ADC_HandleTypeDef*      handle;
    ADC_ChannelConfTypeDef  config;
    uint32_t                bufferSize;
    uint32_t                startIndex;
    uint32_t                lastIndex;
    uint32_t                maxCount;
    uint32_t                count;
    uint16_t                *data;
    struct
    {
        uint32_t            min;
        uint32_t            max;
        float               average;
    }   statistics;
}   ADC_CHANNEL;

static  ADC_HandleTypeDef* adcList_[ADC_COUNT_MAX] = {0, };

static  bool            run = false;
static  ADC_CHANNEL     *channels = NULL;
static  uint32_t        channelCount = 0;
static  uint32_t        currentChannelId = 0;

RET_VALUE   ADC_init(ADC_HandleTypeDef* adcList[], uint32_t count)
{
    if (ADC_COUNT_MAX < count)
    {
        return  RET_ERROR;
    }

    memcpy(adcList_, adcList, sizeof(ADC_HandleTypeDef*) * count);

    return  RET_OK;
}

RET_VALUE   ADC_config(ADC_CONFIG* config)
{
    ASSERT(config);

    if (config->channelCount == 0)
    {
        TRACE("ADC count is 0.\n");
        return  RET_ERROR;
    }

    if(channels != NULL)
    {
        for(int i = 0 ; i < channelCount ; i++)
        {
            vPortFree(channels[i].data);
            channels[i].data = NULL;
        }
        vPortFree(channels);
        channels = NULL;
        channelCount = 0;
    }

    channels = pvPortMalloc(sizeof(ADC_CHANNEL) * config->channelCount);
    if (channels == NULL)
    {
        return  RET_NOT_ENOUGH_MEMORY;
    }
    memset(channels, 0, sizeof(ADC_CHANNEL) * config->channelCount);

    channelCount = config->channelCount;
    for(int i = 0 ; i < channelCount ; i++)
    {
        channels[i].bufferSize = config->dataCount + 1;
        channels[i].maxCount = config->dataCount;
        channels[i].data = pvPortMalloc(sizeof(uint16_t) * channels[i].bufferSize);
        memset(channels[i].data, 0, sizeof(uint16_t) * channels[i].bufferSize);
    }

    for(int i = 0 ; i < channelCount ; i++)
    {
        if (config->channels[i].deviceId < ADC_COUNT_MAX)
        {
            channels[i].handle = adcList_[config->channels[i].deviceId];
            channels[i].config.Channel      = config->channels[i].channelId;  /* Sampled channel number */
            channels[i].config.Rank         = ADC_REGULAR_RANK_1;           /* Rank of sampled channel number ADCx_CHANNEL */
            channels[i].config.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;     /* Sampling time (number of clock cycles unit) */
            channels[i].config.SingleDiff   = ADC_SINGLE_ENDED;             /* Single-ended input channel */
            channels[i].config.OffsetNumber = ADC_OFFSET_NONE;              /* No offset subtraction */
            channels[i].config.Offset       = 0;                            /* Parameter discarded because offset correction is disabled */
        }
    }

    return  RET_OK;
}

RET_VALUE   ADC_start(void)
{
    if (run)
    {
        return  RET_ALREADY_STARTED;
    }

    run  = true;

    return  RET_ERROR;
}

RET_VALUE   ADC_stop(void)
{
    if (!run)
    {
        return  RET_NOT_RUNNING;
    }

    if (osTimerStop(timerStartADCHandle) == osOK)
    {
        run = false;

        return  RET_OK;
    }

    return  RET_ERROR;
}

uint32_t    ADC_CHANNEL_getCount(void)
{
    return  channelCount;
}

RET_VALUE   ADC_CHANNEL_start(uint32_t channelId)
{
    if (channelCount <= channelId)
    {
        TRACE("Out of range(< %d)\n", channelCount);
        return  RET_ERROR;
    }

    currentChannelId = channelId;

    if (HAL_ADC_ConfigChannel(channels[currentChannelId].handle, &channels[currentChannelId].config) != HAL_OK)
    {
        TRACE("The ADC failed to config channel.\n");
        return RET_ERROR;
    }

    if (HAL_ADC_Start_IT(channels[currentChannelId].handle) != HAL_OK)
    {
        TRACE("The ADC failed to start the conversion process.\n");
        return RET_ERROR;
    }


    return  RET_OK;
}

RET_VALUE   ADC_CHANNEL_next(void)
{
    if (channelCount == 0)
    {
        return  RET_ERROR;
    }

    if ((currentChannelId + 1) < channelCount)
    {
        ADC_CHANNEL_start((currentChannelId + 1) % channelCount);
    }

    return  RET_OK;
}

RET_VALUE   ADC_CHANNEL_clearValue(uint32_t channelId)
{
    if (channelId < channelCount)
    {
        channels[channelId].count = 0;
        return  RET_OK;
    }

    return  RET_OUT_OF_RANGE;
}

uint32_t    ADC_CHANNEL_getValueCount(uint32_t channelId)
{
    if (channelId < channelCount)
    {
        return  channels[channelId].count;
    }

    return  0;
}

RET_VALUE   ADC_CHANNEL_getValue(uint32_t channelId, uint32_t index, uint16_t* value)
{
    ASSERT(value);

    if (channelId < channelCount)
    {
        *value = channels[channelId].data[(channels[channelId].startIndex + index) % channels[channelId].bufferSize];
        return  RET_OK;
    }

    return  RET_OUT_OF_RANGE;
}

RET_VALUE   ADC_CHANNEL_getLastValue(uint32_t channelId, uint16_t* value)
{
    ASSERT(value);

    return  ADC_CHANNEL_getValue(channelId, channels[channelId].count - 1, value);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop_IT(channels[currentChannelId].handle);

    channels[currentChannelId].data[channels[currentChannelId].lastIndex] = HAL_ADC_GetValue(channels[currentChannelId].handle);
    if (channels[currentChannelId].count + 1 < channels[currentChannelId].bufferSize)
    {
        channels[currentChannelId].count = channels[currentChannelId].count + 1;
    }
    else
    {
        channels[currentChannelId].startIndex = (channels[currentChannelId].startIndex + 1) % channels[currentChannelId].bufferSize;
    }

    channels[currentChannelId].lastIndex = (channels[currentChannelId].lastIndex + 1) % channels[currentChannelId].bufferSize;

    ADC_CHANNEL_next();
}


RET_VALUE   ADC_CHANNEL_getStatistics(uint32_t channelId, ADC_CHANNEL_STATISTICS* statistics)
{
    ASSERT(statistics);

    if (channelId < channelCount)
    {
        uint32_t            min = 0;
        uint32_t            max = 0;
        float               average = 0;

        if (ADC_CHANNEL_getValueCount(channelId) != 0)
        {
            uint16_t    value = 0;

            ADC_CHANNEL_getValue(channelId, 0, &value);

            min = value;
            max = value;
            average = value;

            for(int i = 1 ; i < ADC_CHANNEL_getValueCount(channelId); i++)
            {
                ADC_CHANNEL_getValue(channelId, i, &value);
                if (value < min)
                {
                    min = value;
                }

                if (max < value)
                {
                    max = value;
                }

                average += value;
            }

            statistics->count = ADC_CHANNEL_getValueCount(channelId);
            statistics->min = min;
            statistics->max = max;
            statistics->average = average / ADC_CHANNEL_getValueCount(channelId);

            return  RET_OK;
        }
        else
        {
            statistics->count = 0;
            statistics->min = 0;
            statistics->max = 0;
            statistics->average = 0;
        }
    }

    return  RET_OUT_OF_RANGE;
}