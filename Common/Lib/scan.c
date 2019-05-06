#include "target.h"
#include "config.h"
#include "adc.h"

#if SUPPORT_DRAM

#define __MODULE_NAME__ "SCAN"
#include "trace.h"

RET_VALUE   SCAN_startLoop(void);
void SCAN_loopFinishedCallback(void const * argument);

static  osTimerId timerLoopFinishedHandler = 0;

static  bool        loopRun_ = false;
static  uint32_t    loopInterval_ = 1;
static  uint32_t    loopCountMax_ = 1000 * 60;
static  SCAN_DATA*  data_ =  NULL;

RET_VALUE   SCAN_init(void)
{
  osTimerDef(timerLoopFinished, SCAN_loopFinishedCallback);
  timerLoopFinishedHandler = osTimerCreate(osTimer(timerLoopFinished), osTimerPeriodic, NULL);

  return    RET_OK;
}

RET_VALUE   SCAN_setConfig(SCAN_CONFIG* config)
{
    ASSERT(config);

    if ((data_ != NULL) || (loopCountMax_ != config->count))
    {
        SDRAM_free(data_);
        data_ = NULL;
    }

    loopInterval_ = config->interval;
    loopCountMax_ = config->count;

    if (data_ == NULL)
    {
        uint32_t    memorySize = sizeof(SCAN_DATA) + sizeof(SCAN_LOOP_DATA) * loopCountMax_;

        data_ = SDRAM_malloc(memorySize);
        if (data_ == NULL)
        {
            return  RET_NOT_ENOUGH_MEMORY;
        }

        memset(data_, 0, memorySize);
    }

    return  RET_OK;
}

RET_VALUE   SCAN_getConfig(SCAN_CONFIG* config)
{
    ASSERT(config);

    config->interval = loopInterval_;
    config->count = loopCountMax_;

    return  RET_OK;
}

RET_VALUE   SCAN_start(bool reset)
{
    if (data_ == NULL)
    {
        uint32_t    memorySize = sizeof(SCAN_DATA) + sizeof(SCAN_LOOP_DATA) * loopCountMax_;

        data_ = SDRAM_malloc(memorySize);
        if (data_ == NULL)
        {
            return  RET_NOT_ENOUGH_MEMORY;
        }
        memset(data_, 0, memorySize);
    }

    if (timerLoopFinishedHandler == 0)
    {
        return  RET_ERROR;
    }

    if (reset)
    {
        TIME2_get(&data_->time);
        data_->count = 0;
    }

    ADC_CHANNEL_start(0);
    osTimerStart(timerLoopFinishedHandler, loopInterval_);
    loopRun_ = true;

    DEBUG("Scan started!\n");

    return  RET_OK;
}

RET_VALUE   SCAN_stop()
{
    if (timerLoopFinishedHandler != 0)
    {
        osTimerStop(timerLoopFinishedHandler);
    }
   DEBUG("Scan stopped!\n");

    return  RET_OK;
}

bool    SCAN_isRun(void)
{
    return  loopRun_;
}

uint32_t    SCAN_getInterval(void)
{
    return  loopInterval_;
}

bool        SCAN_setInterval(uint32_t interval)
{
    loopInterval_ = interval;

    return  true;
}

uint32_t    SCAN_getCurrentLoop(void)
{
    return  data_->count;
}

uint32_t    SCAN_getMaxLoop(void)
{
    return  loopCountMax_;
}

bool    SCAN_setMaxLoop(uint32_t count)
{
    loopCountMax_ = count;

    return  true;
}

SCAN_DATA*   SCAN_getData(void)
{
    return  data_;
}

SCAN_LOOP_DATA*   SCAN_getLoopData(uint32_t index)
{
     if (index < data_->count)
     {
        return  &data_->loop[index];
     }

     return NULL;
}

void    SCAN_loopFinishedCallback(void const * argument)
{
    TIME2_get(&data_->loop[data_->count].time);
    for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
    {
        ADC_CHANNEL_getLastValue(i, &data_->loop[data_->count].data[i]);
        if (data_->count == 0)
        {
            data_->statistics[i].min = data_->loop[data_->count].data[i];
            data_->statistics[i].max = data_->loop[data_->count].data[i];
            data_->statistics[i].total = data_->loop[data_->count].data[i];
        }
        else
        {
            if (data_->loop[data_->count].data[i] < data_->statistics[i].min)
            {
                data_->statistics[i].min = data_->loop[data_->count].data[i];
            }

            if (data_->statistics[i].max < data_->loop[data_->count].data[i])
            {
                data_->statistics[i].max = data_->loop[data_->count].data[i];
            }

            data_->statistics[i].total += data_->loop[data_->count].data[i];
        }
    }

    if (++data_->count < loopCountMax_)
    {
        ADC_CHANNEL_start(0);
    }
    else
    {
        loopRun_ = false;
        osTimerStop(timerLoopFinishedHandler);

        if (data_->count != 0)
        {
            for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
            {
                data_->statistics[i].average = data_->statistics[i].total / (double)data_->count;
            }
        }
    }
}

#endif

