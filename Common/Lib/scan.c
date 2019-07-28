#include "target.h"
#include "config.h"
#include "adc.h"

#if SUPPORT_DRAM

#define __MODULE_NAME__ "SCAN"
#include "trace.h"

RET_VALUE   SCAN_startLoop(void);
void SCAN_loopFinishedCallback(void const * argument);

extern  CONFIG      config_;
        uint64_t    timerLoopStartTime = 0;
static  osTimerId   timerLoopFinishedHandler = 0;

static  bool        trace_ = false;
static  bool        loopRun_ = false;
static  SCAN_DATA*  data_ =  NULL;

RET_VALUE   SCAN_init(void)
{
  osTimerDef(timerLoopFinished, SCAN_loopFinishedCallback);
  timerLoopFinishedHandler = osTimerCreate(osTimer(timerLoopFinished), osTimerPeriodic, NULL);

  return    RET_OK;
}


bool        SCAN_setTrace(bool trace)
{
    trace_ = trace;
    return  true;
}

bool        SCAN_getTrace(void)
{
    return  trace_;
}


RET_VALUE   SCAN_start(void)
{
    if (data_ == NULL)
    {
        uint32_t    memorySize = sizeof(SCAN_DATA) + sizeof(SCAN_LOOP_DATA) * config_.scan.count;

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

    ADC_CHANNEL_start();
    TIME2   start_time;
    TIME2_get(&start_time);
    timerLoopStartTime = start_time * 1000;
    if ((trace_) && (config_.scan.interval < 100))
    {
        osTimerStart(timerLoopFinishedHandler, 100);
    }
    else
    {
        osTimerStart(timerLoopFinishedHandler, config_.scan.interval);
    }

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
    return  config_.scan.interval;
}

bool        SCAN_setInterval(uint32_t interval)
{
    config_.scan.interval = interval;

    return  true;
}

uint32_t    SCAN_getCurrentLoop(void)
{
    return  data_->count;
}

uint32_t    SCAN_getMaxLoop(void)
{
    return  config_.scan.count;
}

bool    SCAN_setMaxLoop(uint32_t count)
{
    config_.scan.count = count;

    return  true;
}

bool    SCAN_DATA_reset(void)
{
    TIME2_get(&data_->time);
    data_->count = 0;

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
    timerLoopStartTime += config_.scan.interval;
    data_->loop[data_->count].time = (uint32_t)(timerLoopStartTime / 1000);
//    TIME2_get(&data_->loop[data_->count].time);
    if (trace_)
    {
        SHELL_printf(" %4d", data_->count);
    }
    for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
    {
        uint16_t    value;
        ADC_CHANNEL_getLastValue(i, &value);
        data_->loop[data_->count].data[i] = value;
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
        if (trace_)
        {
            SHELL_printf(" %5d", value);
        }
    }
    if (trace_)
    {
        SHELL_printf("\n");
    }


    if (++data_->count < config_.scan.count)
    {
        ADC_CHANNEL_start();
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

