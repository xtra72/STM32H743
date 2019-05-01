#include "target.h"
#include "monitor.h"
#include "adc.h"
#include "shell.h"

#define __MODULE_NAME__ "MON"
#include "trace.h"

static  void    MONITOR_monitorCallback(void const * argument);

static  osTimerId   timerTraceMonitorHandle;
static  bool        timerRun = false;

RET_VALUE   MONITOR_start()
{
    if (!timerRun)
    {
        DEBUG("Monitoring has already started!\n");
        return  RET_OK;
    }

    if (timerTraceMonitorHandle == 0)
    {
        osTimerDef(timerTraceMonitor, MONITOR_monitorCallback);
        timerTraceMonitorHandle = osTimerCreate(osTimer(timerTraceMonitor), osTimerPeriodic, NULL);
    }

    osTimerStart(timerTraceMonitorHandle, 1000);
    timerRun = true;

    DEBUG("Monitoring started!\n");

    return  RET_OK;
}

RET_VALUE   MONITOR_stop()
{
    if (timerRun)
    {
        osTimerStop(timerTraceMonitorHandle);

        DEBUG("Monitoring stopped!\n");
    }
    else
    {
        DEBUG("Monitoring has not started!\n");
    }

    return  RET_OK;
}

void    MONITOR_monitorCallback(void const * argument)
{
   SHELL_printf("%8d : ", xTaskGetTickCount());
    for(uint32_t i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
    {
        uint16_t    value;
        ADC_CHANNEL_getLastValue(i, &value);
        SHELL_printf(" %4d %4.2f", ADC_CHANNEL_getValueCount(i), value * 3.3 / 65535);
    }
    SHELL_printf("\n");
}