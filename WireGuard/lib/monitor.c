#include "target.h"
#include "monitor.h"
#include "trace.h"
#include "adc.h"
#include "shell.h"

#define TRACE(...)  TRACE_printf("MON", __VA_ARGS__)

osTimerId timerTraceMonitorHandle;

RET_VALUE   MONITOR_start()
{
    osTimerStart(timerTraceMonitorHandle, 1000);

    TRACE("Monitor started!\n");

    return  RET_OK;
}

RET_VALUE   MONITOR_stop()
{
    osTimerStop(timerTraceMonitorHandle);

    TRACE("Monitor stopped!\n");

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