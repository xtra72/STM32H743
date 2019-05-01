#include "target.h"
#include "config.h"
#include "adc.h"
#include "rf.h"
#include "scan.h"

void    TRACE_monitorCallback(void const * argument);
RET_VALUE   SHELL_monitor(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_scan(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_adc(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_rf(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_sdram(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);

CONFIG  config_;
extern  uint32_t    ST_count;


static const SHELL_COMMAND   shellCommands[] =
{
    {
        .name       = "monitor",
        .function   = SHELL_monitor,
        .shortHelp  = "Monitor"
    },
    {
        .name       = "config",
        .function   = SHELL_config,
        .shortHelp  = "Config"
    },
    {
        .name       = "adc",
        .function   = SHELL_adc,
        .shortHelp  = "ADC"
    },
    {
        .name       = "rf",
        .function   = SHELL_rf,
        .shortHelp  = "RF"
    },
#if SUPPORT_DRAM
    {
        .name       = "scan",
        .function   = SHELL_scan,
        .shortHelp  = "Scan"
    },
    {
        .name       = "sdram",
        .function   = SHELL_sdram,
        .shortHelp  = "SDRAM"
    }
#endif
};

void MAIN_taskEntry(void const * argument)
{
//    if (CONFIG_load(&config_) != RET_OK)
    {
        CONFIG_loadDefault(&config_);
    }

    SHELL_init(shellCommands, sizeof(shellCommands) / sizeof(SHELL_COMMAND));
    TIME2_init();
#if SUPPORT_DRAM
    SCAN_init();
#endif

    TRACE_setConfig(&config_.trace);
    SHELL_setConfig(&config_.shell);
#if SUPPORT_COM
    COM_setConfig(&config_.comport);
#endif
    ADC_config(&config_.adc);
#if SUPPORT_DRAM
    SDRAM_setConfig(&config_.sdram);
    SCAN_setConfig(&config_.scan);
#endif
    RF_setConfig(&config_.rf);

    SHELL_start();
#if SUPPORT_COM
    COM_start();
#endif
    if (config_.adc.enable)
    {
        ADC_start();
    }

    if (config_.rf.enable)
    {
        RF_start();
    }

    for(;;)
    {

        osDelay(1);
    }
}


