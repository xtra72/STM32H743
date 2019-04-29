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

RET_VALUE   COM_monitor(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command);
RET_VALUE   COM_scan(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE   COM_adc(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE   COM_rf(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE   COM_sdram(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE   COM_config(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);

CONFIG  config_;
extern  uint32_t    ST_count;


static const SHELL_COMMAND   shellCommands[] =
{
    {
        .name       = "scan",
        .function   = SHELL_scan,
        .shortHelp  = "Scan"
    },
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
    {
        .name       = "sdram",
        .function   = SHELL_sdram,
        .shortHelp  = "SDRAM"
    }
};


static const COM_COMMAND   comportCommands[] =
{
    {
        .name       = "scan",
        .function   = COM_scan,
        .shortHelp  = "Scan"
    },
#if 0
    {
        .name       = "monitor",
        .function   = COM_monitor,
        .shortHelp  = "Monitor"
    },
    {
        .name       = "config",
        .function   = COM_config,
        .shortHelp  = "Config"
    },
    {
        .name       = "adc",
        .function   = COM_adc,
        .shortHelp  = "ADC"
    },
#endif
    {
        .name       = "rf",
        .function   = COM_rf,
        .shortHelp  = "RF"
    },
#if 0
    {
        .name       = "sdram",
        .function   = COM_sdram,
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
    COM_init(comportCommands, sizeof(comportCommands) / sizeof(SHELL_COMMAND));
    TIME2_init();
    SCAN_init();

    TRACE_setConfig(&config_.trace);
    SHELL_setConfig(&config_.shell);
    COM_setConfig(&config_.comport);
    ADC_config(&config_.adc);
#if SUPPORT_DRAM
    SDRAM_setConfig(&config_.sdram);
#endif
    SCAN_setConfig(&config_.scan);
    RF_setConfig(&config_.rf);

    SHELL_start();
    COM_start();
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


