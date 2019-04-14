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

void MAIN_taskEntry(void const * argument)
{
    if (CONFIG_load(&config_) != RET_OK)
    {
        CONFIG_loadDefault(&config_);
    }

    SHELL_init(shellCommands, sizeof(shellCommands) / sizeof(SHELL_COMMAND));
    TIME2_init();
    SCAN_init();

    SHELL_setConfig(&config_.shell);
    ADC_config(&config_.adc);
    SDRAM_setConfig(&config_.sdram);
    SCAN_setConfig(&config_.scan);

    SHELL_start();
    ADC_start();

    for(;;)
    {
        osDelay(1);
    }
}


