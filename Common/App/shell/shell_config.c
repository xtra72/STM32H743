#include "target.h"
#include "config.h"
#include "shell.h"
#include "shell_rf.h"
#include "shell_scan.h"

extern  CONFIG  config_;

RET_VALUE SHELL_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    if (argc == 1)
    {
        SHELL_printf("%16s : %d s\n",   "Keep Alive", RF_getKeepAlive());
        SHELL_printf("%16s : %d s\n",   "Ready Timeout", RF_getReadyTimeout());
        SHELL_printf("%16s : %d ms\n",  "Transfer Cycle", RF_getTransferInterval());
        SHELL_printf("%16s : %d\n",     "NOP", RF_getTransferNOP());

        SHELL_printf("\n%s\n", "[ System ]");
        SHELL_printf("%16s : %s\n",     "Device ID", config_.deviceId);
        SHELL_printf("%16s : %s\n",     "Status", RF_getStatusString(RF_getStatus()));

        SHELL_RF_info();
#if SUPPORT_DRAM
        SHELL_printf("\n%s\n", "[ SCAN ]");
        SHELL_SCAN_info();
#endif

        SHELL_printf("\n%s\n", "[ Debug ]");
        SHELL_printf("%16s : %s\n",     "Trace", TRACE_getEnable()?"ON":"OFF");

        return  RET_OK;
    }

    if (strcasecmp(argv[1], "save") == 0)
    {
        ret = CONFIG_save(&config_);
        if (ret != RET_OK)
        {
            SHELL_printf("Failed to save settings!\n");
        }
    }
    else if (strcasecmp(argv[1], "load") == 0)
    {
        ret = CONFIG_load(&config_);
    }
    else if (strcasecmp(argv[1], "clean") == 0)
    {
        ret = CONFIG_clear();
    }
    else if (strcasecmp(argv[1],  "rf") == 0)
    {
        ret = SHELL_RF_config(&argv[1], argc - 1, command);
    }

    return  ret;
}
