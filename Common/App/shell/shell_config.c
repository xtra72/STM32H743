#include "target.h"
#include "config.h"
#include "shell.h"
#include "shell_rf.h"
#include "shell_scan.h"
#include "utils.h"
#include "wireguard.h"
#include "gpio.h"

RET_VALUE SHELL_CONFIG_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_save(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_load(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_clean(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

extern  CONFIG  config_;

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .admin      = false,
        .function = SHELL_CONFIG_help,
        .shortHelp = "Help"
    },
    {
        .name = "save",
        .admin      = false,
        .function = SHELL_CONFIG_save,
        .shortHelp = "Save"
    },
    {
        .name = "load",
        .admin      = false,
        .function = SHELL_CONFIG_load,
        .shortHelp = "Load"
    },
    {
        .name = "clean",
        .admin      = false,
        .function = SHELL_CONFIG_clean,
        .shortHelp = "Clean"
    },
    {
        .name = NULL
    }
};

RET_VALUE SHELL_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    if (argc == 1)
    {
        SHELL_printf("%16s : %s\n",     "Device ID", config_.deviceId);
        SHELL_printf("%16s : %d s\n",   "Keep Alive", WG_KEEPALIVE_getPeriod());
        SHELL_printf("%16s : %d s\n",   "Ready Timeout", WG_getReadyTimeout());
        SHELL_printf("%16s : %d ms\n",  "Transfer Cycle", WG_getTransferInterval());
        SHELL_printf("%16s : %d\n",     "NOP", WG_getTransferNOP());
        SHELL_printf("%16s : %s\n",     "Status", WG_STATUS_getString(WG_getStatus()));
        SHELL_printf("%16s : %s\n",     "AVDD", (GPIO_AVDD_isEnable()?"ON":"OFF"));

        SHELL_printf("\n%s\n", "[ RF ]");
        SHELL_RF_info();
#if SUPPORT_DRAM
        SHELL_printf("\n%s\n", "[ SCAN ]");
        SHELL_SCAN_info();
#endif

        SHELL_printf("\n%s\n", "[ Debug ]");
        SHELL_printf("%16s : %s\n",     "Trace", TRACE_getEnable()?"ON":"OFF");

        return  RET_OK;
    }
    else
    {
        SHELL_COMMAND const*   subcommand = commandSet_;
        while(subcommand->name != NULL)
        {
            if ((strcasecmp(subcommand->name, argv[1]) == 0) && (SHELL_getAdmin() || !subcommand->admin))
            {
                ret = subcommand->function(&argv[1], argc - 1, subcommand);
                break;
            }

            subcommand++;
        }
    }

    return  ret;
}


RET_VALUE SHELL_CONFIG_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SHELL_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        if (SHELL_getAdmin() || !subcommand->admin)
        {
            SHELL_printf("%-16s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);
        }

        subcommand++;
    }

    return  RET_OK;
}

RET_VALUE SHELL_CONFIG_save(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    ret = CONFIG_save(&config_);
    if (ret != RET_OK)
    {
        SHELL_printf("Failed to save settings!\n");
    }

    return  ret;
}

RET_VALUE SHELL_CONFIG_load(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    ret = CONFIG_load(&config_);

    return  ret;
}

RET_VALUE SHELL_CONFIG_clean(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    ret = CONFIG_clear();

    return  ret;
}

