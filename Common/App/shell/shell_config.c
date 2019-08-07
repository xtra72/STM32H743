#include "target.h"
#include "config.h"
#include "shell.h"
#include "shell_rf.h"
#include "shell_scan.h"
#include "utils.h"
#include "wireguard.h"

RET_VALUE SHELL_CONFIG_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_save(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_load(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_clean(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_keepAlive(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_readyTimeout(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_CONFIG_nop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

extern  CONFIG  config_;

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = SHELL_CONFIG_help,
        .shortHelp = "Help"
    },
    {
        .name = "save",
        .function = SHELL_CONFIG_save,
        .shortHelp = "Save"
    },
    {
        .name = "load",
        .function = SHELL_CONFIG_load,
        .shortHelp = "Load"
    },
    {
        .name = "clean",
        .function = SHELL_CONFIG_clean,
        .shortHelp = "Clean"
    },
    {
        .name = "keepalive",
        .function = SHELL_CONFIG_keepAlive,
        .shortHelp = "Keep Alive"
    },
    {
        .name = "interval",
        .function = SHELL_CONFIG_interval,
        .shortHelp = "interval"
    },
    {
        .name = "readytimeout",
        .function = SHELL_CONFIG_readyTimeout,
        .shortHelp = "Ready Timeout"
    },
    {
        .name = "nop",
        .function = SHELL_CONFIG_nop,
        .shortHelp = "Number of packets in one transmission"
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
            if (strcasecmp(subcommand->name, argv[1]) == 0)
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
        SHELL_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

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

RET_VALUE SHELL_CONFIG_keepAlive(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d s\n", WG_KEEPALIVE_getPeriod());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = WG_KEEPALIVE_getPeriod();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_KEEPALIVE_setPeriod(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Keep Alive changed : %d s -> %d s\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_CONFIG_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d ms\n", WG_getTransferInterval());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = WG_getTransferInterval();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_setTransferInterval(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Transfer interval changed : %d ms -> %d ms\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_CONFIG_readyTimeout(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d s\n", WG_getReadyTimeout());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = WG_getReadyTimeout();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_setReadyTimeout(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Ready Timeout changed : %d s -> %d s\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_CONFIG_nop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", WG_getTransferNOP());
    }
    else
    {
        uint32_t    old_value = WG_getTransferNOP();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_setTransferNOP(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Transfer NOP changed : %d ms -> %d ms\n", old_value, value);
    }

    return  RET_OK;
}

