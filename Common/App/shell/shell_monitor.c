#include "target.h"
#include "config.h"
#include "shell.h"
#include "monitor.h"

RET_VALUE SHELL_MONITOR_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_MONITOR_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_MONITOR_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

extern  osTimerId timerTraceMonitorHandle;

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .admin      = true,
        .function = SHELL_MONITOR_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .admin      = true,
        .function = SHELL_MONITOR_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .admin      = true,
        .function = SHELL_MONITOR_stop,
        .shortHelp = "Stop"
    },
    {
        .name = NULL
    }
};


RET_VALUE   SHELL_monitor(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_MONITOR_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_MONITOR_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
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


RET_VALUE SHELL_MONITOR_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (MONITOR_start() != RET_OK)
    {
        SHELL_printf("Scan start failed.\n");
    }

   return  RET_OK;
}

RET_VALUE SHELL_MONITOR_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (MONITOR_stop() != RET_OK)
    {
        SHELL_printf("Scan stop failed.\n");
    }

    return  RET_OK;
}