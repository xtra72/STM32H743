#include "target.h"
#include "config.h"
#include "shell.h"
#include "utils.h"

RET_VALUE SHELL_TRANS_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_TRANS_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_TRANS_nop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = SHELL_TRANS_help,
        .shortHelp = "Help"
    },
    {
        .name = "interval",
        .function = SHELL_TRANS_interval,
        .shortHelp = "interval"
    },
    {
        .name = "nop",
        .function = SHELL_TRANS_nop,
        .shortHelp = "Number of packets in one transmission"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_trans(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_TRANS_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_TRANS_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SHELL_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        SHELL_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE SHELL_TRANS_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d ms\n", RF_getTransferInterval());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = RF_getTransferInterval();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (RF_setTransferInterval(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Transfer interval changed : %d ms -> %d ms\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_TRANS_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

