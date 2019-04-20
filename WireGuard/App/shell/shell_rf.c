#include "target.h"
#include "config.h"
#include "shell.h"
#include "rf.h"
#include "utils.h"

RET_VALUE SHELL_RF_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_RF_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_RF_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_RF_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_RF_reg(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_RF_send(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = SHELL_RF_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .function = SHELL_RF_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = SHELL_RF_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "reg",
        .function = SHELL_RF_reg,
        .shortHelp = "Registers"
    },
    {
        .name = "reset",
        .function = SHELL_RF_reset,
        .shortHelp = "Reset"
    },
    {
        .name = "send",
        .function = SHELL_RF_send,
        .shortHelp = "send"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_rf(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_RF_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_RF_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SHELL_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        SHELL_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE SHELL_RF_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RF_start();

    return  RET_OK;
}

RET_VALUE SHELL_RF_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RF_stop();

    return  RET_OK;
}

RET_VALUE SHELL_RF_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RF_reset();

    return  RET_OK;
}


RET_VALUE SHELL_RF_reg(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        for(int i = 0 ; i < 64 ; i++)
        {
            SHELL_printf("%2d : %02x\n", i, RF_readRegister(i));
            osDelay(1);
        }
    }
    else if (argc == 2)
    {

    }

    return  RET_OK;
}



RET_VALUE SHELL_RF_send(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 2)
    {
        uint8_t     buffer[64];
        uint32_t    length;

        if (!strToHexArray(argv[1], buffer, sizeof(buffer), &length))
        {
            SHELL_printf("The payload is invalid!\n");
            return  RET_INVALID_ARGUMENT;
        }

        RF_transmit(buffer, length, 5000);

    }
    else if (argc == 2)
    {

    }

    return  RET_OK;
}

