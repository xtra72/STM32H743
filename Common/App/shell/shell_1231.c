#include "target.h"
#include "config.h"
#include "shell.h"
#include "sx1231.h"

RET_VALUE SHELL_SX1231_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SX1231_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SX1231_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SX1231_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SX1231_init(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SX1231_reg(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = SHELL_SX1231_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .function = SHELL_SX1231_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = SHELL_SX1231_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "reg",
        .function = SHELL_SX1231_reg,
        .shortHelp = "Registers"
    },
    {
        .name = "reset",
        .function = SHELL_SX1231_reset,
        .shortHelp = "Reset"
    },
    {
        .name = "init",
        .function = SHELL_SX1231_init,
        .shortHelp = "Init"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_sx1231(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_SX1231_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_SX1231_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SHELL_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        SHELL_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE SHELL_SX1231_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{

    return  RET_OK;
}

RET_VALUE SHELL_SX1231_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_SX1231_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_SX1231_init(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

extern  SPI_HandleTypeDef hspi3;

RET_VALUE SHELL_SX1231_reg(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    for(int i = 0 ; i < 64 ; i++)
    {
        SHELL_printf("%2d : %02x\n", i, RF_readRegister(i));
        osDelay(1);
    }

    return  RET_OK;
}

