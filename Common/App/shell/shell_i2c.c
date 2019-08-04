#include "target.h"
#include "config.h"
#include "shell.h"
#include "max17043.h"
#include "utils.h"
#include "crc16.h"
#include "i2c.h"

RET_VALUE SHELL_I2C_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_I2C_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_I2C_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_I2C_read(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_I2C_write(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_I2C_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = SHELL_I2C_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .function = SHELL_I2C_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = SHELL_I2C_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "read",
        .function = SHELL_I2C_read,
        .shortHelp = "Registers"
    },
    {
        .name = "write",
        .function = SHELL_I2C_write,
        .shortHelp = "Write"
    },
    {
        .name = "test",
        .function = SHELL_I2C_test,
        .shortHelp = "test"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_i2c(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_I2C_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_I2C_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SHELL_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        SHELL_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE SHELL_I2C_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{

    return  RET_OK;
}

RET_VALUE SHELL_I2C_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_I2C_read(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_I2C_write(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_I2C_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    uint16_t    cell;


    MAX17043_getCell(&cell);
    SHELL_printf("cell : %d\n", cell);

    MAX17043_sleep();
    MAX17043_getCell(&cell);
    SHELL_printf("cell : %d\n", cell);

    MAX17043_wakeup();
    MAX17043_getCell(&cell);
    SHELL_printf("cell : %d\n", cell);

    return  RET_OK;
}

