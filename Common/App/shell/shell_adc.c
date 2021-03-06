#include "target.h"
#include "config.h"
#include "shell.h"

RET_VALUE SHELL_ADC_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_ADC_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_ADC_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .admin      = false,
        .function = SHELL_ADC_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .admin      = true,
        .function = SHELL_ADC_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .admin      = true,
        .function = SHELL_ADC_stop,
        .shortHelp = "Stop"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_adc(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_ADC_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_ADC_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
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


RET_VALUE SHELL_ADC_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    for(uint32_t i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
    {
        ADC_CHANNEL_clearValue(i);
    }

    ADC_CHANNEL_start();

    osDelay(10);
    for(uint32_t i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
    {
        uint16_t    value;
        ADC_CHANNEL_getLastValue(i, &value);
        SHELL_printf(" %5d", value);
    }
    SHELL_printf("\n");

    return  RET_OK;
}

RET_VALUE SHELL_ADC_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

