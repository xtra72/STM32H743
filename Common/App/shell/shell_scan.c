#include "target.h"
#include "config.h"
#include "shell.h"
#include "scan.h"
#include "utils.h"

#if SUPPORT_DRAM
RET_VALUE SHELL_SCAN_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_statistics(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_count(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_channel(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SCAN_data(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .admin      = false,
        .function = SHELL_SCAN_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .admin      = true,
        .function = SHELL_SCAN_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .admin      = true,
        .function = SHELL_SCAN_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "data",
        .admin      = true,
        .function = SHELL_SCAN_data,
        .shortHelp = "Data"
    },
    {
        .name = "interval",
        .admin      = false,
        .function = SHELL_SCAN_interval,
        .shortHelp = "Interval"
    },
    {
        .name = "count",
        .admin      = false,
        .function = SHELL_SCAN_count,
        .shortHelp = "Count"
    },
    {
        .name = "channel",
        .admin      = false,
        .function = SHELL_SCAN_channel,
        .shortHelp = "Channel"
    },
    {
        .name = "statistics",
        .admin      = false,
        .function = SHELL_SCAN_statistics,
        .shortHelp = "Statistics"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_SCAN(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        SHELL_printf("%16s : %s\n", "Round Status", SCAN_isRun()?"Run":"Stop");
        SHELL_printf("%16s : %d ms\n", "Interval", SCAN_getInterval());
        SHELL_printf("%16s : %d\n", "Current Count", SCAN_getCurrentLoop());
        SHELL_printf("%16s : %d\n", "Max Count", SCAN_getMaxLoop());

        if (!SCAN_isRun())
        {
            SCAN_DATA* data = SCAN_getData();

            if (data->count != 0)
            {
                SHELL_printf("%16s : %4s %4s %4s\n", "Statistics", "MIN", "MAX", "AVG");
                for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
                {
                    SHELL_printf("%16d : %4.2f %4.2f %4.2f\n", i,
                                 data->statistics[i].min * 3.3 / 65535,
                                 data->statistics[i].max * 3.3 / 65535,
                                 data->statistics[i].average * 3.3 / 65535);
                }
            }
        }
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

RET_VALUE SHELL_SCAN_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
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

RET_VALUE SHELL_SCAN_info(void)
{
    SHELL_printf("%16s : %s\n", "Status", SCAN_isRun()?"Run":"Stop");
    SHELL_printf("%16s : %d ms\n", "Interval", SCAN_getInterval());
    SHELL_printf("%16s : %d\n", "Channel Count", ADC_CHANNEL_getCount());
    SHELL_printf("%16s : %d\n", "Data Count", SCAN_getCurrentLoop());
    SHELL_printf("%16s : %d\n", "Max Data Count", SCAN_getMaxLoop());

    return  RET_OK;
}

RET_VALUE SHELL_SCAN_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    bool    reset = false;
    bool    trace = false;

    for(uint32_t i = 1 ; i < argc ; i++)
    {
        if (strcasecmp(argv[i], "reset") == 0)
        {
            reset = true;
        }
        else if (strcasecmp(argv[i], "trace") == 0)
        {
            trace = true;
        }
        else
        {
            return  RET_INVALID_ARGUMENT;
        }
    }

    if (reset)
    {
        SCAN_DATA_reset();
    }

    SCAN_setTrace(trace);

    if (SCAN_start() != RET_OK)
    {
        SHELL_printf("Scan start failed.\n");
    }

    return  RET_OK;
}

RET_VALUE SHELL_SCAN_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (SCAN_stop() != RET_OK)
    {
        SHELL_printf("Scan stop failed.\n");
    }

    return  RET_OK;
}

RET_VALUE SHELL_SCAN_statistics(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        if (SCAN_isRun())
        {
            SHELL_printf("Can not be displayed during data collection.\n");
        }
        else
        {
            SCAN_DATA* data = SCAN_getData();

            for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
            {
                SHELL_printf("%d : %4.2f %4.2f %4.2f\n", i,
                             data->statistics[i].min * 3.3 / 65535,
                             data->statistics[i].max * 3.3 / 65535,
                             data->statistics[i].average * 3.3 / 65535);
            }

        }
    }

    return  RET_OK;
}


RET_VALUE SHELL_SCAN_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", SCAN_getInterval());
    }
    else if (argc == 2)
    {
        uint32_t    interval;

        if (!strToUint32(argv[1], &interval) || !SCAN_setInterval(interval))
        {
            SHELL_printf("Invalid interval.\n");
        }
        else
        {
            SHELL_printf("Scan interval : %d ms\n", SCAN_getInterval());
        }
    }

    return  RET_OK;
}

RET_VALUE SHELL_SCAN_count(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", SCAN_getMaxLoop());
    }
    else if (argc == 2)
    {
        uint32_t    count;

        if (!strToUint32(argv[1], &count) || !SCAN_setMaxLoop(count))
        {
            SHELL_printf("Invalid count.\n");
        }
    }

    return  RET_OK;
}

RET_VALUE SHELL_SCAN_channel(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", ADC_CHANNEL_getCount());
    }
    else if (argc == 2)
    {
        uint32_t    count;

        if (!strToUint32(argv[1], &count) || !ADC_CHANNEL_setCount(count))
        {
            SHELL_printf("Invalid count.\n");
        }
        else
        {
            SHELL_printf("Channel count : %d\n", ADC_CHANNEL_getCount());
        }
    }

    return  RET_OK;
}

RET_VALUE SHELL_SCAN_data(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 2)
    {
        uint32_t    index;

        if (!strToUint32(argv[1], &index) || !(index < SCAN_getCurrentLoop()))
        {
            SHELL_printf("Invalid count.\n");
        }
        else
        {
            SCAN_LOOP_DATA*   data = SCAN_getLoopData(index);
            for(int i=  0 ; i < ADC_CHANNEL_getCount() ; i++)
            {
                SHELL_printf("%4.2f ", data->data[i] * 3.3 / 65535);
            }
            SHELL_printf("\n");

        }
    }
    else if (argc == 3)
    {
        uint32_t    index;
        uint32_t    count;

        if (!strToUint32(argv[1], &index) || !strToUint32(argv[2], &count) || !(index < SCAN_getCurrentLoop()) || !(index + count <= SCAN_getCurrentLoop()))
        {
            SHELL_printf("Invalid count.\n");
        }
        else
        {
            for(int j = 0 ; j < count ; j++)
            {
                SCAN_LOOP_DATA*   data = SCAN_getLoopData(index + j);
                for(int i=  0 ; i < ADC_CHANNEL_getCount() ; i++)
                {
                    SHELL_printf("%5d ", data->data[i]);
      //              SHELL_printf("%4.2f ", data->data[i] * 3.3 / 65535);
                }
                SHELL_printf("\n");
            }

        }
    }

    return  RET_OK;
}

#endif