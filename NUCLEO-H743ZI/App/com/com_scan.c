#include "target.h"
#include "config.h"
#include "shell.h"
#include "scan.h"
#include "utils.h"

RET_VALUE COM_SCAN_help(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_statistics(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_interval(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_count(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_data(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);

static const COM_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = COM_SCAN_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .function = COM_SCAN_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = COM_SCAN_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "data",
        .function = COM_SCAN_data,
        .shortHelp = "Data"
    },
    {
        .name = "interval",
        .function = COM_SCAN_interval,
        .shortHelp = "Interval"
    },
    {
        .name = "count",
        .function = COM_SCAN_count,
        .shortHelp = "Count"
    },
    {
        .name = "statistics",
        .function = COM_SCAN_statistics,
        .shortHelp = "Statistics"
    },
    {
        .name = NULL
    }
};

RET_VALUE   COM_scan(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        COM_printf("%16s : %s\n", "Round Status", SCAN_isRun()?"Run":"Stop");
        COM_printf("%16s : %d ms\n", "Interval", SCAN_getInterval());
        COM_printf("%16s : %d\n", "Current Count", SCAN_getCurrentLoop());
        COM_printf("%16s : %d\n", "Max Count", SCAN_getMaxLoop());

        if (!SCAN_isRun())
        {
            SCAN_DATA* data = SCAN_getData();

            if (data->count != 0)
            {
                COM_printf("%16s : %4s %4s %4s\n", "Statistics", "MIN", "MAX", "AVG");
                for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
                {
                    COM_printf("%16d : %4.2f %4.2f %4.2f\n", i,
                                 data->statistics[i].min * 3.3 / 65535,
                                 data->statistics[i].max * 3.3 / 65535,
                                 data->statistics[i].average * 3.3 / 65535);
                }
            }
        }
    }
    else
    {
        COM_COMMAND const*   subcommand = commandSet_;
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

RET_VALUE COM_SCAN_help(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    COM_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        COM_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE COM_SCAN_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (SCAN_start() != RET_OK)
    {
        COM_printf("Scan start failed.\n");
    }

    return  RET_OK;
}

RET_VALUE COM_SCAN_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (SCAN_stop() != RET_OK)
    {
        COM_printf("Scan stop failed.\n");
    }

    return  RET_OK;
}

RET_VALUE COM_SCAN_statistics(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 1)
    {
        if (SCAN_isRun())
        {
            COM_printf("Can not be displayed during data collection.\n");
        }
        else
        {
            SCAN_DATA* data = SCAN_getData();

            for(int i = 0 ; i < ADC_CHANNEL_getCount() ; i++)
            {
                COM_printf("%d : %4.2f %4.2f %4.2f\n", i,
                             data->statistics[i].min * 3.3 / 65535,
                             data->statistics[i].max * 3.3 / 65535,
                             data->statistics[i].average * 3.3 / 65535);
            }

        }
    }

    return  RET_OK;
}


RET_VALUE COM_SCAN_interval(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 1)
    {
        COM_printf("%d\n", SCAN_getInterval());
    }
    else if (argc == 2)
    {
        uint32_t    interval;

        if (!strToUint32(argv[1], &interval) || !SCAN_setInterval(interval))
        {
            COM_printf("Invalid interval.\n");
        }
    }

    return  RET_OK;
}

RET_VALUE COM_SCAN_count(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 1)
    {
        COM_printf("%d\n", SCAN_getMaxLoop());
    }
    else if (argc == 2)
    {
        uint32_t    count;

        if (!strToUint32(argv[1], &count) || !SCAN_setMaxLoop(count))
        {
            COM_printf("Invalid count.\n");
        }
    }

    return  RET_OK;
}

RET_VALUE COM_SCAN_data(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 2)
    {
        uint32_t    index;

        if (!strToUint32(argv[1], &index) || !(index < SCAN_getCurrentLoop()))
        {
            COM_printf("Invalid count.\n");
        }
        else
        {
            SCAN_LOOP_DATA*   data = SCAN_getLoopData(index);
            for(int i=  0 ; i < ADC_CHANNEL_getCount() ; i++)
            {
                COM_printf("%4.2f ", data->data[i] * 3.3 / 65535);
            }
            COM_printf("\n");

        }
    }
    else if (argc == 3)
    {
        uint32_t    index;
        uint32_t    count;

        if (!strToUint32(argv[1], &index) || !strToUint32(argv[2], &count) || !(index < SCAN_getCurrentLoop()) || !(index + count <= SCAN_getCurrentLoop()))
        {
            COM_printf("Invalid count.\n");
        }
        else
        {
            for(int j = 0 ; j < count ; j++)
            {
                SCAN_LOOP_DATA*   data = SCAN_getLoopData(index + j);
                for(int i=  0 ; i < ADC_CHANNEL_getCount() ; i++)
                {
                    COM_printf("%4.2f ", data->data[i] * 3.3 / 65535);
                }
                COM_printf("\n");
            }

        }
    }

    return  RET_OK;
}
