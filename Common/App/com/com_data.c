#include "target.h"
#include "config.h"
#include "shell.h"
#include "scan.h"
#include "utils.h"
#include "comport.h"

RET_VALUE COM_DATA_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_DATA_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);

static const COM_COMMAND   commandSet_[] =
{
    {
        .name = "start",
        .function = COM_DATA_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = COM_DATA_stop,
        .shortHelp = "Stop"
    },
    {
        .name = NULL
    }
};

RET_VALUE   COM_DATA(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command)
{
    RET_VALUE   ret;

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

    return  ret;
}

RET_VALUE COM_DATA_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RET_VALUE   ret = RET_ERROR;
    uint16_t    destAddress = 0;
    uint32_t    offset = 0;
    uint32_t    count = 0;
    uint32_t    timeout = 5000;

    if (argc == 4)
    {
        if (!strToHex16(argv[1], &destAddress))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (!strToUint32(argv[2], &offset))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (!strToUint32(argv[3], &count))
        {
            return  RET_INVALID_ARGUMENT;
        }
    }
    else if (argc == 5)
    {
        if (!strToHex16(argv[1], &destAddress))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (!strToUint32(argv[2], &offset))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (!strToUint32(argv[3], &count))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (!strToUint32(argv[4], &timeout))
        {
            return  RET_INVALID_ARGUMENT;
        }
    }

    uint32_t    currentTick = TICK_get();
    uint32_t    expireTick = TICK_get() + timeout;

    while(TICK_get() < expireTick)
    {
        ret = RF_sendRequestData(destAddress, offset, count, RF_getTimeout());
        if (ret == RET_OK)
        {
            break;
        }
    }

    return  ret;
}

RET_VALUE COM_DATA_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RET_VALUE   ret = RET_ERROR;
    uint16_t    destAddress = 0;
    uint32_t    timeout = 5000;

    if (argc == 2)
    {
        if (!strToHex16(argv[1], &destAddress))
        {
            return  RET_INVALID_ARGUMENT;
        }
    }
    else if (argc == 3)
    {
        if (!strToHex16(argv[1], &destAddress))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (!strToUint32(argv[2], &timeout))
        {
            return  RET_INVALID_ARGUMENT;
        }
    }


    return  ret;
}
