#include "target.h"
#include "config.h"
#include "shell.h"
#include "scan.h"
#include "utils.h"
#include "comport.h"

RET_VALUE COM_SCAN_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_SCAN_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);

static const COM_COMMAND   commandSet_[] =
{
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
        .name = NULL
    }
};

RET_VALUE   COM_scan(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command)
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

RET_VALUE COM_SCAN_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RET_VALUE   ret = RET_ERROR;
    uint16_t    destAddress = 0;
    uint32_t    dataReset = 0;
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

        if (!strToUint32(argv[2], &dataReset))
        {
            return  RET_INVALID_ARGUMENT;
        }
    }

    uint32_t    currentTick = TICK_get();
    uint32_t    expireTick = TICK_get() + timeout;

    while(TICK_get() < expireTick)
    {
        ret = RF_sendStartScan(destAddress, dataReset, RF_getTimeout());
        if (ret == RET_OK)
        {
            break;
        }
    }

    return  ret;
}

RET_VALUE COM_SCAN_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
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

    uint32_t    currentTick = TICK_get();
    uint32_t    expireTick = TICK_get() + timeout;

    while(TICK_get() < expireTick)
    {
        ret = RF_sendStopScan(destAddress, RF_getTimeout());
        if (ret == RET_OK)
        {
            break;
        }
    }

    return  ret;}
