#include "target.h"
#include "system.h"
#include "config.h"
#include "shell.h"
#include "wireguard.h"
#include "utils.h"

extern  CONFIG  config_;

RET_VALUE SHELL_COMMAND_getVersion(char *argv[], uint32_t argc, struct _SHELL_COMMAND* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    if (argc == 1)
    {
        SHELL_printf("%s\n", CONFIG_VERSION);
        ret = RET_OK;
    }
    else if (argc == 2)
    {
        if (strcasecmp(argv[1], "help") == 0)
        {
            SHELL_printf("Version\n");
            ret = RET_OK;
        }
    }


    return  ret;
}

RET_VALUE SHELL_COMMAND_date(char *argv[], uint32_t argc, struct _SHELL_COMMAND* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    if (argc == 1)
    {
        TIME2 time;

        ret = TIME2_get(&time);
        if (ret == RET_OK)
        {
            SHELL_printf("%s (%d)\n", TIME2_toString(time, "%Y-%m-%d %H:%M:%S"), time);
        }
        else
        {
            SHELL_printf("Error : Can't get time!\n");
        }
   }
    else if (argc == 2)
    {
        if (strcasecmp(argv[1], "init") == 0)
        {
            TIME2_init();
        }
        else
        {
            uint32_t    i;
            uint32_t    length;

            length = strlen(argv[1]);
            if (length == 14)
            {
                ret = RET_OK;

                for(i = 0 ; i < length ; i++)
                {
                    if (!isdigit(argv[1][i]))
                    {
                        SHELL_printf("Error : Invalid arguments.\n");
                        ret = RET_INVALID_ARGUMENT;
                        break;
                    }
                }
            }

            if (ret == RET_OK)
            {
                struct tm   tm;
                uint32_t    value;

                memset(&tm, 0, sizeof(tm));

                ret = RET_INVALID_ARGUMENT;

                value =  strtoul(&argv[1][12], 0, 10);
                argv[1][12] = 0;
                if (value < 60)
                {
                    tm.tm_sec   = value;
                    value = strtoul(&argv[1][10], 0, 10);
                    argv[1][10] = 0;
                    if (value < 60)
                    {
                        tm.tm_min   = value;
                        value = strtoul(&argv[1][8], 0, 10);
                        argv[1][8] = 0;
                        if (value < 24)
                        {
                            tm.tm_hour  = value;
                            value = strtoul(&argv[1][6], 0, 10);
                            argv[1][6] = 0;
                            if (1 <= value && value <= 31)
                            {
                                tm.tm_mday  = value;
                                value = strtoul(&argv[1][4], 0, 10);
                                argv[1][4] = 0;
                                if (1 <= value && value <= 12)
                                {
                                    tm.tm_mon   = value - 1;
                                    value = strtoul(&argv[1][0], 0, 10);
                                    if (2000 <= value && value <= 2100)
                                    {
                                        tm.tm_year   = value - 1900;

                                        value = mktime(&tm);
                                        SHELL_printf("value : %s", ctime(&value));
                                        ret = TIME2_set(value);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return  ret;
}

RET_VALUE SHELL_COMMAND_deviceId(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%s\n", config_.deviceId);
    }
    else if (argc == 3)
    {
        if (strlen(argv[1]) > CONFIG_DEVICE_ID_LEN)
        {
            SHELL_printf("ID is too long : Max %d\n", CONFIG_DEVICE_ID_LEN);
            return  RET_INVALID_ARGUMENT;
        }

        char    oldDeviceId[CONFIG_DEVICE_ID_LEN+1];
        strcpy(oldDeviceId, config_.deviceId);

        memset(config_.deviceId, 0, sizeof(config_.deviceId));
        strcpy(config_.deviceId, argv[1]);

        SHELL_printf("Device ID changed : %s -> %s\n", oldDeviceId, config_.deviceId);
    }

    return  RET_OK;
}


RET_VALUE SHELL_COMMAND_keepAlive(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d s\n", WG_KEEPALIVE_getPeriod());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = WG_KEEPALIVE_getPeriod();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_KEEPALIVE_setPeriod(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Keep Alive changed : %d s -> %d s\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_COMMAND_interval(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d ms\n", WG_getTransferInterval());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = WG_getTransferInterval();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_setTransferInterval(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Transfer interval changed : %d ms -> %d ms\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_COMMAND_readyTimeout(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d s\n", WG_getReadyTimeout());
    }
    else if (argc == 2)
    {
        uint32_t    old_value = WG_getReadyTimeout();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_setReadyTimeout(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Ready Timeout changed : %d s -> %d s\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE SHELL_COMMAND_nop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", WG_getTransferNOP());
    }
    else
    {
        uint32_t    old_value = WG_getTransferNOP();
        uint32_t    value;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (WG_setTransferNOP(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        SHELL_printf("Transfer NOP changed : %d ms -> %d ms\n", old_value, value);
    }

    return  RET_OK;
}

RET_VALUE   SHELL_COMMAND_sleep(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 2)
    {
        uint32_t    sleepTime = 0;
        sleepTime = strtoul(argv[1], 0, 10);

        SHELL_printf("Sleep Time : %d\n", sleepTime);
        if (SYS_sleep(sleepTime) != RET_OK)
        {
            SHELL_printf("Sleep failed\n");
        }
    }

    return  RET_OK;
}

RET_VALUE SHELL_COMMAND_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SYS_reset();

    return  RET_OK;
}


RET_VALUE SHELL_COMMAND_trace(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;

    switch(argc)
    {
    case    1:
        {
            SHELL_printf("%16s : %s\n", "Trace", TRACE_getEnable()?"ON":"OFF");
            ret = RET_OK;
        }
        break;

    case    2:
        {
            if (strcasecmp(argv[1], "on") == 0)
            {
                TRACE_setEnable(true);
                SHELL_printf("Trace started.\n");
                ret = RET_OK;
            }
            else if (strcasecmp(argv[1], "off") == 0)
            {
                TRACE_setEnable(false);
                SHELL_printf("Trace stopped.\n");
                ret = RET_OK;
            }
        }
        break;
    }

    return  ret;
}
