#include "target.h"
#include "config.h"
#include "shell.h"
#include "rf.h"
#include "utils.h"
#include "shell_rf.h"

#define __MODULE_NAME__ "SHELL"
#include "trace.h"

RET_VALUE   SHELL_RF_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_send(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_recv(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_cmd(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_statistics(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

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
        .name = "recv",
        .function = SHELL_RF_recv,
        .shortHelp = "recv"
    },
    {
        .name = "config",
        .function = SHELL_RF_config,
        .shortHelp = "Configuration"
    },
    {
        .name = "cmd",
        .function = SHELL_RF_cmd,
        .shortHelp = "Command"
    },
    {
        .name = "statistics",
        .function = SHELL_RF_statistics,
        .shortHelp = "Statistics"
    },
    {
        .name = "test",
        .function = SHELL_RF_test,
        .shortHelp = "Test"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_RF(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        SHELL_RF_info();

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

RET_VALUE SHELL_RF_info(void)
{
    SHELL_printf("%16s : %s\n", "Status", RF_getStatusString(RF_getStatus()));
    SHELL_printf("%16s : %04x\n", "Short Address", RF_getShortAddress());
    SHELL_printf("%16s : %d\n", "Payload Length", RF_getMaxPayloadLength());
    SHELL_printf("%16s : %d ms\n", "Timeout", RF_getTimeout());

    return RET_OK;
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

RET_VALUE SHELL_RF_send(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 4)
    {
        uint16_t    destAddress = 0;
        uint8_t     port = 0;
        uint8_t     buffer[64];
        uint32_t    length;

        if (!strToHex16(argv[1], &destAddress) || strToUint8(argv[2], &port) || !strToHexArray(argv[3], buffer, sizeof(buffer), &length))
        {
            SHELL_printf("Invalid arguments!\n");
            return  RET_INVALID_ARGUMENT;
        }

        RF_sendData(destAddress, port, buffer, length, 0, 5000);

    }
    else if (argc == 2)
    {

    }

    return  RET_OK;
}

RET_VALUE SHELL_RF_recv(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_ERROR;
}


RET_VALUE SHELL_RF_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 2)
    {
        if (strcasecmp(argv[1], "get") == 0)
        {
            RF_CONFIG   config;

            if (RF_getConfig(&config, 5000) != RET_OK)
            {
                SHELL_printf("Config getting failed!\n");
            }
            else
            {
                SHELL_printf("Get Config\n");
                SHELL_printf("Frequency : %d\n", config.frequency);
            }
        }
        else if (strcasecmp(argv[1], "set") == 0)
        {
            RF_CONFIG   config;

            config.frequency = 920000000;

            if (RF_setConfig(&config, 5000) != RET_OK)
            {
                SHELL_printf("Config setting failed!\n");
            }
            else
            {
                SHELL_printf("Set Config\n");
                SHELL_printf("Frequency : %d\n", config.frequency);
            }
        }
    }
    else if (argc == 3)
    {
        if (strcasecmp(argv[1], "address") == 0)
        {
            uint16_t    value = 0;

            if (!strToUint16(argv[2], &value))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            if (RF_setShortAddress(value) != RET_OK)
            {
                SHELL_printf("Invalid argument!\n");
            }

            SHELL_printf("RF Short Address : %d\n", RF_getShortAddress());
        }
        else if (strcasecmp(argv[1], "address") == 0)
        {
            uint16_t    value = 0;

            if (!strToUint16(argv[2], &value))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            if (RF_setShortAddress(value) != RET_OK)
            {
                SHELL_printf("Invalid argument!\n");
            }

            SHELL_printf("RF Short Address : %d\n", RF_getShortAddress());
        }
    }

    return  RET_OK;
}


RET_VALUE SHELL_RF_cmd(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RET_VALUE   ret;

    if (argc == 4)
    {
        if (strcasecmp(argv[1], "scan") == 0)
        {
            if (strcasecmp(argv[2], "start") == 0)
            {
                uint16_t    destAddress = 0;

                if (!strToHex16(argv[3], &destAddress))
                {
                    SHELL_printf("Invalid argument!\n");
                }
                else
                {
                    RF_sendStartScan(destAddress, false, 200);
                }
            }
            else if (strcasecmp(argv[2], "stop") == 0)
            {
                uint16_t    destAddress = 0;

                if (!strToHex16(argv[3], &destAddress))
                {
                    SHELL_printf("Invalid argument!\n");
                }
                else
                {
                    RF_sendStopScan(destAddress, 0);
                }
            }
        }
    }
    else if (argc == 5)
    {
        if (strcasecmp(argv[1], "data") == 0)
        {
            if (strcasecmp(argv[2], "count") == 0)
            {
                uint32_t    nodeId;
                if (!strToUint32(argv[3], &nodeId))
                {
                    SHELL_printf("Invalid argument!\n");
                }

                ret = RF_sendRequestDataCount(nodeId, 5000);
                if (ret != RET_OK)
                {
                    SHELL_printf("Data count request failed!\n");
                }
            }
        }
    }
    if (argc == 6)
    {
        if (strcasecmp(argv[1], "data") == 0)
        {
            if (strcasecmp(argv[2], "get") == 0)
            {
                uint32_t    nodeId = 0;
                uint32_t    offset = 0;
                uint32_t    count = 0;

                if (!strToUint32(argv[3], &nodeId) || !strToUint32(argv[4], &offset) || !strToUint32(argv[5], &count))
                {
                    SHELL_printf("Invalid argument!\n");
                }

                ret = RF_sendRequestData(nodeId, offset, count, 5000);
                if (ret != RET_OK)
                {
                    SHELL_printf("Data get request failed!\n");
                }
            }
        }
    }

    return  RET_OK;
}


RET_VALUE SHELL_RF_statistics(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RF_STATISTICS   statistics;

    if (RF_getStatistics(&statistics) == RET_OK)
    {
        SHELL_printf("[ RX ]\n");
        SHELL_printf("%16s : %d\n", "Bytes", statistics.Rx.bytes);
        SHELL_printf("%16s : %d\n", "Packets", statistics.Rx.packets);
        SHELL_printf("%16s : %d\n", "Errors", statistics.Rx.errors);
        SHELL_printf("[ TX ]\n");
        SHELL_printf("%16s : %d\n", "Bytes", statistics.Tx.bytes);
        SHELL_printf("%16s : %d\n", "Packets", statistics.Tx.packets);
        SHELL_printf("%16s : %d\n", "Errors", statistics.Tx.errors);
    }
    else
    {
        SHELL_printf("Statistics getting failed!\n");
    }

    return  RET_OK;
}


RET_VALUE SHELL_RF_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    static  uint16_t    destAddress = 0;
    static  uint8_t     port = 0;
    static  uint32_t    count = 0;
    static  uint32_t    interval = 1000;
    static  uint32_t    payloadLength = 16;
    static  uint8_t     buffer[RF_PAYLOAD_SIZE_MAX] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    if (argc == 1)
    {
        SHELL_printf("%16s : %d\n", "Count", count);
        SHELL_printf("%16s : %d\n", "Interval", interval);
        SHELL_printf("%16s : %d\n", "Payload Length", payloadLength);
    }
    else if (argc == 2)
    {
        if (strcasecmp(argv[1], "start") == 0)
        {
            RF_startAutoTransfer(10);
        }
        else if (strcasecmp(argv[1], "stop") == 0)
        {
            RF_stopAutoTransfer(10);
        }
        else if (strcasecmp(argv[1], "motion") == 0)
        {
            RF_startMotionDetection(10);
        }
    }
    else if (argc == 3)
    {
        if (strcasecmp(argv[1], "dest") == 0)
        {
            uint16_t    value = 0;

            if (!strToHex16(argv[2], &value))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            destAddress = value;
        }
        else if (strcasecmp(argv[1], "port") == 0)
        {
            uint8_t    value = 0;

            if (!strToUint8(argv[2], &value))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            port = value;
        }
        else if (strcasecmp(argv[1], "count") == 0)
        {
            uint32_t    value = 0;

            if (!strToUint32(argv[2], &value))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            count = value;
        }
        else if (strcasecmp(argv[1], "interval") == 0)
        {
            uint32_t    value = 0;

            if (!strToUint32(argv[2], &value))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            interval = value;
        }
        else if (strcasecmp(argv[1], "data") == 0)
        {
            if (!strToHexArray(argv[2], buffer, sizeof(buffer), &payloadLength))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }
        }
        else
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }


    }
    else if (argc == 4)
    {
        if (strcasecmp(argv[1], "send") == 0)
        {
            uint32_t    count;
            uint32_t    length;

            if (!strToUint32(argv[2], &length) || !strToUint32(argv[3], &count))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            if (length > RF_PAYLOAD_SIZE_MAX)
            {
                SHELL_printf("Data length must be less than or equal to %d.\n", RF_PAYLOAD_SIZE_MAX);
                return  RET_INVALID_ARGUMENT;
            }

            for(uint32_t i = 0 ; i < length ; i++)
            {
                buffer[i] = i;
            }

            for(uint32_t i = 0 ; i < count ; i++)
            {
                RF_sendData(destAddress, port, buffer, length, 0, 10);
            }
        }

    }

    return  RET_OK;
}

