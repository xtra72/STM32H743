#include "target.h"
#include "config.h"
#include "shell.h"
#include "rf.h"
#include "utils.h"
#include "comport.h"

#define __MODULE_NAME__ "COM"
#include "trace.h"

RET_VALUE COM_RF_help(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_reset(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_reg(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_send(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_recv(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_config(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_cmd(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
RET_VALUE COM_RF_test(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);

static const COM_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = COM_RF_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .function = COM_RF_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = COM_RF_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "reg",
        .function = COM_RF_reg,
        .shortHelp = "Registers"
    },
    {
        .name = "reset",
        .function = COM_RF_reset,
        .shortHelp = "Reset"
    },
    {
        .name = "send",
        .function = COM_RF_send,
        .shortHelp = "send"
    },
    {
        .name = "recv",
        .function = COM_RF_recv,
        .shortHelp = "recv"
    },
    {
        .name = "config",
        .function = COM_RF_config,
        .shortHelp = "Configuration"
    },
    {
        .name = "cmd",
        .function = COM_RF_cmd,
        .shortHelp = "Command"
    },
    {
        .name = "test",
        .function = COM_RF_test,
        .shortHelp = "Test"
    },
    {
        .name = NULL
    }
};

RET_VALUE   COM_rf(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        COM_printf("%16s : %s\n", "Status", RF_getStatusString(RF_getStatus()));
        COM_printf("%16s : %04x\n", "Short Address", RF_getShortAddress());
        COM_printf("%16s : %s\n", "Confirmed", RF_getConfirmed()?"On":"Off");
        COM_printf("%16s : %d bps\n", "Bitrate", RF_getBitrate());
        COM_printf("%16s : %d\n", "Payload Length", RF_getMaxPayloadLength());

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

RET_VALUE COM_RF_help(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    COM_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        COM_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE COM_RF_start(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RF_start();

    return  RET_OK;
}

RET_VALUE COM_RF_stop(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RF_stop();

    return  RET_OK;
}

RET_VALUE COM_RF_reset(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RF_reset();

    return  RET_OK;
}


RET_VALUE COM_RF_reg(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 1)
    {
        for(int i = 0 ; i < 64 ; i++)
        {
            COM_printf("%2d : %02x\n", i, RF_readRegister(i));
            osDelay(1);
        }
    }
    else if (argc == 2)
    {

    }

    return  RET_OK;
}



RET_VALUE COM_RF_send(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 4)
    {
        uint16_t    destAddress = 0;
        uint8_t     port = 0;
        uint8_t     buffer[64];
        uint32_t    length;

        if (!strToHex16(argv[1], &destAddress) || strToUint8(argv[2], &port) || !strToHexArray(argv[3], buffer, sizeof(buffer), &length))
        {
            COM_printf("Invalid arguments!\n");
            return  RET_INVALID_ARGUMENT;
        }

        RF_send(destAddress, port, buffer, length, 0, 5000);

    }
    else if (argc == 2)
    {

    }

    return  RET_OK;
}

RET_VALUE COM_RF_recv(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 1)
    {
        uint8_t     buffer[64];
        uint32_t    length;

        while(1)
        {
            uint16_t    destAddress;
            uint8_t     port;
            uint8_t     options;

            if (RF_recv(buffer, sizeof(buffer), &length, &destAddress, &port, &options, 10000) == RET_OK)
            {
                COM_printf("Packet received[%d, %4X, %d, %02X].\n", length, destAddress, port, options);
            }
        }

    }

    return  RET_OK;
}


RET_VALUE COM_RF_config(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 3)
    {
        if (strcasecmp(argv[1], "speed") == 0)
        {
            uint32_t    value = 0;

            if (!strToUint32(argv[2], &value))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            if (RF_setBitrate(value) == RET_OK)
            {
                COM_printf("RF speed : %d\n", RF_getBitrate());
            }
            else
            {
                COM_printf("Invalid argument!\n");
            }
        }
    }

    return  RET_OK;
}


RET_VALUE COM_RF_cmd(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    if (argc == 4)
    {
        uint8_t port = 0x80;

        if (strcasecmp(argv[1], "scan") == 0)
        {
            if (strcasecmp(argv[2], "start") == 0)
            {
                uint16_t    destAddress = 0;
                uint8_t data[1] = { RF_CMD_START_SCAN };

                if (!strToHex16(argv[3], &destAddress))
                {
                    COM_printf("Invalid argument!\n");
                }
                else
                {
                    RF_send(destAddress, port, data, sizeof(data), 0, 1000);
                }
            }
            else if (strcasecmp(argv[2], "stop") == 0)
            {
                uint16_t    destAddress = 0;
                uint8_t data[1] = { RF_CMD_STOP_SCAN };

                if (!strToHex16(argv[3], &destAddress))
                {
                    COM_printf("Invalid argument!\n");
                }
                else
                {
                    RF_send(destAddress, port, data, sizeof(data), 0, 1000);
                }
            }
        }
    }

    return  RET_OK;
}

RET_VALUE COM_RF_test(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    static  uint16_t    destAddress = 0;
    static  uint8_t     port = 0;
    static  uint32_t    count = 0;
    static  uint32_t    interval = 1000;
    static  uint32_t    payloadLength = 16;
    static  uint8_t     buffer[128];
    if (argc == 1)
    {
        COM_printf("%16s : %d\n", "Count", count);
        COM_printf("%16s : %d\n", "Interval", interval);
        COM_printf("%16s : %d\n", "Payload Length", payloadLength);
    }
    else if (argc == 2)
    {
        if (strcasecmp(argv[1], "start") == 0)
        {
            uint16_t    srcAddress;
            bool    stop = false;
            TickType_t  tickStart = xTaskGetTickCount();
            TickType_t  tickTimeout = interval / portTICK_PERIOD_MS;


            RF_clearStatistics();

            for(uint32_t i = 0 ; i < count && !stop; i++)
            {
                uint8_t     options = 0;

                memset(buffer, (uint8_t)i, payloadLength);
                if (RF_send(destAddress, port, buffer, payloadLength, 0, interval) == RET_OK)
                {
                    DEBUG("Successful transmission[%d]\n", i);

                    uint32_t    remain = 0;
                    uint32_t    receivedPayloadLength;
                    if (tickTimeout < (xTaskGetTickCount() - tickStart))
                    {
                        remain = 1 * portTICK_PERIOD_MS;
                    }
                    else
                    {
                        remain = (tickTimeout - (xTaskGetTickCount() - tickStart)) * portTICK_PERIOD_MS;
                    }

                    if (RF_recv(buffer, sizeof(buffer), &receivedPayloadLength, &srcAddress, &port, &options, remain) == RET_OK)
                    {
                        DEBUG("ACK received.\n");
                    }
                }
                else
                {
                    DEBUG("Failed to send data[%d]\n", i);
                }


                if (xTaskGetTickCount() < tickStart + tickTimeout)
                {
                    char ch = 0;
                    uint32_t    remain = (tickTimeout - (xTaskGetTickCount() - tickStart)) * portTICK_PERIOD_MS;

                    while(COM_getc(&ch, remain) == RET_OK)
                    {
                        if (ch == 'c')
                        {
                            stop = true;
                            break;
                        }
                    }
                }

                tickStart += interval / portTICK_PERIOD_MS;
            }
        }
        else if (strcasecmp(argv[1], "stop") == 0)
        {
        }
        else if (strcasecmp(argv[1], "show") == 0)
        {
            RF_STATISTICS   statistics;

            RF_getStatistics(&statistics);

            COM_printf("%16s : %d\n", "Tx Count", statistics.Tx.count);
            COM_printf("%16s : %d\n", "Tx Ack", statistics.Tx.ack);
            COM_printf("%16s : %d\n", "Rx Count", statistics.Rx.count);
            COM_printf("%16s : %d\n", "Rx Ack", statistics.Rx.ack);
        }
    }
    else if (argc == 3)
    {
        if (strcasecmp(argv[1], "dest") == 0)
        {
            uint16_t    value = 0;

            if (!strToHex16(argv[2], &value))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            destAddress = value;
        }
        else if (strcasecmp(argv[1], "port") == 0)
        {
            uint8_t    value = 0;

            if (!strToUint8(argv[2], &value))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            port = value;
        }
        else if (strcasecmp(argv[1], "count") == 0)
        {
            uint32_t    value = 0;

            if (!strToUint32(argv[2], &value))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            count = value;
        }
        else if (strcasecmp(argv[1], "interval") == 0)
        {
            uint32_t    value = 0;

            if (!strToUint32(argv[2], &value))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            interval = value;
        }
        else if (strcasecmp(argv[1], "length") == 0)
        {
            uint32_t    value = 0;

            if (!strToUint32(argv[2], &value))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            payloadLength = value;
        }
        else
        {
            COM_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

    }
    else if (argc == 5)
    {
        if (strcasecmp(argv[1], "send") == 0)
        {
            uint32_t    interval;
            uint32_t    count;
            uint8_t     buffer[64];
            uint32_t    length;

            if (!strToUint32(argv[2], &interval) || !strToUint32(argv[3], &count) || !strToHexArray(argv[4], buffer, sizeof(buffer), &length))
            {
                COM_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }
        }

    }

    return  RET_OK;
}

