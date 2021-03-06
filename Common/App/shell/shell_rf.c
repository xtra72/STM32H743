#include "target.h"
#include "config.h"
#include "shell.h"
#include "rf.h"
#include "wireguard.h"
#include "utils.h"
#include "shell_rf.h"

#define __MODULE_NAME__ "SHELL"
#include "trace.h"

RET_VALUE   SHELL_RF_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_motion(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_send(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_recv(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_address(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_frequency(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_power(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_cmd(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_statistics(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE   SHELL_RF_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .admin      = false,
        .function = SHELL_RF_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .admin      = true,
        .function = SHELL_RF_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .admin      = true,
        .function = SHELL_RF_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "reset",
        .admin      = true,
        .function = SHELL_RF_reset,
        .shortHelp = "Reset"
    },
    {
        .name = "motion",
        .admin      = true,
        .function = SHELL_RF_motion,
        .shortHelp = "motion <[start|stop]>"
    },
    {
        .name = "send",
        .admin      = true,
        .function = SHELL_RF_send,
        .shortHelp = "send"
    },
    {
        .name = "recv",
        .admin      = true,
        .function = SHELL_RF_recv,
        .shortHelp = "recv"
    },
    {
        .name = "config",
        .admin      = false,
        .function = SHELL_RF_config,
        .shortHelp = "Configuration"
    },
    {
        .name = "address",
        .admin      = false,
        .function = SHELL_RF_address,
        .shortHelp = "Address"
    },
    {
        .name = "frequency",
        .admin      = false,
        .function = SHELL_RF_frequency,
        .shortHelp = "Frequency"
    },
    {
        .name = "power",
        .admin      = false,
        .function = SHELL_RF_power,
        .shortHelp = "Power"
    },
    {
        .name = "cmd",
        .admin      = true,
        .function = SHELL_RF_cmd,
        .shortHelp = "Command"
    },
    {
        .name = "statistics",
        .admin      = false,
        .function = SHELL_RF_statistics,
        .shortHelp = "Statistics"
    },
    {
        .name = "test",
        .admin      = true,
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
        ret = SHELL_RF_info();
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

RET_VALUE SHELL_RF_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
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

RET_VALUE SHELL_RF_info(void)
{
    SHELL_printf("%16s : %d\n", "Short Address", RF_getShortAddress());
    SHELL_printf("%16s : %d Hz\n", "Frequency", RF_getFrequency());
    SHELL_printf("%16s : %d dBm\n", "Power", RF_getPower());
    SHELL_printf("%16s : %d ms\n", "Timeout", RF_getTimeout());
    SHELL_printf("%16s : %d\n", "Payload Length", RF_getMaxPayloadLength());

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

    if (argc == 3)
    {
        uint16_t    destAddress = 0;
        uint8_t     buffer[64];
        uint32_t    length;

        if (!strToHex16(argv[1], &destAddress) || !strToHexArray(argv[2], buffer, sizeof(buffer), &length))
        {
            SHELL_printf("Invalid arguments!\n");
            return  RET_INVALID_ARGUMENT;
        }

        RF_sendData(destAddress, buffer, length, 0, 5000);
    }

    return  RET_OK;
}

RET_VALUE SHELL_RF_motion(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        if (WG_getStatus() == WG_STATUS_READY)
        {
            SHELL_printf("Motion detection is in progress.\n");
        }
        else
        {
            SHELL_printf("It is not motion detection mode.\n");
        }
    }
    else if (argc == 2)
    {
        if (strcasecmp(argv[1], "start") == 0)
        {
            if (RF_motionDetectionStart(0) != RET_OK)
            {
                SHELL_printf("Motion detection start failed.\n");
            }
        }
        else if (strcasecmp(argv[1], "stop") == 0)
        {
            if (RF_motionDetectionStop(0) != RET_OK)
            {
                SHELL_printf("Motion detection stop failed.\n");
            }
        }
    }
    else
    {
        SHELL_printf("Invalid arguments.\n");
    }

    return  RET_OK;
}
RET_VALUE SHELL_RF_recv(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_ERROR;
}


RET_VALUE SHELL_RF_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        if (RF_CC1310_readConfig(2000) != RET_OK)
        {
            SHELL_printf("Config getting failed!\n");
        }
        else
        {
            SHELL_RF_info();
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


RET_VALUE SHELL_RF_address(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", RF_getShortAddress());
    }
    else if (argc == 2)
    {
        uint16_t    value = 0;

        if (!strToUint16(argv[1], &value))
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
    else
    {
        SHELL_printf("Invalid argument!\n");
         return  RET_INVALID_ARGUMENT;
    }

    return  RET_OK;
}

RET_VALUE SHELL_RF_frequency(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", RF_getFrequency());
    }
    else if (argc == 2)
    {
        uint32_t    value = 0;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (RF_setFrequency(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
        }

        SHELL_printf("RF Frequency : %d\n", RF_getFrequency());
    }
    else
    {
        SHELL_printf("Invalid argument!\n");
         return  RET_INVALID_ARGUMENT;
    }

    return  RET_OK;
}

RET_VALUE SHELL_RF_power(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_printf("%d\n", RF_getPower());
    }
    else if (argc == 2)
    {
        uint32_t    value = 0;

        if (!strToUint32(argv[1], &value))
        {
            SHELL_printf("Invalid argument!\n");
            return  RET_INVALID_ARGUMENT;
        }

        if (RF_setPower(value) != RET_OK)
        {
            SHELL_printf("Invalid argument!\n");
        }

        SHELL_printf("RF Power : %d\n", RF_getPower());
    }
    else
    {
        SHELL_printf("Invalid argument!\n");
         return  RET_INVALID_ARGUMENT;
    }

    return  RET_OK;
}

RET_VALUE SHELL_RF_cmd(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
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
        if (strcasecmp(argv[1], "motion") == 0)
        {
            RF_motionDetectionStart(0);
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
            uint32_t    loop;
            uint16_t    value;

            if (!strToUint32(argv[2], &count) || !strToUint32(argv[3], &loop))
            {
                SHELL_printf("Invalid argument!\n");
                return  RET_INVALID_ARGUMENT;
            }

            if ((count * 16 + 4) > RF_PAYLOAD_SIZE_MAX)
            {
                SHELL_printf("Data length must be less than or equal to %d.\n", (RF_PAYLOAD_SIZE_MAX - 4) / 16);
                return  RET_INVALID_ARGUMENT;
            }

            uint32_t    time = 0;
            for(uint32_t i = 0 ; i < loop ; i++)
            {
                buffer[0] = (time >> 24) & 0xFF;
                buffer[1] = (time >> 16) & 0xFF;
                buffer[2] = (time >>  8) & 0xFF;
                buffer[3] = (time      ) & 0xFF;

                for(uint32_t j = 0 ; j < count ; j++)
                {
                    for(uint32_t k = 0 ; k < 8 ; k++)
                    {
                        value = (uint16_t)(rand() % 0x10000);

                        buffer[4 + j*16 + k*2] = (value >> 8) & 0xFF;
                        buffer[4 + j*16 + k*2 + 1] = (value     ) & 0xFF;
                    }
                }

                RF_sendData(destAddress, buffer, 4 + count * 16, 0, 10);
                time += count*2;
            }
        }

    }

    return  RET_OK;
}
