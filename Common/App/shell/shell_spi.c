#include "target.h"
#include "config.h"
#include "shell.h"
#include "sx1231.h"
#include "utils.h"
#include "crc16.h"
#include "spi.h"

RET_VALUE SHELL_SPI_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SPI_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SPI_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SPI_read(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SPI_write(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SPI_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .admin      = true,
        .function = SHELL_SPI_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .admin      = true,
        .function = SHELL_SPI_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .admin      = true,
        .function = SHELL_SPI_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "read",
        .admin      = true,
        .function = SHELL_SPI_read,
        .shortHelp = "Registers"
    },
    {
        .name = "write",
        .admin      = true,
        .function = SHELL_SPI_write,
        .shortHelp = "Write"
    },
    {
        .name = "test",
        .admin      = true,
        .function = SHELL_SPI_test,
        .shortHelp = "test"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_spi(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        char*   argc[] = {"help"};

        ret = SHELL_SPI_help(argc, 1, &commandSet_[0]);
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

RET_VALUE SHELL_SPI_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
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


RET_VALUE SHELL_SPI_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{

    return  RET_OK;
}

RET_VALUE SHELL_SPI_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_SPI_read(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    uint8_t buffer[16];

    memset(buffer, 0, sizeof(buffer));

    if (SPI_receive(buffer, 16, 10) == RET_OK)
    {
        for(uint8_t i = 0 ; i < sizeof(buffer) ; i++)
        {
            SHELL_printf("%02X ", buffer[i]);
        }
        SHELL_printf("\n");
    }
    else
    {
        SHELL_printf("Receive timeout!\n");
    }

    return  RET_OK;
}

RET_VALUE SHELL_SPI_write(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    uint8_t buffer[16];

    for(uint8_t i = 0 ; i < sizeof(buffer) ; i++)
    {
        buffer[i] = 'A' + i;
    }

    SPI_transmit(buffer, 16, 10);

    return  RET_OK;
}

RET_VALUE SHELL_SPI_test(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 4)
    {
        uint32_t    length;
        uint32_t    count;
        if (!strToUint32(argv[2], &length) || !strToUint32(argv[3], &count))
        {
            SHELL_printf("Invalid arguments\n");
            return  RET_OK;
        }

        if (strcmp(argv[1], "r") == 0)
        {
#if 0
            RF_SPI_FRAME    frame;
            uint32_t    total = 0;
            uint32_t    failed = 0;

            for(uint32_t i = 0 ; i < count ; i++)
            {
                memset(&frame, 0, sizeof(frame));


                while (HAL_GPIO_ReadPin(RF_DIO3_GPIO_Port, RF_DIO3_Pin) != GPIO_PIN_RESET)
                {
                    SHELL_printf("Waiting for slave ready\n");
                }

                SPI_receive((uint8_t*)&frame, sizeof(frame), 10);
                total++;
                if (frame.crc != CRC16_calc(frame.payload, frame.length))
                {
                    failed++;
//                    SHELL_dump((uint8_t *)&frame, sizeof(frame));
                }

                if (total % 10 == 0)
                {
                    SHELL_printf("%4d %4d\n", total, failed);
                }
            }

            SHELL_printf("%4d %4d\n", total, failed);
#endif

        }
        else if (strcmp(argv[1], "w") == 0)
        {
#if 0
            for(uint32_t i = 0 ; i < count ; i++)
            {
                frame.cmd = i + 1;
                frame.length = i%60 + 1;
                for(uint32_t j = 0 ; j < frame.length ; j++)
                {
                    frame.payload[j] = j;
                }

                frame.crc = CRC16_calc(frame.payload, frame.length);

                while (HAL_GPIO_ReadPin(RF_DIO3_GPIO_Port, RF_DIO3_Pin) != GPIO_PIN_RESET)
                {
                    SHELL_printf("Waiting for slave ready\n");
                }

                SPI_transmit((uint8_t*)&frame, sizeof(frame), 10);
                total++;
                if (total % 10 == 0)
                {
                    SHELL_printf("%4d\n", total);
                }
            }
            SHELL_printf("%4d\n", total);
#endif
        }
    }

    return  RET_OK;
}

