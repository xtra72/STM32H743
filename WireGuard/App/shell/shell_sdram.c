#include "target.h"
#include "config.h"
#include "shell.h"
#include "rf.h"
#include "utils.h"

RET_VALUE SHELL_SDRAM_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_read(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_write(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_fill(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
RET_VALUE SHELL_SDRAM_print(uint32_t address, uint32_t length, uint32_t size);

static const SHELL_COMMAND   commandSet_[] =
{
    {
        .name = "help",
        .function = SHELL_SDRAM_help,
        .shortHelp = "Help"
    },
    {
        .name = "start",
        .function = SHELL_SDRAM_start,
        .shortHelp = "Start"
    },
    {
        .name = "stop",
        .function = SHELL_SDRAM_stop,
        .shortHelp = "Stop"
    },
    {
        .name = "read",
        .function = SHELL_SDRAM_read,
        .shortHelp = "read"
    },
    {
        .name = "fill",
        .function = SHELL_SDRAM_fill,
        .shortHelp = "Fill"
    },
    {
        .name = "write",
        .function = SHELL_SDRAM_write,
        .shortHelp = "write"
    },
    {
        .name = "reset",
        .function = SHELL_SDRAM_reset,
        .shortHelp = "Reset"
    },
    {
        .name = NULL
    }
};

RET_VALUE   SHELL_sdram(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command)
{
    RET_VALUE   ret = RET_INVALID_COMMAND;
    if (argc == 1)
    {
        SDRAM_CONFIG    config;

        SDRAM_getConfig(&config);

        SHELL_printf("[ %16s ]\n", "Total");
        SHELL_printf("%16s : %08x\n", "Start Address", config.startAddress);
        SHELL_printf("%16s : %08x\n", "Size", config.size);
        SHELL_printf("[ %16s ]\n", "Heap");
        SHELL_printf("%16s : %08x\n", "Start Address", config.startHeapAddress);
        SHELL_printf("%16s : %08x\n", "Total Size", config.heapSize);
        SHELL_printf("%16s : %08x\n", "Free Size", SDRAM_getFreeHeapSize( ));

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

RET_VALUE SHELL_SDRAM_help(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    SHELL_COMMAND const*   subcommand = commandSet_;
    while(subcommand->name != NULL)
    {
        SHELL_printf("%-8s : %s\n", subcommand->name, subcommand->shortHelp, subcommand);

        subcommand++;
    }

    return  RET_OK;
}


RET_VALUE SHELL_SDRAM_start(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{

    return  RET_OK;
}

RET_VALUE SHELL_SDRAM_stop(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_SDRAM_reset(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    RF_reset();

    return  RET_OK;
}

RET_VALUE SHELL_SDRAM_read(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 1)
    {
        SHELL_SDRAM_print(0xD0000000, 256, 1);
    }
    else if (argc == 2)
    {
        uint32_t    address;

        if (!strToHex32(argv[1], &address))
        {
            SHELL_SDRAM_print(address, 256, 1);
        }
    }
    else if (argc == 3)
    {
        uint32_t    address;
        uint32_t    size;

        if (!strToHex32(argv[1], &address) || !strToUint32(argv[2], &size))
        {
            SHELL_SDRAM_print(address, size, 1);
        }
    }
    else
    {
        return  RET_INVALID_ARGUMENT;
    }

    return  RET_OK;
}

RET_VALUE SHELL_SDRAM_fill(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    if (argc == 4)
    {
        uint32_t    address;
        uint32_t    value;
        uint32_t    size;

        if (!strToHex32(argv[1], &address) || !strToHex32(argv[2], &value) || !strToUint32(argv[3], &size))
        {
            return  RET_INVALID_ARGUMENT;
        }

        if (SDRAM_fill(address, value, size) != RET_OK)
        {
            SHELL_printf("SDRAM fill failed.\n[%08x/%d/%d]", address, value, size);
            return  RET_INVALID_ARGUMENT;
        }
    }
    else
    {
        return  RET_INVALID_ARGUMENT;
    }

    return  RET_OK;
}

RET_VALUE SHELL_SDRAM_write(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command)
{
    return  RET_OK;
}

RET_VALUE SHELL_SDRAM_print(uint32_t address, uint32_t length, uint32_t size)
{
    uint32_t    startAddress = address & ~0x0000000F;
    uint32_t    endAddress = (address + length + 15) & ~0x0000000F;

    switch(size)
    {
    case    1:
        {
            while(startAddress < endAddress)
            {
                SHELL_printf("%08X : ", startAddress);
                for(int i = 0 ; i < 16 ; i++)
                {
                    if (startAddress < address)
                    {
                        SHELL_printf("   ");
                    }
                    else if (address + length <= startAddress)
                    {
                        SHELL_printf("   ");
                    }
                    else
                    {
                        SHELL_printf("%02X ", *((uint8_t*)startAddress));
                    }

                    startAddress += 1;
                }
                SHELL_printf("\n");
            }
        }
        break;

    case    2:
        {
            while(startAddress < endAddress)
            {
                SHELL_printf("%08X : ", startAddress);
                for(int i = 0 ; i < 8 ; i++)
                {
                    if (startAddress < address)
                    {
                        SHELL_printf("   ");
                    }
                    else if (address + length <= startAddress)
                    {
                        SHELL_printf("   ");
                    }
                    else
                    {
                        SHELL_printf("%04X ", *((uint16_t*)startAddress));
                    }

                    startAddress += 2;
                }
                SHELL_printf("\n");
            }
        }
        break;

    case    4:
        {
            while(startAddress < endAddress)
            {
                SHELL_printf("%08X : ", startAddress);
                for(int i = 0 ; i < 4 ; i++)
                {
                    if (startAddress < address)
                    {
                        SHELL_printf("   ");
                    }
                    else if (address + length <= startAddress)
                    {
                        SHELL_printf("   ");
                    }
                    else
                    {
                        SHELL_printf("%08X ", *((uint32_t*)startAddress));
                    }

                    startAddress += 4;
                }
                SHELL_printf("\n");
            }
        }
        break;

    default:
        return  RET_ERROR;
    }

    return  RET_OK;
}