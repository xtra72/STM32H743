#include "target.h"
#include "config.h"
#include "crc16.h"

#ifndef TARGET_FLASH_CONFIG_START_ADDR
#define FLASH_CONFIG_START_ADDR 0x08030000  /* Start @ of user Flash area */
#else
#define FLASH_CONFIG_START_ADDR TARGET_FLASH_CONFIG_START_ADDR
#endif

#ifndef TARGET_FLASH_CONFIG_SIZE
#define FLASH_CONFIG_SIZE       0x00000400
#else
#define FLASH_CONFIG_SIZE       TARGET_FLASH_CONFIG_SIZE
#endif

#ifndef TARGET_FLASH_CONFIG_SLOT_COUNT
#define FLASH_CONFIG_SLOT_COUNT 4
#else
#define FLASH_CONFIG_SLOT_COUNT TARGET_FLASH_CONFIG_SLOT_COUNT
#endif

#define FLASH_CONFIG_END_ADDR   (FLASH_CONFIG_START_ADDR + ( FLASH_CONFIG_SIZE * FLASH_CONFIG_SLOT_COUNT))

RET_VALUE   CONFIG_saveAt(uint32_t index, CONFIG* config);

char    *CONFIG_SHELL_BANNER[] =
{
    "\r\n==========================================",
    "\r\n=   (C) COPYRIGHT 2018 FutureICT         =",
    "\r\n=                         SWG-V1         =",
    "\r\n=                         By DevTeam     =",
    "\r\n==========================================",
    "\r\n",
    "\r\n",
    NULL
};

const CONFIG  defaultConfig =
{
    .crc        =   0,
    .sequence   =   0,
    .deviceId   =  "01234567",
    .adc =
    {
        .enable = true,
        .channelCount = TARGET_ADC_CHANNEL_COUNT_MAX,
        .channels =
        {
            { 1, ADC_CHANNEL_3},
            { 1, ADC_CHANNEL_2},
            { 1, ADC_CHANNEL_0},
            { 1, ADC_CHANNEL_1},
            { 0, ADC_CHANNEL_0},
            { 0, ADC_CHANNEL_1},
            { 0, ADC_CHANNEL_3},
            { 0, ADC_CHANNEL_5},
        },
        .dataCount = TARGET_ADC_DATA_COUNT_MAX
    },
#if SUPPORT_DRAM
    .sdram =
    {
        .startAddress = TARGET_SDRAM_START_ADDRESS,
        .size = TARGET_SDRAM_SIZE,
        .startHeapAddress = TARGET_SDRAM_START_HEAP_ADRESS,
        .heapSize = TARGET_SDRAM_HEAP_SIZE
    },
#endif
    .scan =
    {
        .interval = TARGET_SCAN_INTERVAL,
        .count = TARGET_SCAN_COUNT
    },
    .rf =
    {
        .cc1310 =
        {
            .shortAddress = TARGET_RF_SHORT_ADDRESS,
            .frequency = TERGET_RF_FREQUENCY,
            .power = TARGET_RF_POWER,
            .maxPayloadLength = TARGET_RF_PAYLOAD_MAX,
            .timeout = TARGET_RF_TIMEOUT
        },
        .keepAlive        = TARGET_RF_KEEP_ALIVE,
        .transferInterval = TARGET_RF_TRANSFER_INTERVAL,
        .readyTimeout     = 60,
        .nop              = TARGET_RF_TRANSFER_NOP


    },
    .shell =
    {
        .serial =
        {
            .port = SERIAL_PORT_1,
            .baudrate = SERIAL_BAUDRATE_115200,
            .parity = SERIAL_PARITY_NONE,
            .dataBits = SERIAL_DATA_BITS_8,
            .stopBits = SERIAL_STOP_BITS_1,
            .priority = 5
        }
    },
    .trace =
    {
        .enable = true
    }
};

RET_VALUE   CONFIG_init(void)
{
    return  RET_OK;
}

RET_VALUE   CONFIG_loadDefault(CONFIG* config)
{
    ASSERT(config != NULL);

    memcpy(config, &defaultConfig, sizeof(CONFIG));

    return  RET_OK;
}

RET_VALUE   CONFIG_load(CONFIG* config)
{
    ASSERT(config != NULL);

    uint32_t    i;
    CONFIG*     last = NULL;

    for(i = 0 ; i < FLASH_CONFIG_SLOT_COUNT ; i++)
    {
        CONFIG* item = (CONFIG*)(FLASH_CONFIG_START_ADDR + FLASH_CONFIG_SIZE * i);
        if (CONFIG_isValid(item))
        {
            if ((last == NULL) ||(last->sequence < item->sequence))
            {
                last = item;
            }
        }
    }

    if (last == NULL)
    {
        return  RET_ERROR;
    }

    memcpy(config, last, sizeof(CONFIG));

    return  RET_OK;
}

RET_VALUE   CONFIG_save(CONFIG* config)
{
    ASSERT(config != NULL);

    uint32_t    i, index = 0;
    CONFIG*     last = NULL;

    for(i = 0 ; i < FLASH_CONFIG_SLOT_COUNT ; i++)
    {
        CONFIG* item = (CONFIG*)(FLASH_CONFIG_START_ADDR + FLASH_CONFIG_SIZE * i);
        if (CONFIG_isValid(item))
        {
            if ((last == NULL) ||(last->sequence < item->sequence))
            {
                index = i;
                last = item;
            }
        }
    }

    config->sequence++;
    config->crc = CRC16_calc(&config->sequence, sizeof(CONFIG) - sizeof(config->crc));

    if (last == NULL)
    {
        return  CONFIG_saveAt(0, config);
    }

    return  CONFIG_saveAt((index + 1) % FLASH_CONFIG_SLOT_COUNT, config);
}

RET_VALUE   CONFIG_saveAt(uint32_t index, CONFIG* config)
{
    return  RET_OK;
}


RET_VALUE   CONFIG_clear(void)
{
    return  RET_OK;
}

bool    CONFIG_isValid(CONFIG* config)
{
    ASSERT(config != NULL);

    return  config->crc == CRC16_calc(&config->sequence, sizeof(CONFIG) - sizeof(config->crc));
}

