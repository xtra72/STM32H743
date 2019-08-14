#ifndef TARGET_H_
#define TARGET_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "ret_value.h"
#include "assert.h"


#define TARGET_PRODUCT_NAME             "SWG"
#define TARGET_VERSION                  "19.02.10-A"

#define TARGET_SHELL_STACK_SIZE         256
#define TARGET_SHELL_PRIORITY           osPriorityNormal

#define TARGET_SHELL_COMMAND_MAX        128
#define TARGET_DEVICE_ID_LEN            8
#define TARGET_PASSWORD_LEN             32

#define TARGET_KEEP_ALIVE               3600

#define TARGET_TRANSFER_INTERVAL        20
#define TARGET_TRANSFER_NOP             3
#define TARGET_TRANSFER_NOP_MIN         1
#define TARGET_TRANSFER_NOP_MAX         3

#define TARGET_READY_TIMEOUT            1800

#define TARGET_ADC_COUNT_MAX            3
#define TARGET_ADC_CHANNEL_COUNT_MAX    8
#define TARGET_ADC_BANK_COUNT_MAX       2
#define TARGET_ADC_DATA_COUNT_MAX       1000

#define TARGET_FLASH_CONFIG_START_ADDR  0x08100000
#define TARGET_FLASH_CONFIG_SECTOR_SIZE 0x20000
#define TARGET_FLASH_CONFIG_SIZE        0x400
#define TARGET_FLASH_CONFIG_PER_SECTOR  (TARGET_FLASH_CONFIG_SECTOR_SIZE / TARGET_FLASH_CONFIG_SIZE)
#define TARGET_FLASH_CONFIG_SLOT_COUNT  (TARGET_FLASH_CONFIG_PER_SECTOR * 2)

#define TARGET_SDRAM_START_ADDRESS      0xD0000000
#define TARGET_SDRAM_SIZE               (32 * 1024 * 1024)
#define TARGET_SDRAM_START_HEAP_ADRESS  0xD0000000
#define TARGET_SDRAM_HEAP_SIZE          ((32 * 1024 * 1024) - 1024)

#define TARGET_SDRAM_STORAGE_ADRESS     (0xD0000000 + TARGET_SDRAM_HEAP_SIZE)
#define TARGET_SDRAM_STORAGE_SIZE       1024

#define TARGET_SCAN_INTERVAL            2
#define TARGET_SCAN_COUNT               (30 * 60 * 500)

#define TARGET_RF_SHORT_ADDRESS         2
#define TERGET_RF_FREQUENCY             920000000
#define TARGET_RF_POWER                 18
#define TARGET_RF_BITRATE               4800
#define TARGET_RF_PAYLOAD_MAX           60
#define TARGET_RF_TIMEOUT               1000

#include "main.h"

#endif