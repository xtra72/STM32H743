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
#include "trace.h"


#define TARGET_PRODUCT_NAME             "SWG"
#define TARGET_VERSION                  "19.02.10-A"

#define TARGET_SHELL_STACK_SIZE         256
#define TARGET_SHELL_PRIORITY           osPriorityNormal

#define TARGET_SHELL_COMMAND_MAX        128
#define TARGET_SERIAL_NUMBER_LEN        32
#define TARGET_PASSWORD_LEN             32


#define TARGET_ADC_COUNT_MAX            3
#define TARGET_ADC_CHANNEL_COUNT_MAX    8
#define TARGET_ADC_BANK_COUNT_MAX       2
#define TARGET_ADC_DATA_COUNT_MAX       1000

#define TARGET_FLASH_CONFIG_START_ADDR  0x08030000
#define TARGET_FLASH_CONFIG_SIZE        0x400
#define TARGET_FLASH_CONFIG_SLOT_COUNT  4

#include "main.h"

#endif