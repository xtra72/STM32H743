#ifndef CONFIG_H_
#define CONFIG_H_

#include "target.h"
#include "shell.h"
#include "comport.h"
#include "adc.h"
#include "sdram.h"
#include "time2.h"
#include "rf.h"
#include "scan.h"

#ifndef TARGET_PASSWORD_LEN
#define CONFIG_PASSWORD_LEN 32
#else
#define CONFIG_PASSWORD_LEN TARGET_PASSWORD_LEN
#endif

#ifndef TARGET_DEVICE_ID_LEN
#define CONFIG_DEVICE_ID_LEN    32
#else
#define CONFIG_DEVICE_ID_LEN TARGET_DEVICE_ID_LEN
#endif

#ifndef TARGET_PRODUCT_NAME
#define CONFIG_PRODUCT_NAME "SWG"
#else
#define CONFIG_PRODUCT_NAME TARGET_PRODUCT_NAME
#endif

#ifndef TARGET_VERSION
#define CONFIG_VERSION  "19.02.10-A"
#else
#define CONFIG_VERSION  TARGET_VERSION
#endif

extern  char    *CONFIG_SHELL_BANNER[];

typedef struct
{
    uint32_t        crc;
    uint32_t        sequence;

    char            password[CONFIG_PASSWORD_LEN + 1];
    char            deviceId[CONFIG_DEVICE_ID_LEN + 1];

    uint32_t        transferInterval;
    uint32_t        keepAlive;
    uint32_t        readyTimeout;
    uint32_t        nop;

    ADC_CONFIG      adc;
#if SUPPORT_DRAM
    SDRAM_CONFIG    sdram;
#endif
    RF_CONFIG       rf;

    SCAN_CONFIG     scan;
    SHELL_CONFIG    shell;
#if SUPPORT_COM
    COM_CONFIG      comport;
#endif
    TRACE_CONFIG    trace;
}   CONFIG;

RET_VALUE   CONFIG_init(void);
RET_VALUE   CONFIG_loadDefault(CONFIG* config);
RET_VALUE   CONFIG_load(CONFIG* config);
RET_VALUE   CONFIG_save(CONFIG* config);
bool        CONFIG_isValid(CONFIG* config);
RET_VALUE   CONFIG_clear(void);

const ADC_DESCRIPTION*    CONFIG_getADCDescription(uint32_t id);

extern  const   char firmwareVersion[];

#endif