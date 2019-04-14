#ifndef CONFIG_H_
#define CONFIG_H_

#include "target.h"
#include "shell.h"
#include "fi_time.h"

extern  char    *CONFIG_SHELL_BANNER[];

#define CONFIG_PRODUCT_NAME         "SWG"
#define CONFIG_VERSION              "19.02.10-A"

#define CONFIG_SHELL_STACK_SIZE     256
#define CONFIG_SHELL_PRIORITY       osPriorityNormal

#define CONFIG_SERIAL_NUMBER_LEN    32
#define CONFIG_PASSWORD_LEN         32

typedef struct
{
    uint32_t        crc;
    uint32_t        sequence;
    
    char            password[CONFIG_PASSWORD_LEN];
    char            serialNumber[CONFIG_SERIAL_NUMBER_LEN];
    
    SHELL_CONFIG    shell;
    TRACE_CONFIG    trace;
}   CONFIG;


RET_VALUE   CONFIG_init(void);
RET_VALUE   CONFIG_loadDefault(CONFIG* config);
RET_VALUE   CONFIG_load(CONFIG* config);
RET_VALUE   CONFIG_save(CONFIG* config);
bool        CONFIG_isValid(CONFIG* config);
RET_VALUE   CONFIG_clear(void);


extern  const   char firmwareVersion[];

extern  CONFIG  config;
#endif