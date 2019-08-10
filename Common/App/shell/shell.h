#ifndef __SHELL_H__
#define __SHELL_H__

#include "target.h"
#include "serial.h"

#ifndef TARGET_SHELL_COMMAND_MAX
#define SHELL_COMMAND_MAX           32
#else
#define SHELL_COMMAND_MAX           TARGET_SHELL_COMMAND_MAX
#endif

#ifndef TARGET_SHELL_PRIORITY
#define SHELL_PRIORITY              osPriorityNormal
#else
#define SHELL_PRIORITY              TARGET_SHELL_PRIORITY
#endif

#ifdef  SHELL_STACK_SIZE
#define SHELL_STACK_SIZE            configMINIMAL_STACK_SIZE
#else
#define SHELL_STACK_SIZE            TARGET_SHELL_STACK_SIZE
#endif

typedef struct
{
    SERIAL_CONFIG   serial;
}   SHELL_CONFIG;

typedef struct _SHELL_COMMAND
{
    char*       name;
    bool        admin;
    RET_VALUE   (*function)(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
    char*       shortHelp;
}   SHELL_COMMAND;

RET_VALUE   SHELL_init(const SHELL_COMMAND   *commands, uint32_t count);
RET_VALUE   SHELL_final(void);

RET_VALUE   SHELL_addCommands(const SHELL_COMMAND   *commands, uint32_t count);

RET_VALUE   SHELL_setConfig(SHELL_CONFIG* config);
RET_VALUE   SHELL_getConfig(SHELL_CONFIG* config);

bool        SHELL_getAdmin(void);
RET_VALUE   SHELL_setAdmin(bool _enable);

RET_VALUE   SHELL_start(void);
RET_VALUE   SHELL_stop(void);

int         SHELL_getLine(char* line, uint32_t maxLength, bool secure);
RET_VALUE   SHELL_getc(char* ch, uint32_t timeout);
RET_VALUE   SHELL_printf(const char *format, ...);
RET_VALUE   SHELL_print(char* string);
RET_VALUE   SHELL_print2(char *title, char* buffer);
RET_VALUE   SHELL_dump(uint8_t *buffer, uint32_t length);

#endif
