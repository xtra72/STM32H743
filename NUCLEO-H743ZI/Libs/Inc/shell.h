#ifndef __SHELL_H__
#define __SHELL_H__

#include "target.h"
#include "serial.h"

typedef struct
{
    SERIAL_CONFIG   serial;
}   SHELL_CONFIG;

typedef struct _SHELL_COMMAND
{
    char*       name;
    RET_VALUE   (*function)(char *argv[], uint32_t argc, struct _SHELL_COMMAND const* command);
    char*       shortHelp;
}   SHELL_COMMAND;

RET_VALUE   SHELL_init(SHELL_CONFIG* config);
RET_VALUE   SHELL_final(void);

RET_VALUE   SHELL_getConfig(SHELL_CONFIG* config);

RET_VALUE   SHELL_start(void);
RET_VALUE   SHELL_stop(void);

int         SHELL_getLine(char* line, uint32_t maxLength, bool secure);
RET_VALUE   SHELL_printf(const char *format, ...);
RET_VALUE   SHELL_print(char* string);
RET_VALUE   SHELL_print2(char *title, char* buffer);
RET_VALUE   SHELL_dump(uint8_t *buffer, uint32_t length);
#endif
