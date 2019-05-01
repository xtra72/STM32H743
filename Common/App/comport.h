#ifndef __COM_H__
#define __COM_H__

#include "target.h"
#include "serial.h"

#ifndef TARGET_COM_COMMAND_MAX
#define COM_COMMAND_MAX           32
#else
#define COM_COMMAND_MAX           TARGET_COM_COMMAND_MAX
#endif

#ifndef TARGET_COM_PRIORITY
#define COM_PRIORITY              osPriorityNormal
#else
#define COM_PRIORITY              TARGET_COM_PRIORITY
#endif

#ifdef  COM_STACK_SIZE
#define COM_STACK_SIZE            configMINIMAL_STACK_SIZE
#else
#define COM_STACK_SIZE            TARGET_COM_STACK_SIZE
#endif

typedef struct
{
    SERIAL_CONFIG   serial;
}   COM_CONFIG;

typedef struct _COM_COMMAND
{
    char*       name;
    RET_VALUE   (*function)(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);
    char*       shortHelp;
}   COM_COMMAND;

RET_VALUE   COM_init(const COM_COMMAND   *commands, uint32_t count);
RET_VALUE   COM_final(void);

RET_VALUE   COM_addCommands(const COM_COMMAND   *commands, uint32_t count);

RET_VALUE   COM_setConfig(COM_CONFIG* config);
RET_VALUE   COM_getConfig(COM_CONFIG* config);

RET_VALUE   COM_start(void);
RET_VALUE   COM_stop(void);

int         COM_getLine(char* line, uint32_t maxLength, bool secure);
RET_VALUE   COM_getc(char* ch, uint32_t timeout);
RET_VALUE   COM_printf(const char *format, ...);
RET_VALUE   COM_print(char* string);
RET_VALUE   COM_print2(char *title, char* buffer);
RET_VALUE   COM_dump(uint8_t *buffer, uint32_t length);

#endif
