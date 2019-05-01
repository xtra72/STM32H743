#ifndef _TRACE_H
#define _TRACE_H

#include <stdbool.h>
#include <stdint.h>
#include "ret_value.h"

typedef struct
{
    bool        enable;
    struct
    {
        bool    input;
        bool    send;
    }   dump;
}   TRACE_CONFIG;

typedef enum
{
    TRACE_LEVEL_DEBUG,
    TRACE_LEVEL_TRACE,
    TRACE_LEVEL_INFO,
    TRACE_LEVEL_WARNING,
    TRACE_LEVEL_ERROR,
    TRACE_LEVEL_FATAL
}   TRACE_LEVEL;

void        TRACE_init(void);

RET_VALUE   TRACE_setConfig(TRACE_CONFIG* config);
RET_VALUE   TRACE_getConfig(TRACE_CONFIG* config);

void        TRACE_setEnable(bool enable);
bool        TRACE_getEnable(void);
void        TRACE_print(char *string);
void        TRACE_printUINT8(uint8_t value);
void        TRACE_printDump(const char* _module, TRACE_LEVEL _level, const char* title, uint8_t* value, uint32_t count, uint32_t columnLength);
RET_VALUE   TRACE_printf(const char* _module, TRACE_LEVEL _level, const char *format, ... );

#ifndef __MODULE_NAME__
#define __MODULE_NAME__ "ALL"
#endif

#define DEBUG(...)      TRACE_printf(__MODULE_NAME__, TRACE_LEVEL_DEBUG, __VA_ARGS__)
#define TRACE(...)      TRACE_printf(__MODULE_NAME__, TRACE_LEVEL_TRACE, __VA_ARGS__)
#define INFO(...)       TRACE_printf(__MODULE_NAME__, TRACE_LEVEL_INFO, __VA_ARGS__)
#define WARNING(...)    TRACE_printf(__MODULE_NAME__, TRACE_LEVEL_WARNING, __VA_ARGS__)
#define ERROR(...)      TRACE_printf(__MODULE_NAME__, TRACE_LEVEL_ERROR, __VA_ARGS__)
#define FATAL(...)      TRACE_printf(__MODULE_NAME__, TRACE_LEVEL_FATAL, __VA_ARGS__)

#define DEBUG_DUMP(t, v, c, l)      TRACE_printDump(__MODULE_NAME__, TRACE_LEVEL_DEBUG, (t), (v), (c), (l))
#define TRACE_DUMP(t, v, c, l)      TRACE_printDump(__MODULE_NAME__, TRACE_LEVEL_TRACE, (t), (v), (c), (l))
#define INFO_DUMP(t, v, c, l)       TRACE_printDump(__MODULE_NAME__, TRACE_LEVEL_INFO, (t), (v), (c), (l))
#define WARNING_DUMP(t, v, c, l)    TRACE_printDump(__MODULE_NAME__, TRACE_LEVEL_WARNING, (t), (v), (c), (l))
#define ERROR_DUMP(t, v, c, l)      TRACE_printDump(__MODULE_NAME__, TRACE_LEVEL_ERROR, (t), (v), (c), (l))
#define FATAL_DUMP(t, v, c, l)      TRACE_printDump(__MODULE_NAME__, TRACE_LEVEL_FATAL, (t), (v), (c), (l))

#endif