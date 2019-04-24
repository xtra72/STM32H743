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

void        TRACE_init(TRACE_CONFIG* config);
RET_VALUE   TRACE_getConfig(TRACE_CONFIG* config);

void        TRACE_setEnable(bool enable);
bool        TRACE_getEnable(void);
void        TRACE_print(char *string);
void        TRACE_printUINT8(uint8_t value);
void        TRACE_printDump(uint8_t* value, uint32_t count, uint32_t columnLength);
RET_VALUE   TRACE_printf(const char* module, const char *format, ... );


#endif