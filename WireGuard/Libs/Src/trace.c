#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "target.h"
#include "shell.h"
#include "time2.h"

static  TRACE_CONFIG    config_;
static char     buffer_[256];

void    TRACE_init(TRACE_CONFIG* config)
{
    if (config != NULL)
    {
        memcpy(&config_, config, sizeof(TRACE_CONFIG));
    }
}

RET_VALUE   TRACE_getConfig(TRACE_CONFIG* config)
{
    ASSERT(config != NULL);
    
    memcpy(config, &config_, sizeof(TRACE_CONFIG));
    
    return  RET_OK;
}

void        TRACE_setEnable(bool enable)
{
    config_.enable = enable;
}

bool    TRACE_getEnable(void)
{
    return  config_.enable;
}

void    TRACE_print(char *string)
{
    if (config_.enable)
    {
        SHELL_print(string);
    }
}

void    TRACE_printUINT8(uint8_t value)
{
    if (config_.enable)
    {
        char    pBuff[16];

        sprintf(pBuff, "%02x ", value);
        SHELL_print(pBuff);
    }
}

void    TRACE_printDump(uint8_t* value, uint32_t count, uint32_t columnLength)
{
    if (config_.enable)
    {    
        while(count > 0)
        {
            uint32_t    i;
            uint32_t    ulLen = 0;
            
            for(i = 0 ; i < count && ((columnLength == 0) || (i < columnLength)) ; i++)
            {
                ulLen += sprintf(&buffer_[ulLen], "%02x ", value[i]);
            }
            ulLen += sprintf(&buffer_[ulLen], "\n");
            SHELL_print(buffer_);
            
            value += i;
            count -= i;
        }
    }
}

RET_VALUE    TRACE_printf
(
    const char *module,
    const char *format, 
    ... 
)
{
    static  char    title[32];
    if (config_.enable)
    {
        va_list  ap;
        uint32_t titleLength = 0;
        TIME2 time;
        
        TIME2_get(&time);
        
        strcpy(title, TIME2_toString(time, "[%Y%m%d%H%M%S]"));
        titleLength = strlen(title);
        titleLength += snprintf(&title[titleLength], sizeof(title) - titleLength, "[%8s] : ", module);

        va_start(ap, format);
        vsnprintf(buffer_, sizeof(buffer_),  (char *)format, ap );
        va_end(ap);
       
        SHELL_print2(title, buffer_);
    }
    
    return  RET_OK;
}
