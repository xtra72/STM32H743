#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "target.h"
#include "shell.h"
#include "time2.h"


const char* _titleName[] =
{
    "DBG",
    "TRC",
    "INF",
    "WRN",
    "ERR",
    "FTL"
};

static  TRACE_CONFIG    config_;
static char     buffer_[256];

void    TRACE_init(void)
{
}

RET_VALUE   TRACE_setConfig(TRACE_CONFIG* config)
{
    if (config != NULL)
    {
        memcpy(&config_, config, sizeof(TRACE_CONFIG));
    }

    return  RET_OK;
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

void    TRACE_printDump
(
    const char*     _module,
    TRACE_LEVEL     _level,
    const char*     _title,
    uint8_t*        _value,
    uint32_t        _count,
    uint32_t        _columnLength
)
{
    if (config_.enable)
    {
        static  char    header[64];
        uint32_t        headerLength = 0;
        TIME2 time;

        TIME2_get(&time);

//        strcpy(header, TIME2_toString(time, "[%Y%m%d%H%M%S]"));
        headerLength  = sprintf(header, "[%4d.%03d]", xTaskGetTickCount() / 1000, xTaskGetTickCount() % 1000);
        headerLength += snprintf(&header[headerLength], sizeof(header) - headerLength, "[%8s][%3s] : %s - ", _module, _titleName[_level], _title);

        while(_count > 0)
        {
            uint32_t    i;
            uint32_t    ulLen = 0;

            ulLen = sprintf(buffer_, "%s", header);
            for(i = 0 ; i < _count && ((_columnLength == 0) || (i < _columnLength)) ; i++)
            {
                ulLen += sprintf(&buffer_[ulLen], "%02x ", _value[i]);
            }
            ulLen += sprintf(&buffer_[ulLen], "\n");
            SHELL_print(buffer_);

            memset(header, ' ', strlen(header));
            _value += i;
            _count -= i;
        }
    }
}

RET_VALUE    TRACE_printf
(
    const char*     _module,
    TRACE_LEVEL     _level,
    const char*     _format,
    ...
)
{
    static  char    header[64];
    if (config_.enable)
    {
        va_list  ap;
        uint32_t headerLength = 0;
        TIME2 time;

        TIME2_get(&time);

//        strcpy(title, TIME2_toString(time, "[%Y%m%d%H%M%S]"));
        headerLength = sprintf(header, "[%4d.%03d]", xTaskGetTickCount() / 1000, xTaskGetTickCount() % 1000);
        headerLength += snprintf(&header[headerLength], sizeof(header) - headerLength, "[%8s][%3s] : ", _module, _titleName[_level]);

        va_start(ap, _format);
        vsnprintf(buffer_, sizeof(buffer_),  (char *)_format, ap );
        va_end(ap);

        SHELL_print2(header, buffer_);
    }

    return  RET_OK;
}
