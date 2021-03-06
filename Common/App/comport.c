/* Standard includes. */
#include "target.h"
#include "semphr.h"
#include "comport.h"
#include "config.h"
#include "system.h"
#include "serial.h"
#include "utils.h"

#define cmdMAX_INPUT_SIZE		    50
#define cmdQUEUE_LENGTH			    25
#define cmdASCII_DEL		        0x7F
#define cmdASCII_CR                 '\r'
#define cmdASCII_LF                 '\n'

#define cmdMAX_MUTEX_WAIT		pdMS_TO_TICKS( 300 )

#ifndef COM_CONFIG_MAX_ARGUMENT_COUNT
#define COM_CONFIG_MAX_ARGUMENT_COUNT 20
#endif

#ifndef COM_CONFIG_STREAM_BUFFER_SIZE
#define COM_CONFIG_STREAM_BUFFER_SIZE   128
#endif


/*-----------------------------------------------------------*/
static  SERIAL_HANDLE       serial_ = 0;
static  osThreadId          threadId_ = NULL;

/*
 * The task that implements the command console processing.
 */
static void COM_main( void const *paramaters );
uint32_t    COM_parser(char* line, char* arguments[], uint32_t maxArgumentCount );
char*       COM_token(char *line);
RET_VALUE COM_COMMAND_help(char *argv[], uint32_t argc, struct _COM_COMMAND const* command);

/*-----------------------------------------------------------*/

/* Const messages output by the command console. */
//static const char * const inputPassword_ = "Input Password : ";
static  const char * const newLine_ = "\r\n";
static  COM_CONFIG    config_;

static const COM_COMMAND   defaultCommands_[] =
{
    {
        .name       = "help",
        .function   = COM_COMMAND_help,
        .shortHelp  = "Help"
    }
};

static COM_COMMAND    commands_[COM_COMMAND_MAX];
static  uint32_t        commandCount = 0;

/*-----------------------------------------------------------*/
RET_VALUE   COM_init(const COM_COMMAND   *commands, uint32_t count)
{
    COM_addCommands(defaultCommands_, sizeof(defaultCommands_) / sizeof(COM_COMMAND));

    return  COM_addCommands(commands, count);
}

RET_VALUE   COM_addCommands(const COM_COMMAND   *commands, uint32_t count)
{
    ASSERT(commands);

    for(uint32_t i = 0 ; i < count && (commandCount < COM_COMMAND_MAX) ; i++)
    {
        memcpy(&commands_[commandCount++], &commands[i], sizeof(COM_COMMAND));
    }

    return  RET_OK;
}

RET_VALUE   COM_setConfig(COM_CONFIG* config)
{
    ASSERT(config != NULL);

    RET_VALUE   ret = RET_OK;

    memcpy(&config_, config, sizeof(COM_CONFIG));

    return  ret;
}

/*-----------------------------------------------------------*/
RET_VALUE   COM_getConfig(COM_CONFIG* config)
{
    ASSERT(config != NULL);

    RET_VALUE   ret = RET_OK;

    memcpy(config, &config_, sizeof(COM_CONFIG));

    return  ret;
}
/*-----------------------------------------------------------*/
RET_VALUE   COM_start(void)
{
    RET_VALUE   ret = RET_OK;

    if (threadId_ == NULL)
    {
        osThreadDef(comportTask, COM_main, COM_PRIORITY, 0, configMINIMAL_STACK_SIZE);
        threadId_ = osThreadCreate(osThread(comportTask), &config_);
        if (threadId_ == NULL)
        {
            ret = RET_ERROR;
        }
    }

    return  ret;
}

/*-----------------------------------------------------------*/
RET_VALUE   COM_stop(void)
{
    RET_VALUE   ret = RET_OK;

    if (threadId_ != 0)
    {
        osThreadTerminate(threadId_);
        threadId_ = 0;
    }

    return ret;
}

/*-----------------------------------------------------------*/

RET_VALUE COM_COMMAND_help(char *argv[], uint32_t argc, struct _COM_COMMAND const* command)
{
    RET_VALUE   ret = RET_INVALID_ARGUMENT;
    int32_t i, j;

    if (argc == 1)
    {
        for(i = 0 ; i < commandCount ; i++)
        {
            COM_printf("%-8s : %s\n", commands_[i].name, commands_[i].shortHelp,  &commands_[i]);
        }

        ret = RET_OK;
    }
    else
    {
        for(i = 1 ; i < argc ; i++)
        {
            ret = RET_INVALID_ARGUMENT;

            for(j = 0 ; j < commandCount ; j++)
            {
                if (strcasecmp(argv[i], commands_[j].name) == 0)
                {
                    char*   _argv[2];
                    _argv[0] = argv[i];
                    _argv[1] = argv[0];
                    ret = commands_[j].function(_argv, 2, &commands_[j]);
                    break;
                }
            }

            if (ret != RET_OK)
            {
                COM_printf("%s : Invalid argument\n", argv[i]);
            }
        }
    }

    return  RET_OK;
}

/*-----------------------------------------------------------*/

void COM_main( void const *params )
{
    RET_VALUE   ret;
    COM_CONFIG*   config = (COM_CONFIG*)params;
    int32_t     i, length = 0;
    static char line[ COM_CONFIG_STREAM_BUFFER_SIZE ];
    static char buffer[ COM_CONFIG_STREAM_BUFFER_SIZE ];

	/* Initialise the UART. */

    ret = SERIAL_open(&config->serial, COM_CONFIG_STREAM_BUFFER_SIZE, &serial_);
    if (ret != RET_OK)
    {
        return ;
    }

	for( ;; )
	{

        while(SERIAL_getc(serial_, (uint8_t *)line, 0) == RET_OK)
            ;

        memset(line, 0, sizeof(line));
        length = COM_getLine( line, sizeof(line), 0 );
        if (length != 0)
        {
            char*       argv[COM_CONFIG_MAX_ARGUMENT_COUNT];
            uint32_t    argc;

            SHELL_printf("CMD : %s\n", line);
            ret = RET_INVALID_ARGUMENT;
            strcpy(buffer, line);
            argc = COM_parser(buffer, argv, COM_CONFIG_MAX_ARGUMENT_COUNT);
            if (argc  > 0)
            {
                for(i = 0 ;  i < commandCount ; i++)
                {
                    if (strcasecmp(commands_[i].name, argv[0]) == 0)
                    {
                        ret = commands_[i].function(argv, argc, &commands_[i]);
                        break;
                    }
                }
            }

            if (ret == RET_OK)
            {
                COM_print("OK\n");
            }
            else
            {
                COM_print("ERR\n");
            }
        }
	}
}


int COM_getLine( char* line, uint32_t maxLength, bool secure)
{
    uint8_t value;
    int     readLength = 0;

	for( ;; )
	{
		while(  SERIAL_getc(serial_, &value, 100 ) != RET_OK)
        {
            vTaskDelay(0);
        }

        /* Echo the character back. */
        if (secure)
        {
            SERIAL_putc(serial_, '*', HAL_MAX_DELAY );
        }
        else
        {
            SERIAL_putc(serial_, value, HAL_MAX_DELAY );
        }

        /* Was it the end of the line? */
        if(( value == cmdASCII_CR) || ( value == cmdASCII_LF))
        {
            /* Just to space the output from the input. */
            SERIAL_puts(serial_, (uint8_t const *)newLine_, strlen( newLine_ ), HAL_MAX_DELAY );

            break;
        }

        if( ( value == '\b' ) || ( value == cmdASCII_DEL ) )
        {
            if( readLength > 0 )
            {
                readLength --;
                line[ readLength ] = '\0';
            }
        }
        else
        {
            if( ( value >= ' ' ) && ( value <= '~' ) )
            {
                if( readLength < maxLength)
                {
                    line[ readLength ] = value;
                    readLength ++;
                }
            }
        }
	}

    return  readLength;
}

RET_VALUE   COM_getc(char* ch, uint32_t timeout)
{
    return  SERIAL_getc(serial_, (uint8_t *)ch, timeout);
}

uint32_t    COM_parser(char* line, char* arguments[], uint32_t maxArgumentCount )
{
    char*   token;
    uint32_t    argumentCount = 0;

    if (strncasecmp(line, "AT+", 3) == 0)
    {
        line += 3;

        token = COM_token(line);
        if (token != NULL)
        {
            arguments[argumentCount++] = token;

            while(argumentCount < maxArgumentCount)
            {
                token = COM_token(NULL);
                //token = strtok(NULL, " ");
                if (token == NULL)
                {
                    break;
                }

                arguments[argumentCount++] = token;
            }
        }
    }

    return  argumentCount;
}

char*   COM_token(char* line)
{
    static  char*   head;
    char*   ptr;
    char*   token;

    if (line != NULL)
    {
        head = line;
    }

    while(*head != NULL)
    {
        if ((*head != ',') && (*head != ':'))
        {
            break;
        }
        head++;
    }

    if (*head == NULL)
    {
        return  NULL;
    }

    token = head;
    ptr = head;

    if (*ptr == '"')
    {
        ptr++;

        while(*ptr != NULL)
        {
            if (*ptr == '\\')
            {
                ptr++;
                if (*ptr == NULL)
                {
                    head = ptr;
                    return  NULL;
                }
            }
            else if (*ptr == '"')
            {
                ptr++;
                break;
            }

            ptr++;
        }

        if (*ptr != NULL)
        {
            head = ptr + 1;
            *ptr = NULL;
        }
        else
        {
            head = ptr;
        }
    }
    else
    {
        while(*ptr != NULL)
        {
            if ((*ptr == ',') || (*ptr == ':'))
            {
                break;
            }

            ptr++;
        }

        if (*ptr != NULL)
        {
            head = ptr + 1;
            *ptr = NULL;
        }
        else
        {
            head = ptr;
        }
    }

    return  token;
}

RET_VALUE    COM_printf
(
    const char *pFormat,
    ...
)
{
   va_list  ap;
   uint32_t ulLen = 0;
   static char  pBuff[256];

    va_start(ap, pFormat);
    vsnprintf(&pBuff[ulLen], sizeof(pBuff) - ulLen,  (char *)pFormat, ap );
    va_end(ap);

    COM_print(pBuff);

    return  RET_OK;
}

RET_VALUE   COM_print(char* buffer)
{
    /* Just to space the output from the input. */
    if (serial_ != NULL)
    {
        SERIAL_puts(serial_, (uint8_t *)buffer, strlen( buffer), HAL_MAX_DELAY );
    }

    return  RET_OK;
}

RET_VALUE   COM_print2(char *title, char* buffer)
{
    /* Just to space the output from the input. */
    if (serial_ != NULL)
    {
        char*       lines[10];
        uint32_t    lineCount;
        uint32_t    i;

        lineCount = seperateString(buffer, "\n", lines, 10);
        for(i = 0 ; i < lineCount ; i++)
        {
            SERIAL_puts(serial_, (uint8_t *)title, strlen( title), HAL_MAX_DELAY );
            SERIAL_puts(serial_, (uint8_t *)lines[i], strlen( lines[i]), HAL_MAX_DELAY );
            SERIAL_puts(serial_, "\n", 1, HAL_MAX_DELAY );
        }
    }

    return  RET_OK;
}

RET_VALUE   COM_dump(uint8_t *pBuffer, uint32_t ulLen)
{
    for(uint32_t i = 0 ; i < ulLen ; i++)
    {
        if (i == 0)
        {
            SERIAL_printf(serial_, "%02x", pBuffer[i]);
        }
        else
        {
            SERIAL_printf(serial_, " %02x", pBuffer[i]);
        }
    }
    SERIAL_printf(serial_, "\n");

    return  RET_OK;
}

