#ifndef SERIAL_H_
#define SERIAL_H_

#include "target.h"
#include "semphr.h"

#define SERIAL_MAX_MUTEX_WAIT		pdMS_TO_TICKS( 300 )

typedef struct SERIAL_STRUCT
{
    UART_HandleTypeDef  uart;
    IRQn_Type           irq;
    SemaphoreHandle_t   semaphore;
    xQueueHandle        rxQueue;
    uint8_t             *rxBuffer;
    uint8_t             buffer[8];
    void                (*outputEnable)(bool enable);

}   SERIAL, *SERIAL_HANDLE;

typedef enum
{ 
    SERIAL_PORT_1,
    SERIAL_PORT_2,
    SERIAL_PORT_3,
    SERIAL_PORT_4,
    SERIAL_PORT_5,
    SERIAL_PORT_MAX
} SERIAL_PORT;

typedef enum 
{ 
	SERIAL_PARITY_NONE, 
	SERIAL_PARITY_ODD, 
	SERIAL_PARITY_EVEN, 
	SERIAL_PARITY_MARK, 
	SERIAL_PARITY_SPACE,
    SERIAL_PARITY_MAX
} SERIAL_PARITY;

typedef enum 
{ 
	SERIAL_STOP_BITS_1, 
	SERIAL_STOP_BITS_2 
} SERIAL_STOP_BITS;

typedef enum 
{ 
	SERIAL_DATA_BITS_5, 
	SERIAL_DATA_BITS_6, 
	SERIAL_DATA_BITS_7, 
	SERIAL_DATA_BITS_8,
	SERIAL_DATA_BITS_9 
} SERIAL_DATA_BITS;

typedef enum 
{ 
	SERIAL_BAUDRATE_50,		
	SERIAL_BAUDRATE_75,		
	SERIAL_BAUDRATE_110,		
	SERIAL_BAUDRATE_134,		
	SERIAL_BAUDRATE_150,    
	SERIAL_BAUDRATE_200,
	SERIAL_BAUDRATE_300,		
	SERIAL_BAUDRATE_600,		
	SERIAL_BAUDRATE_1200,	
	SERIAL_BAUDRATE_1800,	
	SERIAL_BAUDRATE_2400,   
	SERIAL_BAUDRATE_4800,
	SERIAL_BAUDRATE_9600,		
	SERIAL_BAUDRATE_19200,	
	SERIAL_BAUDRATE_38400,	
	SERIAL_BAUDRATE_57600,	
	SERIAL_BAUDRATE_115200
} SERIAL_BAUDRATE;

typedef struct
{
    SERIAL_PORT         port;
    SERIAL_BAUDRATE     baudrate;
    SERIAL_DATA_BITS    dataBits;
    SERIAL_PARITY       parity;
    SERIAL_STOP_BITS    stopBits;
    uint8_t             priority;
}   SERIAL_CONFIG;

RET_VALUE       SERIAL_open(SERIAL_CONFIG* config, uint32_t bufferLength, SERIAL_HANDLE* serial);
RET_VALUE       SERIAL_close(SERIAL_HANDLE serial);

uint32_t        SERIAL_gets(SERIAL_HANDLE serial, uint8_t* value, uint32_t size, uint32_t timeout);
RET_VALUE       SERIAL_puts(SERIAL_HANDLE serial, const uint8_t* const buffer, uint32_t length, uint32_t timeout);
uint8_t         SERIAL_getc(SERIAL_HANDLE serial, uint8_t* value, uint32_t timeout);
uint8_t         SERIAL_putc(SERIAL_HANDLE serial, uint8_t value, uint32_t timeout);
RET_VALUE       SERIAL_printf(SERIAL_HANDLE   serial, const char *format, ... );

RET_VALUE       SERIAL_waitForSemaphore(SERIAL_HANDLE serial);

char*           SERIAL_baudrateName(SERIAL_BAUDRATE baudrate);
char*           SERIAL_dataBitsName(SERIAL_DATA_BITS dataBits);
char*           SERIAL_stopBitsName(SERIAL_STOP_BITS stopBits);
char*           SERIAL_parityName(SERIAL_PARITY parity);
#endif

