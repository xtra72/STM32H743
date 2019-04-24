#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "assert.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "assert.h"
/* Library includes. */

/* Demo application includes. */
#include "serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define SERIAL_INVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define SERIAL_NO_BLOCK						( ( TickType_t ) 0 )
#define SERIAL_TX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )

#define SERIAL_RX_BUFFER_SIZE                8
/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void    vUARTInterruptHandler( void );
void    SERIAL_outputEnable(bool enable);

/*-----------------------------------------------------------*/
static  SERIAL  serials_[SERIAL_PORT_MAX];

RET_VALUE       SERIAL_open(SERIAL_CONFIG* config, uint32_t bufferLength, SERIAL_HANDLE *serialHandle)
{
    ASSERT((config != NULL) && (serialHandle != NULL));
    
    RET_VALUE   ret = RET_OK;
    
    switch(config->port)
    {
#ifdef  USART1
    case    SERIAL_PORT_1:    serials_[config->port].uart.Instance = USART1; break;
#endif
#ifdef  USART2
    case    SERIAL_PORT_2:    serials_[config->port].uart.Instance = USART2; break;
#endif
#ifdef  USART3
    case    SERIAL_PORT_3:    serials_[config->port].uart.Instance = USART3; break;
#endif
#ifdef  UART4
    case    SERIAL_PORT_4:    serials_[config->port].uart.Instance = UART4; break;
#endif

#ifdef  UART5
    case    SERIAL_PORT_5:    serials_[config->port].uart.Instance = UART5; break;
#endif
    default:
        return  RET_ERROR;
    }
    
    switch(config->baudrate)
    {
    case    SERIAL_BAUDRATE_50:		serials_[config->port].uart.Init.BaudRate = 50;   break;
    case    SERIAL_BAUDRATE_75:	    serials_[config->port].uart.Init.BaudRate = 75;   break;
    case    SERIAL_BAUDRATE_110:	serials_[config->port].uart.Init.BaudRate = 110;   break;
    case    SERIAL_BAUDRATE_134:	serials_[config->port].uart.Init.BaudRate = 134;   break;
    case    SERIAL_BAUDRATE_150:	serials_[config->port].uart.Init.BaudRate = 150;   break;
    case    SERIAL_BAUDRATE_200:	serials_[config->port].uart.Init.BaudRate = 200;   break;
    case    SERIAL_BAUDRATE_300:	serials_[config->port].uart.Init.BaudRate = 300;   break;
    case    SERIAL_BAUDRATE_600:	serials_[config->port].uart.Init.BaudRate = 600;   break;
    case    SERIAL_BAUDRATE_1200:	serials_[config->port].uart.Init.BaudRate = 1200;   break;
    case    SERIAL_BAUDRATE_1800:	serials_[config->port].uart.Init.BaudRate = 1800;   break;
    case    SERIAL_BAUDRATE_2400:	serials_[config->port].uart.Init.BaudRate = 2400;   break;
    case    SERIAL_BAUDRATE_4800:	serials_[config->port].uart.Init.BaudRate = 4800;   break;
    case    SERIAL_BAUDRATE_9600:	serials_[config->port].uart.Init.BaudRate = 9600;   break;
    case    SERIAL_BAUDRATE_19200:	serials_[config->port].uart.Init.BaudRate = 19200;   break;
    case    SERIAL_BAUDRATE_38400:	serials_[config->port].uart.Init.BaudRate = 38400;   break;
    case    SERIAL_BAUDRATE_57600:	serials_[config->port].uart.Init.BaudRate = 57600;   break;
    case    SERIAL_BAUDRATE_115200:	serials_[config->port].uart.Init.BaudRate = 115200;   break;
    default:	                    serials_[config->port].uart.Init.BaudRate = 115200;   break;
    }

    switch(config->parity)
    {
    case    SERIAL_PARITY_NONE: serials_[config->port].uart.Init.Parity = UART_PARITY_NONE;  break;
    case    SERIAL_PARITY_ODD:  serials_[config->port].uart.Init.Parity = UART_PARITY_ODD;  break;
    case    SERIAL_PARITY_EVEN: serials_[config->port].uart.Init.Parity = UART_PARITY_EVEN;  break;
    default:                    serials_[config->port].uart.Init.Parity = UART_PARITY_NONE;  break;
    }

    switch(config->dataBits)
    {
    case    SERIAL_DATA_BITS_8: serials_[config->port].uart.Init.WordLength = UART_WORDLENGTH_8B;    break;
	case    SERIAL_DATA_BITS_9: serials_[config->port].uart.Init.WordLength = UART_WORDLENGTH_9B;    break;
    default:                    serials_[config->port].uart.Init.WordLength = UART_WORDLENGTH_8B;    break;
    }

    switch(config->stopBits)
    {
    case    SERIAL_STOP_BITS_1: serials_[config->port].uart.Init.StopBits = UART_STOPBITS_1;    break;
    case    SERIAL_STOP_BITS_2: serials_[config->port].uart.Init.StopBits = UART_STOPBITS_2;    break;
    default:                    serials_[config->port].uart.Init.StopBits = UART_STOPBITS_1;    break;
    }

    serials_[config->port].uart.Init.Mode = UART_MODE_TX_RX;
    serials_[config->port].uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    serials_[config->port].uart.Init.OverSampling = UART_OVERSAMPLING_16;

    serials_[config->port].semaphore = xSemaphoreCreateMutex();
	if (serials_[config->port].semaphore == NULL)
    {
        ret = RET_ERROR;
        goto finished;
    }

    if (bufferLength == 0)
    {
        bufferLength = SERIAL_RX_BUFFER_SIZE;
    }
    
    serials_[config->port].rxQueue = xQueueCreate( bufferLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
    
    if (HAL_UART_Init(&serials_[config->port].uart) != HAL_OK)
    {
        ret = RET_ERROR;
        goto finished;
    }

    //serials_[config->port].rxBuffer = pvPortMalloc(bufferLength);
    HAL_UART_Receive_IT(&serials_[config->port].uart, serials_[config->port].buffer, sizeof(serials_[config->port].buffer));    
    
    switch(config->port)
    {
#ifdef  USART1
    case    SERIAL_PORT_1:    serials_[config->port].irq = USART1_IRQn; break;
#endif
#ifdef  USART2
    case    SERIAL_PORT_2:    serials_[config->port].irq = USART2_IRQn; break;
#endif
#ifdef  USART3
    case    SERIAL_PORT_3:    serials_[config->port].irq = USART3_IRQn; break;
#endif
#ifdef  UART4
    case    SERIAL_PORT_4:    serials_[config->port].irq = UART4_IRQn; break;
#endif
#ifdef  UART5
    case    SERIAL_PORT_5:    serials_[config->port].irq = UART5_IRQn; break;
#endif
    default:
        ret = RET_ERROR;
        goto finished;
    }
    
    HAL_NVIC_SetPriority(serials_[config->port].irq, config->priority, 0);
    HAL_NVIC_EnableIRQ(serials_[config->port].irq );
    
    *serialHandle = &serials_[config->port];
    
finished:
    if (ret != RET_OK)
    {
        if (serials_[config->port].semaphore != NULL)
        {
            vSemaphoreDelete(serials_[config->port].semaphore);
            serials_[config->port].semaphore = NULL;            
        }
        
        if (serials_[config->port].rxQueue != NULL)
        {
            vQueueDelete(serials_[config->port].rxQueue);
            serials_[config->port].rxQueue = NULL;
        }

        if (serials_[config->port].rxBuffer != NULL)
        {
            vPortFree(serials_[config->port].rxBuffer);
            serials_[config->port].rxBuffer= NULL;
        }
    }
    
    return  ret;
}

/*-----------------------------------------------------------*/

RET_VALUE       SERIAL_close(SERIAL_HANDLE serialHandle)
{
    ASSERT(serialHandle != NULL);

    HAL_NVIC_DisableIRQ(serialHandle->irq);

    if (HAL_UART_DeInit(&serialHandle->uart) != HAL_OK)
    {
        return  RET_ERROR;
    }
    
	if (serialHandle->semaphore != NULL)
    {
        vSemaphoreDelete(serialHandle->semaphore);
        serialHandle->semaphore = NULL;
    }
    
    if (serialHandle->rxQueue != NULL)
    {
        vQueueDelete(serialHandle->rxQueue);
        serialHandle->rxQueue = NULL;
    }
    
    if (serialHandle->rxBuffer != NULL)
    {
        vPortFree(serialHandle->rxBuffer);
        serialHandle->rxBuffer= NULL;
    }

    return  RET_OK;        
}

uint32_t SERIAL_gets(SERIAL_HANDLE serialHandle, uint8_t* buffer, uint32_t bufferSize, uint32_t timeout)
{
    ASSERT(serialHandle != NULL);
    uint32_t    length = 0;
    uint8_t     readByte;
    
    TickType_t  xStartTick = xTaskGetTickCount();
    
    while(length < bufferSize)
    {
        if (SERIAL_getc(serialHandle, &readByte, 1) == RET_OK)
        {
           buffer[length++] = readByte;
        }
        else
        {
            osDelay(0);
        }

        if (xTaskGetTickCount() - xStartTick > timeout)
        {
            break;
        }
    }
    
    /*if (serialHandle->outputEnable)
    {
            serialHandle->outputEnable(true);
    }*/

    return  length;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
RET_VALUE   SERIAL_puts(SERIAL_HANDLE serialHandle, const uint8_t* const buffer, uint32_t length, uint32_t timeout)
{
    ASSERT(serialHandle != NULL);

    RET_VALUE   ret = RET_OK;
    
    if( xSemaphoreTake( serialHandle->semaphore, timeout) != pdPASS )
    {
        ret = RET_ERROR;
    }
    else
    {        
        if (serialHandle->outputEnable)
        {
            serialHandle->outputEnable(true);
        }
        
        if (HAL_UART_Transmit(&serialHandle->uart, (uint8_t *)buffer, length, timeout) != HAL_OK)
        {
            ret = RET_ERROR;
        }

        if (serialHandle->outputEnable)
        {
            serialHandle->outputEnable(false);
            //osDelay(100);
        }
        
        xSemaphoreGive( serialHandle->semaphore);
    }
    
    return  ret;
}

/*-----------------------------------------------------------*/

uint8_t SERIAL_getc(SERIAL_HANDLE serialHandle, uint8_t *value, uint32_t timeout)
{ 
    if (xQueueReceive(serialHandle->rxQueue, value, timeout))
    {
        return  RET_OK;
    }
    else
    {
        return RET_ERROR;
    }
}

/*-----------------------------------------------------------*/

uint8_t SERIAL_putc(SERIAL_HANDLE serialHandle, uint8_t value, uint32_t timeout)
{
    ASSERT(serialHandle != NULL);

    RET_VALUE   ret = RET_OK;
    
    if( xSemaphoreTake( serialHandle->semaphore, timeout) != pdPASS )
    {
        ret = RET_ERROR;
    }
    else
    {        
        if (serialHandle->outputEnable)
        {
            serialHandle->outputEnable(true);
        }
        
         if (HAL_UART_Transmit(&serialHandle->uart, &value, 1, timeout) != HAL_OK)
        {
            ret = RET_ERROR;
        }
        
        if (serialHandle->outputEnable)
        {
            serialHandle->outputEnable(false);
        }
        xSemaphoreGive( serialHandle->semaphore);
    }
    
    return  ret;
}

/*-----------------------------------------------------------*/

RET_VALUE    SERIAL_printf
(
    SERIAL_HANDLE   serialHandle,
    const char *format, 
    ... 
)
{
   va_list  ap;
   uint32_t length = 0;
   static uint8_t  buffer[128];
   
#if 0
    TIME_STRUCT time;
    uint8_t     timeBuffer[64];
   
    _time_get(&time);
    FTE_TIME_toStr(&time, timeBuffer, sizeof(timeBuffer));   
    length = sprintf(&buffer[length], "[%s] ", timeBuffer);
#endif   
    va_start(ap, format);
    vsnprintf((char *)&buffer[length], sizeof(buffer) - length,  (char *)format, ap );
    va_end(ap);

    SERIAL_puts(serialHandle, buffer, strlen((char *)buffer), HAL_MAX_DELAY );

    return  RET_OK;
}

char* SERIAL_baudrateName(SERIAL_BAUDRATE baudrate)
{
    switch(baudrate)
    {
    case    SERIAL_BAUDRATE_50:		return  "50";
    case    SERIAL_BAUDRATE_75:	    return  "75";
    case    SERIAL_BAUDRATE_110:	return  "110";
    case    SERIAL_BAUDRATE_134:	return  "134";
    case    SERIAL_BAUDRATE_150:	return  "150";
    case    SERIAL_BAUDRATE_200:	return  "200";
    case    SERIAL_BAUDRATE_300:	return  "300";
    case    SERIAL_BAUDRATE_600:	return  "600";
    case    SERIAL_BAUDRATE_1200:	return  "1200";
    case    SERIAL_BAUDRATE_1800:	return  "1800";
    case    SERIAL_BAUDRATE_2400:	return  "2400";
    case    SERIAL_BAUDRATE_4800:	return  "4800";
    case    SERIAL_BAUDRATE_9600:	return  "9600";
    case    SERIAL_BAUDRATE_19200:	return  "19200";
    case    SERIAL_BAUDRATE_38400:	return  "38400";
    case    SERIAL_BAUDRATE_57600:	return  "57600";
    case    SERIAL_BAUDRATE_115200:	return  "115200";
    default:	                    return  "115200";
    }
}

char* SERIAL_dataBitsName(SERIAL_DATA_BITS dataBits)
{
    switch(dataBits)
    {
    case    SERIAL_DATA_BITS_8: return  "8";
	case    SERIAL_DATA_BITS_9: return  "9";
    default:                    return  "8";
    }
}

char* SERIAL_stopBitsName(SERIAL_STOP_BITS stopBits)
{

    switch(stopBits)
    {
    case    SERIAL_STOP_BITS_1: return  "1";
    case    SERIAL_STOP_BITS_2: return  "2";
    default:                    return  "1";
    }
}

static char*    parityNames_[] = 
{
	"NONE", 
	"ODD", 
	"EVEN", 
	"MARK", 
	"SPACE"    
};

char*   SERIAL_parityName(SERIAL_PARITY parity)
{
    if (parity < SERIAL_PARITY_MAX)
    {
        return  parityNames_[parity];
    }
    
    return  "unknown";
}

void USARTx_IRQHandler(SERIAL *serial)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    HAL_UART_IRQHandler(&serial->uart);
    
    if (serial->uart.RxXferSize != serial->uart.RxXferCount)
    {

        serial->uart.pRxBuffPtr--;
        serial->uart.RxXferCount++;
        xQueueSendFromISR(serial->rxQueue, serial->uart.pRxBuffPtr, &xHigherPriorityTaskWoken);
    }
    
    if( xHigherPriorityTaskWoken )
	{
	//	portYIELD();
	}

}

void USART1_IRQHandler(void)
{
    USARTx_IRQHandler(&serials_[SERIAL_PORT_1]);
}

void USART2_IRQHandler(void)
{
    USARTx_IRQHandler(&serials_[SERIAL_PORT_2]);
}

void USART3_IRQHandler(void)
{
    USARTx_IRQHandler(&serials_[SERIAL_PORT_3]);
}

void UART4_IRQHandler(void)
{
    USARTx_IRQHandler(&serials_[SERIAL_PORT_4]);
}

void UART5_IRQHandler(void)
{
    USARTx_IRQHandler(&serials_[SERIAL_PORT_5]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    assert_param(huart != NULL);
    
    huart->pRxBuffPtr -= huart->RxXferSize;
    huart->RxXferCount = huart->RxXferSize;
}

void    HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}