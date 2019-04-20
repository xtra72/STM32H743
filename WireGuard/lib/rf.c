#include "target.h"
#include "sx1231.h"
#include "rf.h"
#include "trace.h"

#define TRACE(...)  TRACE_printf("RF", __VA_ARGS__)

typedef enum
{
    RF_CMD_STOP
}   RF_CMD;

typedef struct
{
    RF_CMD  cmd;
    bool    inProgress;
    bool    invalid;
}   RF_MESSAGE;

typedef struct
{
    RF_CMD  cmd;
    bool    inProgress;
    bool    invalid;
    uint8_t startAddress;

}   RF_MESSAGE_READ_REG;

static  void RF_taskMain(void const * argument);

static  SPI_HandleTypeDef*  spi_ = NULL;

static  uint32_t            bitrate_ = 300000;
static   QueueHandle_t      messageQueueHandle_;
static  osThreadId          threadId_ = NULL;
static  bool                stop_ = true;
static  SemaphoreHandle_t   semaphore_ = NULL;

RET_VALUE   RF_init(SPI_HandleTypeDef* spi)
{
    messageQueueHandle_ = xQueueCreate( 16, sizeof( void* ) );
    semaphore_ = xSemaphoreCreateMutex();
    spi_ = spi;
    SX1231_init();

    return  RET_OK;
}

RET_VALUE   RF_setConfig(RF_CONFIG* config)
{
    ASSERT(config);

    bitrate_ = config->bitrate;

    return  RET_OK;
}

RET_VALUE   RF_getConfig(RF_CONFIG* config)
{
    ASSERT(config);

    config->bitrate = bitrate_;

    return  RET_OK;
}

RET_VALUE   RF_start(void)
{
    RET_VALUE   ret = RET_OK;

    if (threadId_ == NULL)
    {
        osThreadDef(rfTask, RF_taskMain, osPriorityIdle, 0, 512);
        threadId_ = osThreadCreate(osThread(rfTask), NULL);
        if (threadId_ == NULL)
        {
            ret = RET_ERROR;
        }
    }

    return  ret;
}

RET_VALUE   RF_stop(void)
{
    RET_VALUE   ret = RET_OK;

    RF_MESSAGE* message = pvPortMalloc(sizeof(RF_MESSAGE));
    if (message != NULL)
    {
        message->cmd = RF_CMD_STOP;

        while(!stop_)
        {
            osDelay(1);
        }

        osThreadTerminate(threadId_);
        threadId_ = NULL;
    }
    else
    {
        ret = RET_NOT_ENOUGH_MEMORY;
    }

    return  ret;
}

void RF_taskMain(void const * argument)
{
    RF_MESSAGE*  message;

    TRACE("RF task start!\n");

    RF_reset();

    SX1231_initRFChip();

    stop_ = false;

    while(!stop_)
    {
        if( xQueueReceive( messageQueueHandle_, &(message), ( TickType_t ) 10 ) )
		{
            message->inProgress = true;
            if (message->cmd == RF_CMD_STOP)
            {
                vPortFree(message);
                break;
            }
            else
            {
                switch(message->cmd)
                {
                case    RF_CMD_READ_REG:
                    {

                    }
                }
                vPortFree(message);
            }
        }
    }

    while( xQueueReceive( messageQueueHandle_, &(message), ( TickType_t ) 0 ) )
    {
        vPortFree(message);
    }

    stop_ = true;

}


typedef struct
{
    uint32_t    bitrate;
    uint8_t     msb;
    uint8_t     lsb;
}   RF_BIT_RATE;

RF_BIT_RATE bitrateTables [] =
{
    {   1200,   SX1231_RF_BITRATELSB_1200,  SX1231_RF_BITRATELSB_1200 },
    {   2400,   SX1231_RF_BITRATEMSB_2400,  SX1231_RF_BITRATELSB_2400 },
    {   4800,   SX1231_RF_BITRATEMSB_4800,  SX1231_RF_BITRATELSB_4800 },
    {   9600,   SX1231_RF_BITRATEMSB_9600,  SX1231_RF_BITRATELSB_9600 },
    {   12500,  SX1231_RF_BITRATEMSB_12500, SX1231_RF_BITRATELSB_12500 },
    {   19200,  SX1231_RF_BITRATEMSB_19200, SX1231_RF_BITRATELSB_19200 },
    {   25000,  SX1231_RF_BITRATEMSB_25000, SX1231_RF_BITRATELSB_25000 },
    {   32768,  SX1231_RF_BITRATEMSB_32768, SX1231_RF_BITRATELSB_32768 },
    {   38400,  SX1231_RF_BITRATEMSB_38400, SX1231_RF_BITRATELSB_38400 },
    {   50000,  SX1231_RF_BITRATEMSB_50000, SX1231_RF_BITRATELSB_50000 },
    {   57600,  SX1231_RF_BITRATEMSB_57600, SX1231_RF_BITRATELSB_57600 },
    {   76800,  SX1231_RF_BITRATEMSB_76800, SX1231_RF_BITRATELSB_76800 },
    {   100000, SX1231_RF_BITRATEMSB_100000,SX1231_RF_BITRATELSB_100000 },
    {   115200, SX1231_RF_BITRATEMSB_115200,SX1231_RF_BITRATELSB_115200 },
    {   150000, SX1231_RF_BITRATEMSB_150000,SX1231_RF_BITRATELSB_150000 },
    {   153600, SX1231_RF_BITRATEMSB_153600,SX1231_RF_BITRATELSB_153600 },
    {   200000, SX1231_RF_BITRATEMSB_200000,SX1231_RF_BITRATELSB_200000 },
    {   250000, SX1231_RF_BITRATEMSB_250000,SX1231_RF_BITRATELSB_250000 },
    {   300000, SX1231_RF_BITRATEMSB_300000,SX1231_RF_BITRATELSB_300000 },
    {   0,      0,                          0}
};

RET_VALUE   RF_setBitrate(uint32_t bitrate)
{
    for(int i = 1 ; bitrateTables[i].bitrate != 0; i++)
    {
        if (bitrate < bitrateTables[i].bitrate)
        {
            SX1231_writeRegister(SX1231_REG_BITRATEMSB, bitrateTables[i-1].msb);
            SX1231_writeRegister(SX1231_REG_BITRATELSB, bitrateTables[i-1].lsb);
            bitrate_ = bitrateTables[i].bitrate;

            return  RET_OK;
        }
    }

    return  RET_OUT_OF_RANGE;
}

uint32_t    RF_getBitrate(void)
{
    return  bitrate_;
}

uint8_t RF_readRegister(uint8_t    address)
{
    return  value = 0;

    if( xSemaphoreTake( semaphore_, ( TickType_t ) 10 ) == pdTRUE )
    {
        value = SX1231_readRegister(address);

        xSemaphoreGive( semaphore_ );
    }

    return  value;
}

void    RF_writeRegister(uint8_t    address, uint8_t value)
{
    if( xSemaphoreTake( semaphore_, ( TickType_t ) 10 ) == pdTRUE )
    {
        SX1231_writeRegister(address, value);

        xSemaphoreGive( semaphore_ );
    }
}

void    RF_reset(void)
{
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(RF_RESET_GPIO_Port, RF_RESET_Pin, GPIO_PIN_RESET);
    osDelay(5);
}

void    RF_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    uint8_t ret;

    if( xSemaphoreTake( semaphore_, ( TickType_t ) 10 ) == pdTRUE )
    {
        SX1231_sendRfFrame(buffer, size, &ret);

        xSemaphoreGive( semaphore_ );
    }
}


void   SX1231_SPI_select(bool   select)
{
    if (select)
    {
        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
    }
}

void    SX1231_SPI_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        HAL_SPI_Transmit(spi_, buffer, size, timeout);
    }
}

void    SX1231_SPI_receive(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        HAL_SPI_Receive(spi_, buffer, size, timeout);
    }
}
