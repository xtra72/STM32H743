#include "target.h"
#include "sx1231.h"
#include "rf.h"
#include "trace.h"

#define TRACE(...)  TRACE_printf("RF", __VA_ARGS__)

typedef enum
{
    RF_CMD_STOP,
    RF_CMD_READ_REG,
    RF_CMD_TEST_SEND,
    RF_CMD_TEST_RECV,
    RF_CMD_TEST_STOP,
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

typedef struct
{
    uint8_t payload[256];
    uint32_t size;
    uint32_t interval;
    uint32_t count;
    uint32_t countMax;
}   RF_MESSAGE_TEST_SEND_PARAMS;

typedef struct
{
    RF_CMD  cmd;
    bool    inProgress;
    bool    invalid;
    RF_MESSAGE_TEST_SEND_PARAMS params;
}   RF_MESSAGE_TEST_SEND;

typedef struct
{
    uint8_t payload[256];
    uint32_t receivedLength;
    uint32_t interval;
    uint32_t timeout;
    uint32_t count;
    uint32_t countMax;
}   RF_MESSAGE_TEST_RECV_PARAMS;

typedef struct
{
    RF_CMD  cmd;
    bool    inProgress;
    bool    invalid;
    RF_MESSAGE_TEST_RECV_PARAMS params;
}   RF_MESSAGE_TEST_RECV;

static  void RF_taskMain(void const * argument);
static  RET_VALUE   RF_internalTestSendStart(uint8_t* buffer, uint32_t size, uint32_t interval, uint32_t count);
static  RET_VALUE   RF_internalTestRecvStart(uint32_t interval);
static  void        RF_internalTestSendCallback(void const * argument);
static  void        RF_internalTestRecvCallback(void const * argument);
static  RET_VALUE   RF_internalTestStop(void);

static  SPI_HandleTypeDef*  spi_ = NULL;

static  uint32_t            bitrate_ = 300000;
static   QueueHandle_t      messageQueueHandle_;
static  osThreadId          threadId_ = NULL;
static  bool                stop_ = true;
static  SemaphoreHandle_t   semaphore_ = NULL;
static  RF_STATUS           status_ = RF_STATUS_STOPPED;
static  osTimerId           testSendHandle = NULL;
static  osTimerId           testRecvHandle = NULL;
static  RF_MESSAGE_TEST_SEND_PARAMS testSendParams_ =
{
    .size = 0,
    .interval = 0,
    .count = 0
};
static  RF_MESSAGE_TEST_RECV_PARAMS testRecvParams_ =
{
    .interval = 0,
    .count = 0
};

static  const char*   statusStrings_[] =
{
    "Stopped",      //RF_STATUS_STOPPED
    "Ready",        //RF_STATUS_READY,
    "Test Send",    //RF_STATUS_TEST_SEND,
    "Test Recv",    //RF_STATUS_TEST_RECV,
};

RET_VALUE   RF_init(SPI_HandleTypeDef* spi)
{
    messageQueueHandle_ = xQueueCreate( 16, sizeof( void* ) );
    semaphore_ = xSemaphoreCreateMutex();
    spi_ = spi;

    osTimerDef(timerTestSend, RF_internalTestSendCallback);
    testSendHandle = osTimerCreate(osTimer(timerTestSend), osTimerPeriodic, &testSendParams_);

    osTimerDef(timerTestRecv, RF_internalTestRecvCallback);
    testRecvHandle = osTimerCreate(osTimer(timerTestRecv), osTimerPeriodic, &testSendParams_);

    SX1231_init();

    return  RET_OK;
}

RF_STATUS   RF_getStatus(void)
{
    return  status_;
}

const char*       RF_getStatusString(RF_STATUS status)
{
    if (status < RF_STATUS_MAX)
    {
        return  statusStrings_[status];
    }

    return  "Unknown";
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

    status_ = RF_STATUS_READY;
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

            switch(message->cmd)
            {
            case    RF_CMD_READ_REG:
                {

                }
                break;

            case    RF_CMD_TEST_SEND:
                {
                    RF_MESSAGE_TEST_SEND*   messageTestSend =(RF_MESSAGE_TEST_SEND*)message;

                    RF_internalTestSendStart(messageTestSend->params.payload, messageTestSend->params.size, messageTestSend->params.interval, messageTestSend->params.count);
                }
                break;

            case    RF_CMD_TEST_RECV:
                {
                    RF_MESSAGE_TEST_RECV*   messageTestRecv =(RF_MESSAGE_TEST_RECV*)message;

                    RF_internalTestRecvStart(messageTestRecv->params.interval);
                }
                break;

            case    RF_CMD_TEST_STOP:
                {
                    RF_internalTestStop();
                }
                break;
            }
            vPortFree(message);
        }
    }

    while( xQueueReceive( messageQueueHandle_, &(message), ( TickType_t ) 0 ) )
    {
        vPortFree(message);
    }

    stop_ = true;
    status_ = RF_STATUS_STOPPED;

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
    uint8_t value = 0;

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

RET_VALUE   RF_send(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    uint8_t ret = ERROR;

    if( xSemaphoreTake( semaphore_, ( TickType_t ) timeout / portTICK_PERIOD_MS) == pdTRUE )
    {
        TRACE("RF send!\n");
        ret = SX1231_sendRfFrame(buffer, size, timeout);

        xSemaphoreGive( semaphore_ );
    }

    if (ret != OK)
    {
        TRACE("RF send failed!\n");
        return  RET_ERROR;
    }

    TRACE("RF send success!\n");
    return  RET_OK;
}

RET_VALUE   RF_recv(uint8_t* buffer, uint32_t bufferSize, uint32_t* receivedLength, uint32_t timeout)
{
    uint8_t ret = ERROR;

    if( xSemaphoreTake( semaphore_, ( TickType_t ) timeout / portTICK_PERIOD_MS) == pdTRUE )
    {
        uint32_t length;

        TRACE("RF receive!\n");
        ret  = SX1231_receiveRfFrame(buffer, bufferSize, &length);

        *receivedLength = length;

        xSemaphoreGive( semaphore_ );
    }

    if (ret != OK)
    {
        TRACE("RF send failed!\n");
        return  RET_ERROR;
    }

    TRACE("RF send success!\n");
    return  RET_OK;
}

RET_VALUE   RF_testSend(uint8_t* buffer, uint32_t size, uint32_t interval, uint32_t count)
{
    RF_MESSAGE_TEST_SEND*   message = pvPortMalloc(sizeof(RF_MESSAGE_TEST_SEND));

    if (!message)
    {
        return  RET_NOT_ENOUGH_MEMORY;
    }

    message->cmd = RF_CMD_TEST_SEND;
    memcpy(message->params.payload, buffer, size);
    message->params.size = size;
    message->params.interval = interval;
    message->params.count = 0;
    message->params.countMax = count;

    if (xQueueSend( messageQueueHandle_, &message, ( TickType_t ) 10 ) == pdPASS)
    {
        return  RET_OK;
    }

    vPortFree(message);

    return  RET_ERROR;
}

RET_VALUE   RF_testRecv(uint32_t interval, uint32_t timeout)
{
    RF_MESSAGE_TEST_RECV*   message = pvPortMalloc(sizeof(RF_MESSAGE_TEST_RECV));

    if (!message)
    {
        return  RET_NOT_ENOUGH_MEMORY;
    }

    message->cmd = RF_CMD_TEST_RECV;
    message->params.interval = interval;
    message->params.timeout = timeout;

    if (xQueueSend( messageQueueHandle_, &message, ( TickType_t ) 10 ) == pdPASS)
    {
        return  RET_OK;
    }

    vPortFree(message);

    return  RET_ERROR;
}

RET_VALUE   RF_testStop()
{
    RF_MESSAGE*   message = pvPortMalloc(sizeof(RF_MESSAGE));

    if (!message)
    {
        return  RET_NOT_ENOUGH_MEMORY;
    }

    message->cmd = RF_CMD_TEST_STOP;

    if (xQueueSend( messageQueueHandle_, &message, ( TickType_t ) 10 ) == pdPASS)
    {
        return  RET_OK;
    }

    vPortFree(message);

    return  RET_ERROR;
}

RET_VALUE   RF_internalTestSendStart(uint8_t* buffer, uint32_t size, uint32_t interval, uint32_t count)
{
    if ( status_ != RF_STATUS_READY)
    {
        return  RET_ERROR;
    }

    memcpy(testSendParams_.payload, buffer, size);
    testSendParams_.size = size;
    testSendParams_.interval = interval;
    testSendParams_.count = 0;
    testSendParams_.countMax = count;

    osTimerStart(testSendHandle, interval);

    status_ = RF_STATUS_TEST_SEND;

    return  RET_OK;
}


RET_VALUE   RF_internalTestRecvStart(uint32_t interval)
{
    if ( status_ != RF_STATUS_READY)
    {
        return  RET_ERROR;
    }

    testRecvParams_.interval = interval;

    osTimerStart(testRecvHandle, interval);

    status_ = RF_STATUS_TEST_RECV;

    return  RET_OK;
}

void RF_internalTestSendCallback(void const * argument)
{
//    RF_MESSAGE_TEST_SEND_PARAMS* params =(RF_MESSAGE_TEST_SEND_PARAMS*)argument;

    ++testSendParams_.count;

    if (testSendParams_.countMax != 0)
    {
        TRACE("Test Send : %d / %d\n", testSendParams_.count, testSendParams_.countMax);
    }
    else
    {
        TRACE("Test Send : %d\n", testSendParams_.count);
    }

    RF_send(testSendParams_.payload, testSendParams_.size, 10);

    if ((testSendParams_.countMax != 0) && (testSendParams_.count == testSendParams_.countMax))
    {
        osTimerStop(testSendHandle);
        status_ = RF_STATUS_READY;
    }
}


void RF_internalTestRecvCallback(void const * argument)
{
    RF_recv(testRecvParams_.payload, sizeof(testRecvParams_.payload), &testRecvParams_.receivedLength, testRecvParams_.timeout);
}

RET_VALUE   RF_internalTestStop(void)
{
    switch(status_)
    {
    case    RF_STATUS_TEST_SEND:
        {
            osTimerStop(testSendHandle);
            status_ = RF_STATUS_READY;
            TRACE("Send test stopped!\n");
        }
        break;

    case    RF_STATUS_TEST_RECV:
        {
            osTimerStop(testRecvHandle);
            status_ = RF_STATUS_READY;
            TRACE("Recv test stopped!\n");
        }
        break;
    }

    return  RET_OK;
}


void  RF_DIO0Callback()
{
    if (SX1231_getPreviousMode() == SX1231_RF_TRANSMITTER)
    {

    }
    else if (SX1231_getPreviousMode() == SX1231_RF_RECEIVER)
    {
        SX1231_receiveDoneCallback();
    }

}

/////////////////////////////////////////////////////////////////////////////
// SX1231 Control Functions
/////////////////////////////////////////////////////////////////////////////
void SX1231_wait(uint32_t usecs)
{
    uint32_t    msecs = (usecs + 999) / 1000;

    osDelay(msecs);
}

bool SX1231_getDIO0(void)
{
    return  HAL_GPIO_ReadPin(RF_DIO0_GPIO_Port, RF_DIO0_Pin) == GPIO_PIN_SET;
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

bool    SX1231_SPI_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        return  (HAL_SPI_Transmit(spi_, buffer, size, timeout) == HAL_OK);
    }

    return  false;
}

bool    SX1231_SPI_receive(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (spi_)
    {
        return  (HAL_SPI_Receive(spi_, buffer, size, timeout) == HAL_OK);
    }

    return  false;
}
