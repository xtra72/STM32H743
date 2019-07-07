#include "target.h"
#include "spi.h"
#include "time2.h"

#define __MODULE_NAME__ "RF"
#include "trace.h"

static  SPI_HandleTypeDef*  spi_ = NULL;
static  SemaphoreHandle_t   ioSemaphore_ = NULL;

RET_VALUE   SPI_init(SPI_HandleTypeDef* _spi)
{
    spi_ = _spi;

    ioSemaphore_ = xSemaphoreCreateBinary();

    return  RET_OK;
}


RET_VALUE   SPI_receive(uint8_t* _buffer, uint32_t _size, uint32_t _timeout)
{
    ASSERT(spi_ != NULL);
    uint32_t    startTime = TICK_get();

    if (HAL_SPI_Receive_IT(spi_, _buffer, _size) == HAL_OK);
    {
        if (xSemaphoreTake( ioSemaphore_, TICK_remainTime(startTime, _timeout))  == pdTRUE)
        {
            return  RET_OK;
        }
    }

    return  RET_ERROR;
}

RET_VALUE   SPI_transmit(uint8_t* _buffer, uint32_t _size, uint32_t _timeout)
{
    ASSERT(spi_ != NULL);
    uint32_t    startTime = TICK_get();

    if (HAL_SPI_Transmit_IT(spi_, _buffer, _size) == HAL_OK)
    {
        if (xSemaphoreTake( ioSemaphore_, TICK_remainTime(startTime, _timeout))  == pdTRUE)
        {
            return  RET_OK;
        }
    }

    return  RET_ERROR;
}

RET_VALUE   SPI_transmitReceive(uint8_t* _txBuffer, uint8_t* _rxBuffer, uint32_t _size, uint32_t _timeout)
{
    ASSERT(spi_ != NULL);

    RET_VALUE   ret = RET_ERROR;
    uint32_t    startTime = TICK_get();
    _timeout = TICK_remainTime(startTime, _timeout);

    if (HAL_SPI_TransmitReceive_IT(spi_, _txBuffer, _rxBuffer, _size) == HAL_OK)
    {
        if (xSemaphoreTake( ioSemaphore_, _timeout)  == pdTRUE)
        {
            ret = RET_OK;
        }
    }

    if (ret != RET_OK)
    {
        DEBUG("SPI Transmit failed : %d, %d, %d\n", startTime, TICK_get(), _timeout);
    }

    return  ret;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    static BaseType_t xHigherPriorityTaskWoken;

    xSemaphoreGiveFromISR(ioSemaphore_, &xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        // We can force a context switch here.  Context switching from an
        // ISR uses port specific syntax.  Check the demo task for your port
        // to find the syntax required.
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    static BaseType_t xHigherPriorityTaskWoken;

    xSemaphoreGiveFromISR(ioSemaphore_, &xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        // We can force a context switch here.  Context switching from an
        // ISR uses port specific syntax.  Check the demo task for your port
        // to find the syntax required.
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    static BaseType_t xHigherPriorityTaskWoken;

    xSemaphoreGiveFromISR(ioSemaphore_, &xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        // We can force a context switch here.  Context switching from an
        // ISR uses port specific syntax.  Check the demo task for your port
        // to find the syntax required.
    }

}

