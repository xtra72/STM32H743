#include "target.h"
#include "gpio.h"


void    GPIO_MASTER_setStatus(bool _ready)
{
    if (_ready)
    {
        HAL_GPIO_WritePin(RF_MASTER_READY_GPIO_Port, RF_MASTER_READY_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(RF_MASTER_READY_GPIO_Port, RF_MASTER_READY_Pin, GPIO_PIN_SET);
    }
}

bool    GPIO_SLAVE_isReady(void)
{
    return  (HAL_GPIO_ReadPin(RF_SLAVE_READY_GPIO_Port, RF_SLAVE_READY_Pin) == GPIO_PIN_RESET);
}

