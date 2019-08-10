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


void    GPIO_AVDD_enable(void)
{
    HAL_GPIO_WritePin(AVDD_EN_GPIO_Port, AVDD_EN_Pin, GPIO_PIN_SET);
}


void    GPIO_AVDD_disable(void)
{
    HAL_GPIO_WritePin(AVDD_EN_GPIO_Port, AVDD_EN_Pin, GPIO_PIN_RESET);
}


bool    GPIO_AVDD_isEnable(void)
{
    return  HAL_GPIO_ReadPin(AVDD_EN_GPIO_Port, AVDD_EN_Pin) == GPIO_PIN_SET;
}