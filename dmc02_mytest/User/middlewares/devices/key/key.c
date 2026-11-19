//
// Created by DQJ on 2026/11/18.
//
#include "stm32h7xx_hal.h"
#include "key.h"
#include "stm32h7xx_hal_gpio.h"

volatile uint8_t State = 0;
void key_set(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
    {
    HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
        {
            State = !State;
        }
    }
while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET);

}
