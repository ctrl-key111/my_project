//
// Created by DQJ on 2026/11/19.
//

#include "bsp_usart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include"stm32h7xx_hal.h"
#include "stm32h723xx.h"
#include"usart.h"
#include "stm32h7xx_it.h"


uint8_t RX_data;
char RX_buffer[20];
uint8_t RX_index = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {

        if (RX_data == '\n'|| RX_data == '\r' )
        {

            RX_buffer[RX_index] = '\0';
            if(RX_buffer[0] != '\0' &&((RX_buffer[0] >= '0' && RX_buffer[0] <= '9') ||RX_buffer[0] == '+' || RX_buffer[0] == '-' ))//以防垃圾字符
            {
                = atof(RX_buffer);


            }
            RX_index = 0;                // 清零
            memset(RX_buffer, 0, sizeof(RX_buffer));

        }
        else
        {

            if (RX_index < 19)
            {
                RX_buffer[RX_index++] = RX_data;
            }
        }

        HAL_UART_Receive_IT(&huart4, &RX_data, 1);
    }
}