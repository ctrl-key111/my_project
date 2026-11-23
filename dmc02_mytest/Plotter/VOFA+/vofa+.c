//
// Created by DQJ on 2026/11/19.
//

#include <string.h>
#include "stm32h7xx_hal.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"
#include "vofa+.h"

static uint8_t   tx_buf[VOFA_BUF_SIZE];  // 生命周期 = 整个程序
static volatile uint8_t tx_busy = 0u;

void Vofa_JustFloat(const float *data, uint8_t num)
{
    static const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

    if (data == NULL || num == 0u)
    {
        return;
    }
    if (num > VOFA_JUSTFLOAT_MAX_CH)
    {
        num = VOFA_JUSTFLOAT_MAX_CH;
    }

    if (tx_busy)
    {
        return;
    }

    memcpy(tx_buf, data, sizeof(float) * num);
    memcpy(&tx_buf[num * 4u], tail, sizeof(tail));

    tx_busy = 1u;
    if (HAL_UART_Transmit_DMA(&huart4, tx_buf ,
                              (uint16_t)((num + 1u) * 4u)) != HAL_OK)
    {
        tx_busy = 0u;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        tx_busy = 0u;
    }
}