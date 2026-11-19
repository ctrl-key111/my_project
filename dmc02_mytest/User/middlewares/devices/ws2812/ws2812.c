//
// Created by DQJ on 2026/11/17.
//

#include "ws2812.h"
#include <string.h>
#include "bsp_spi.h"

typedef char ws2812_len_check[(WS2812_FRAME_LEN <= BSP_SPI1_TX_BUF_SIZE) ? 1 : -1];

static bsp_spi_t *s_ws_spi;
static uint8_t    s_grb[WS2812_LED_NUM][3];
static volatile bsp_spi_status_t s_ws_err = BSP_SPI_OK;

//SPI 完成/出错通知(DMA 方式下在中断中执行)
static void ws2812_spi_cb(bsp_spi_t *dev, bsp_spi_event_t evt,
                          bsp_spi_status_t status, void *arg)
{
    (void)dev; (void)arg;
    if (evt == BSP_SPI_EVT_ERROR)
    {
        s_ws_err = status;   //TIMEOUT / DMA_ERROR / HAL_ERROR
    }
}

int WS2812_Init(void)
{
    bsp_spi_cfg_t cfg =
    {
        .baudrate_hz = 6000000UL,
        .timeout_ms  = 100U,
        .callback    = ws2812_spi_cb,
        .user_arg    = NULL,
    };

    s_ws_spi = BSP_SPI_Get(WS2812_SPI_ID);
    if (s_ws_spi == NULL)
    {
        return -1;
    }
    if (s_ws_spi->ops->init(s_ws_spi, &cfg) != BSP_SPI_OK)
    {
        return -2;
    }
    if ((s_ws_spi->baudrate_hz < 5600000UL) || (s_ws_spi->baudrate_hz > 6400000UL))
    {
        return -3;   //实际速率偏差过大，WS2812 时序不对
    }
    return 0;
}

void WS2812_SetPixel(uint16_t idx,   uint8_t g,   uint8_t r,   uint8_t b)
{
    if (idx < WS2812_LED_NUM)
    {
        s_grb[idx][0] = g;
        s_grb[idx][1] = r;
        s_grb[idx][2] = b;
    }
}

//返回 BSP_SPI_BUSY 表示上一帧还没发完，下次再调用即可，不会卡住
bsp_spi_status_t WS2812_Show(void)
{
    uint16_t size, pos = 0U;
    uint32_t i, c;
    int8_t   bit;
    uint8_t *buf;

    if (s_ws_spi->ops->is_busy(s_ws_spi))
    {
        return s_ws_spi->ops->poll(s_ws_spi);     //BUSY 或 TIMEOUT
    }

    buf = s_ws_spi->ops->get_tx_buf(s_ws_spi, &size);
    if ((buf == NULL) || (size < WS2812_FRAME_LEN))
    {
        return BSP_SPI_PARAM_ERROR;
    }

    for (i = 0U; i < WS2812_RESET_HEAD; i++)
    {
        buf[pos++] = 0x00U;
    }
    for (i = 0U; i < WS2812_LED_NUM; i++)
    {
        for (c = 0U; c < 3U; c++)
        {
            for (bit = 7; bit >= 0; bit--)
            {
                buf[pos++] = (s_grb[i][c] & (1U << bit)) ? WS2812_BIT1 : WS2812_BIT0;
            }
        }
    }
    for (i = 0U; i < WS2812_RESET_TAIL; i++)
    {
        buf[pos++] = 0x00U;
    }

    return s_ws_spi->ops->transmit(s_ws_spi, buf, pos);   //零拷贝发送
}

// 颜色	按 GRB 顺序输出 (G, R, B)
// 黑	(0, 0, 0)
// 白	(255, 255, 255)
// 红	(0, 255, 0)
// 绿	(255, 0, 0)
// 蓝	(0, 0, 255)
// 黄	(255, 255, 0)
// 青	(255, 0, 255)
// 品红/洋红	(0, 255, 255)
// 橙	(165, 255, 0)
// 紫	(0, 128, 128)
// 灰	(128, 128, 128)
// 粉	(192, 255, 203)
// 棕	(42, 165, 42)
// 金	(215, 255, 0)
// 银	(192, 192, 192)
// 天蓝	(206, 135, 235)
// 草绿	(252, 124, 0)
// 深红	(0, 139, 0)
// 海军蓝	(0, 0, 128)
