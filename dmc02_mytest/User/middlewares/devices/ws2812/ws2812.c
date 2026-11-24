//
// Created by DQJ on 2026/11/17.
//

#include "ws2812.h"
#include <string.h>
#include "bsp_spi.h"

//================== 编译期检查 ==================
//一帧的长度必须放得下所选 SPI 的发送缓冲区，放不下直接编译报错
typedef char ws2812_frame_check[(WS2812_FRAME_LEN <= WS2812_SPI_TX_BUF_SIZE) ? 1 : -1];

static bsp_spi_t *s_ws_spi = NULL;
static uint8_t    s_grb[WS2812_LED_NUM][3];
static volatile bsp_spi_status_t s_ws_err = BSP_SPI_OK;

//SPI 完成/出错通知(DMA 方式下在中断中执行，只记录错误，不做别的事)
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
        .baudrate_hz = WS2812_SPI_BAUD,
        .timeout_ms  = 100U,
        .callback    = ws2812_spi_cb,
        .user_arg    = NULL,
    };

    //1. 取设备对象：编号越界返回 ERR_GET；该路 SPI 在 bsp_spi.h 里没使能时 handle 为 NULL
    s_ws_spi = BSP_SPI_Get(WS2812_SPI_ID);
    if (s_ws_spi == NULL)
    {
        return WS2812_INIT_ERR_GET;
    }
    if (s_ws_spi->handle == NULL)
    {
        s_ws_spi = NULL;
        return WS2812_INIT_ERR_SPI;      //bsp_spi.h 里把 BSP_SPIx_ENABLE 打开
    }

    //2. 先确认缓冲区放得下一帧
    if (s_ws_spi->tx_buf_size < WS2812_FRAME_LEN)
    {
        s_ws_spi = NULL;
        return WS2812_INIT_ERR_BUF;
    }

    //3. 交给封装层初始化速率 / 超时 / 回调，并检查时钟、DMA、SRAM1 段
    if (BSP_SPI_Init(s_ws_spi, &cfg) != BSP_SPI_OK)
    {
        s_ws_spi = NULL;
        return WS2812_INIT_ERR_SPI;
    }

    //4. 实际分频后的速率偏离 6MHz 太多，WS2812 时序就不对了
    if ((s_ws_spi->baudrate_hz < WS2812_BAUD_MIN) ||
        (s_ws_spi->baudrate_hz > WS2812_BAUD_MAX))
    {
        return WS2812_INIT_ERR_BAUD;
    }

    s_ws_err = BSP_SPI_OK;
    WS2812_Clear();          //上电全灭
    return WS2812_INIT_OK;
}

void WS2812_SetPixel(uint16_t idx, uint8_t g, uint8_t r, uint8_t b)
{
    if (idx < WS2812_LED_NUM)
    {
        s_grb[idx][0] = g;
        s_grb[idx][1] = r;
        s_grb[idx][2] = b;
    }
}

void WS2812_Clear(void)
{
    memset(s_grb, 0, sizeof(s_grb));
}

//返回 BSP_SPI_BUSY 表示上一帧还没发完，下次再调用即可，不会卡住
bsp_spi_status_t WS2812_Show(void)
{
    uint16_t size = 0U, pos = 0U;
    uint32_t i, c;
    int8_t   bit;
    uint8_t *buf;
    bsp_spi_status_t st;

    if (s_ws_spi == NULL)                     //没调 WS2812_Init，或者初始化失败
    {
        return BSP_SPI_PARAM_ERROR;
    }

    //1. 上一帧还在发：顺带做一次超时检测，返回 BUSY 或 TIMEOUT
    if (BSP_SPI_IsBusy(s_ws_spi))
    {
        return BSP_SPI_Poll(s_ws_spi);
    }

    //2. 零拷贝：直接拿封装层的发送缓冲区来编码，省一次 memcpy
    buf = BSP_SPI_GetTxBuf(s_ws_spi, &size);
    if ((buf == NULL) || (size < WS2812_FRAME_LEN))
    {
        return BSP_SPI_PARAM_ERROR;
    }

    //3. 编码：帧头低电平 -> 每颗灯 GRB 共 24 个 bit -> 帧尾复位低电平
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

    //4. 零拷贝发送：data 就是 tx_buf，封装层内部不会再拷一遍
    st = BSP_SPI_Transmit(s_ws_spi, buf, pos);
    if (st != BSP_SPI_OK)
    {
        s_ws_err = st;    //同步就能拿到的错误；DMA 异步错误由回调记录
    }
    return st;
}

bsp_spi_status_t WS2812_GetLastError(void)
{
    bsp_spi_status_t st = s_ws_err;

    s_ws_err = BSP_SPI_OK;      //读完清零
    return st;
}

// 颜色	按 GRB 顺序输出 (G, R, B)
// 黑	(0, 0, 0)
// 白	(255, 255, 255)
// 红	(0, 255, 0)
// 绿	(255, 0, 0)
// 蓝	(0, 0, 255)
// 黄	(255, 255, 0)
// 青	(0, 255, 255)
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
