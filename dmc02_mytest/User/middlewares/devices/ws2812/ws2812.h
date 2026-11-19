//
// Created by DQJ on 2026/11/17.
//

#ifndef DMC02_MYTEST_WS2812_H
#define DMC02_MYTEST_WS2812_H
#include <stdint.h>
#include "bsp_spi.h"     //只用到 bsp_spi_status_t，不包含任何 HAL 头文件


//================== 配置 ==================
#define WS2812_SPI_ID      BSP_SPI_1   //改成 BSP_SPI_6 就是阻塞方式，其它代码不用改
#define WS2812_LED_NUM     1U         //灯珠数量

//6MHz 时 1 个 SPI 字节约 1.333us，对应 WS2812 的 1 个 bit
#define WS2812_BIT0        0xC0U       //高电平约 333ns
#define WS2812_BIT1        0xF8U       //高电平约 833ns
#define WS2812_RESET_HEAD  8U          //帧头低电平
#define WS2812_RESET_TAIL  240U        //帧尾复位低电平约 320us
#define WS2812_FRAME_LEN   (WS2812_RESET_HEAD + WS2812_LED_NUM * 24U + WS2812_RESET_TAIL)

//================== 初始化返回值 ==================
#define WS2812_INIT_OK          0      //成功
#define WS2812_INIT_ERR_GET    (-1)    //SPI 编号无效
#define WS2812_INIT_ERR_SPI    (-2)    //SPI 初始化失败(未使能 / 时钟 / DMA / SRAM1 段问题)
#define WS2812_INIT_ERR_BAUD   (-3)    //实际速率偏离 6MHz 过多

//================== 接口 ==================
//初始化：内部调用封装层的 init，速率 6MHz，超时 100ms
int WS2812_Init(void);

//设置单个灯的颜色，只写入颜色缓存，要调用 WS2812_Show 才会刷新
void WS2812_SetPixel(uint16_t idx, uint8_t g, uint8_t r, uint8_t b);

//清空颜色缓存(全部熄灭)，同样要调用 WS2812_Show 才会生效
void WS2812_Clear(void);

//把颜色缓存编码后发送，不阻塞
//返回 BSP_SPI_OK：已启动发送
//返回 BSP_SPI_BUSY：上一帧还没发完，稍后再调用
//返回 BSP_SPI_TIMEOUT / BSP_SPI_DMA_ERROR / BSP_SPI_HAL_ERROR / BSP_SPI_PARAM_ERROR：出错
bsp_spi_status_t WS2812_Show(void);

//读取最近一次在回调中记录的错误，读完自动清零
bsp_spi_status_t WS2812_GetLastError(void);

#endif //DMC02_MYTEST_WS2812_H
