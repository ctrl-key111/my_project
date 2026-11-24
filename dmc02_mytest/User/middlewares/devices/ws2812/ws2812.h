//
// Created by DQJ on 2026/11/17.
//

#ifndef DMC02_MYTEST_WS2812_H
#define DMC02_MYTEST_WS2812_H
#include <stdint.h>
#include "bsp_spi.h"     //bsp_spi_t / bsp_spi_status_t / bsp_spi_event_t 都从这里来，不包含任何 HAL 头文件


//================== 配置 ==================
#define WS2812_SPI_ID      BSP_SPI_1
#define WS2812_LED_NUM     1U


//改 WS2812_SPI_ID 时这里要同步改
#define WS2812_SPI_TX_BUF_SIZE   BSP_SPI1_TX_BUF_SIZE

//6MHz 时 1 个 SPI 字节约 1.333us，对应 WS2812 的 1 个 bit
#define WS2812_SPI_BAUD    6000000UL  //目标速率
#define WS2812_BAUD_MIN    5600000UL  //实际分频后的速率必须落在这个范围里，否则时序不对
#define WS2812_BAUD_MAX    6400000UL

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
#define WS2812_INIT_ERR_BUF    (-4)    //发送缓冲区放不下一帧(小于 WS2812_FRAME_LEN)

//================== 接口 ==================
//初始化：内部调用 BSP_SPI_Init，速率 6MHz，超时 100ms
//注意：调用前要已经执行过 MX_SPI1_Init()（CubeMX 生成的初始化）
int WS2812_Init(void);

//设置单个灯的颜色，只写入颜色缓存，要调用 WS2812_Show 才会刷新
void WS2812_SetPixel(uint16_t idx, uint8_t g, uint8_t r, uint8_t b);

//清空颜色缓存(全部熄灭)，同样要调用 WS2812_Show 才会生效
void WS2812_Clear(void);

//把颜色缓存编码后发送
//返回 BSP_SPI_OK：DMA 方式下表示"已启动发送"，SPI6 阻塞方式下表示"已经发完"
//返回 BSP_SPI_BUSY：上一帧还没发完，稍后再调用
//返回 BSP_SPI_TIMEOUT / BSP_SPI_DMA_ERROR / BSP_SPI_HAL_ERROR / BSP_SPI_PARAM_ERROR：出错
bsp_spi_status_t WS2812_Show(void);

//读取最近一次出错的状态，读完自动清零；返回 BSP_SPI_OK 表示最近没有出错
//DMA 方式下错误是异步产生的(在回调里记录)，所以要多读几次或配合 WS2812_Show 的返回值判断
bsp_spi_status_t WS2812_GetLastError(void);

#endif //DMC02_MYTEST_WS2812_H
