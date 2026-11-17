//
// Created by DQJ on 2026/11/17.
//

#ifndef DMC02_MYTEST_BSP_SPI_H
#define DMC02_MYTEST_BSP_SPI_H

#include <stdint.h>
#include <stddef.h>


//================== SPI 使能开关(必须与 CubeMX 中实际启用的 SPI 一致) ==================
#define BSP_SPI1_ENABLE   1U    //目前只用 SPI1 测试 WS2812
#define BSP_SPI2_ENABLE   0U
#define BSP_SPI3_ENABLE   0U
#define BSP_SPI4_ENABLE   0U
#define BSP_SPI5_ENABLE   0U
#define BSP_SPI6_ENABLE   0U    //CubeMX 中启用 SPI6 后改为 1

//================== 缓冲区大小 ==================
//SPI1~SPI5：DMA 缓冲区，放在 SRAM1(H723 上 SRAM1 共 16KB，所有已使能的总和不能超过)
#define BSP_SPI1_TX_BUF_SIZE   4096U
#define BSP_SPI1_RX_BUF_SIZE   256U
#define BSP_SPI2_TX_BUF_SIZE   1024U
#define BSP_SPI2_RX_BUF_SIZE   256U
#define BSP_SPI3_TX_BUF_SIZE   1024U
#define BSP_SPI3_RX_BUF_SIZE   256U
#define BSP_SPI4_TX_BUF_SIZE   1024U
#define BSP_SPI4_RX_BUF_SIZE   256U
#define BSP_SPI5_TX_BUF_SIZE   1024U
#define BSP_SPI5_RX_BUF_SIZE   256U
//SPI6：阻塞方式，缓冲区放普通 RAM
#define BSP_SPI6_TX_BUF_SIZE   256U
#define BSP_SPI6_RX_BUF_SIZE   64U

#define BSP_SPI_DEFAULT_TIMEOUT_MS   100U        //默认超时 100ms
#define BSP_SPI_DEFAULT_BAUD_HZ      6000000UL   //默认 6Mbit/s

//================== 状态码 ==================
typedef enum
{
    BSP_SPI_OK = 0,          //成功
    BSP_SPI_BUSY,            //忙，立即返回，不阻塞
    BSP_SPI_TIMEOUT,         //超时
    BSP_SPI_DMA_ERROR,       //DMA 错误
    BSP_SPI_HAL_ERROR,       //HAL 其它错误
    BSP_SPI_PARAM_ERROR,     //参数错误 / SPI 未使能
    BSP_SPI_NOT_SUPPORT      //接收接口预留，暂未实现
} bsp_spi_status_t;

//================== SPI 编号 ==================
typedef enum
{
    BSP_SPI_1 = 0,
    BSP_SPI_2,
    BSP_SPI_3,
    BSP_SPI_4,
    BSP_SPI_5,
    BSP_SPI_6,
    BSP_SPI_NUM
} bsp_spi_id_t;

//================== 传输方式 ==================
typedef enum
{
    BSP_SPI_XFER_DMA = 0,    //SPI1~SPI5
    BSP_SPI_XFER_BLOCKING    //SPI6
} bsp_spi_xfer_mode_t;

//================== 通知事件 ==================
typedef enum
{
    BSP_SPI_EVT_TX_DONE = 0, //发送完成
    BSP_SPI_EVT_RX_DONE,     //接收完成(预留)
    BSP_SPI_EVT_ERROR        //出错，具体原因看 status
} bsp_spi_event_t;

typedef struct bsp_spi bsp_spi_t;

//上层通知函数指针。注意：DMA 模式下在中断中执行，处理要尽量短
typedef void (*bsp_spi_callback_t)(bsp_spi_t *dev,
                                   bsp_spi_event_t evt,
                                   bsp_spi_status_t status,
                                   void *user_arg);

//================== 初始化配置 ==================
typedef struct
{
    uint32_t           baudrate_hz;  //0 表示用默认 6MHz
    uint32_t           timeout_ms;   //0 表示用默认 100ms
    bsp_spi_callback_t callback;     //完成/出错通知，可为 NULL
    void              *user_arg;     //回调时原样传回
} bsp_spi_cfg_t;

//================== 操作函数表(函数指针) ==================
typedef struct
{
    bsp_spi_status_t (*init)      (bsp_spi_t *dev, const bsp_spi_cfg_t *cfg);
    bsp_spi_status_t (*deinit)    (bsp_spi_t *dev);
    bsp_spi_status_t (*transmit)  (bsp_spi_t *dev, const uint8_t *data, uint16_t len);
    bsp_spi_status_t (*receive)   (bsp_spi_t *dev, uint8_t *data, uint16_t len);  //预留
    bsp_spi_status_t (*poll)      (bsp_spi_t *dev);     //主循环调用，检测 DMA 超时
    uint8_t          (*is_busy)   (bsp_spi_t *dev);
    uint8_t *        (*get_tx_buf)(bsp_spi_t *dev, uint16_t *size);  //零拷贝写发送缓冲区
} bsp_spi_ops_t;

//================== 设备对象(上层只读) ==================
struct bsp_spi
{
    const bsp_spi_ops_t *ops;

    bsp_spi_id_t         id;
    bsp_spi_xfer_mode_t  mode;
    void                *handle;         //底层句柄，用 void* 隐藏 HAL 类型；NULL 表示未使能

    uint8_t             *tx_buf;
    uint16_t             tx_buf_size;
    uint8_t             *rx_buf;
    uint16_t             rx_buf_size;

    uint32_t             baudrate_hz;    //实际分频后的速率
    uint32_t             timeout_ms;

    volatile uint8_t     initialized;
    volatile uint8_t     busy;           //1：传输中
    volatile uint8_t     aborting;       //1：超时中止中
    volatile uint32_t    start_tick;
    volatile bsp_spi_status_t last_status;
    volatile uint32_t    err_count;

    bsp_spi_callback_t   callback;
    void                *user_arg;
};

//================== 对外函数 ==================
bsp_spi_t       *BSP_SPI_Get(bsp_spi_id_t id);
bsp_spi_status_t BSP_SPI_Init(bsp_spi_t *dev, const bsp_spi_cfg_t *cfg);
bsp_spi_status_t BSP_SPI_DeInit(bsp_spi_t *dev);
bsp_spi_status_t BSP_SPI_Transmit(bsp_spi_t *dev, const uint8_t *data, uint16_t len);
bsp_spi_status_t BSP_SPI_Receive(bsp_spi_t *dev, uint8_t *data, uint16_t len);
bsp_spi_status_t BSP_SPI_Poll(bsp_spi_t *dev);
uint8_t          BSP_SPI_IsBusy(bsp_spi_t *dev);
uint8_t         *BSP_SPI_GetTxBuf(bsp_spi_t *dev, uint16_t *size);



#endif //DMC02_MYTEST_BSP_SPI_H
