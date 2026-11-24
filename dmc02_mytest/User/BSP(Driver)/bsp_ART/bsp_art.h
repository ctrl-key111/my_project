//
// Created by DQJ on 2026/11/19.
//
//说实话此驱动默认所有串口都是异步模式
#ifndef DMC02_MYTEST_BSP_UART_H
#define DMC02_MYTEST_BSP_UART_H

#include"stdint.h"

//==================  USART 使能开关（需与cubeMX保持一致）  ===============
//LPUART1目前用不上
#define BSP_UART4_ENABLE 1U
#define BSP_UART5_ENABLE 0U
#define BSP_UART7_ENABLE 0U
#define BSP_UART8_ENABLE 0U
#define BSP_UART9_ENABLE 0U


#define BSP_USART1_ENABLE 0U
#define BSP_USART2_ENABLE 0U
#define BSP_USART3_ENABLE 0U
#define BSP_USART6_ENABLE 0U
#define BSP_USART10_ENABLE 0U


//================== 缓冲区大小 ==================
#define BSP_UART4_TX_SIZE 1024U
#define BSP_UART4_RX_SIZE 1024U
#define BSP_UART5_TX_SIZE 1024U
#define BSP_UART5_RX_SIZE 1024U
#define BSP_UART7_TX_SIZE 1024U
#define BSP_UART7_RX_SIZE 1024U
#define BSP_UART8_TX_SIZE 1024U
#define BSP_UART8_RX_SIZE 1024U
#define BSP_UART9_TX_SIZE 1024U
#define BSP_UART9_RX_SIZE 1024U

#define BSP_USART1_TX_SIZE 1024U
#define BSP_USART1_RX_SIZE 1024U
#define BSP_USART2_TX_SIZE 1024U
#define BSP_USART2_RX_SIZE 1024U
#define BSP_USART3_TX_SIZE 1024U
#define BSP_USART3_RX_SIZE 1024U
#define BSP_USART6_TX_SIZE 1024U
#define BSP_USART6_RX_SIZE 1024U
#define BSP_USART10_TX_SIZE 1024U
#define BSP_USART10_RX_SIZE 1024U

#define BSP_ART_DEFAULT_TIMEOUT_MS   100U        //默认超时 100ms


//================ UART/USART 编号 ==================
typedef enum
{
    BSP_UART4 =0,
    BSP_UART5,
    BSP_UART7,
    BSP_UART8,
    BSP_UART9,
    BSP_USART1,
    BSP_USART2,
    BSP_USART3,
    BSP_USART6,
    BSP_USART10,
    BSP_ART_NUM
}bsp_art_id_t;
//================== 状态码 ==================
typedef enum
{
    BSP_USART_OK = 0,
    BSP_USART_ERR_PARAM,      // 参数错误(编号越界、空指针)
    BSP_USART_ERR_DISABLED,   // 该串口未使能
    BSP_USART_ERR_NOT_INIT,   // 未初始化
    BSP_USART_ERR_FULL,       // 发送缓冲区空间不足
    BSP_USART_ERR_HAL,        // HAL 返回错误
    BSP_USART_ERR_BUSY,       // 上一帧还在发送
    BSP_USART_ERR_TIMEOUT,    // 发送超时(已中止)
    BSP_USART_ERR_DMA,        // DMA 错误
} bsp_art_status_t;

//================== 通知事件 ==================
typedef enum
{
    BSP_ART_EVT_TX_DONE = 0, //发送完成
    BSP_ART_EVT_RX_DONE,     //接收完成(预留)
    BSP_ART_EVT_ERROR        //出错，具体原因看 status
} bsp_art_event_t;


typedef struct bsp_art bsp_art_t;

typedef void (*bsp_art_callback_t)(bsp_art_t *dev,
                                   bsp_art_event_t evt,
                                   bsp_art_status_t status,
                                   void *user_arg);

typedef struct
{
    uint32_t           timeout_ms;   //0 表示用默认 100ms
    bsp_art_callback_t callback;     //完成/出错通知，可为 NULL
    void              *user_arg;     //回调时原样传回
} bsp_art_cfg_t;

//================== 设备对象(上层只读，不要直接改) ==================
struct bsp_art
{
    //---- 固定信息 ----
    bsp_art_id_t       id;
    void              *handle;
    uint8_t           *tx_buf;
    uint16_t           tx_buf_size;

    //---- Init 时填写 ----
    uint32_t           timeout_ms;
    bsp_art_callback_t callback;
    void              *user_arg;

    //---- 运行状态(中断里会修改，所以加 volatile) ----
    volatile uint8_t          initialized;
    volatile uint8_t          busy;        //1：传输中
    volatile uint8_t          aborting;    //1：正在超时中止
    volatile uint32_t         start_tick;
    volatile bsp_art_status_t last_status;
    volatile uint32_t         err_count;
};

//================== 对外函数 ==================
bsp_art_t       *BSP_ART_Get(bsp_art_id_t id);
bsp_art_status_t BSP_ART_Init(bsp_art_t *dev,const bsp_art_cfg_t *cfg);
bsp_art_status_t BSP_ART_DeInit(bsp_art_t *dev);
bsp_art_status_t BSP_ART_Transmit(bsp_art_t *dev,const uint8_t *data,uint16_t len);
bsp_art_status_t BSP_ART_Receive(bsp_art_t *dev,uint8_t *data,uint16_t len);
bsp_art_status_t BSP_ART_Poll(bsp_art_t *dev);
uint8_t          BSP_ART_IsBusy(bsp_art_t *dev);
uint8_t         *BSP_ART_GetTxBuf(bsp_art_t *dev);


#endif //DMC02_MYTEST_BSP_UART_H
