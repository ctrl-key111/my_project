//
// Created by DQJ on 2026/11/19.
//

#include "bsp_art.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

//================== AXI SRAM ==================
//H723 的 AXI SRAM 从 0x24000000 开始，默认(TCM_AXI_SHARED=00)是 320KB，到 0x2404FFFF
//(512KB 是 H743 的大小，H723 没有那么大)
//DMA1/DMA2 能访问 AXI SRAM；链接脚本里需要有 .axi_sram 段(NOLOAD)
#define AXI_SRAM_START   0x24000000UL
#define AXI_SRAM_END     0x24050000UL
#define IN_AXI_SRAM      __attribute__((section(".axi_sram"), aligned(32)))

//================== 句柄(未使能的为 NULL) ==================
#if BSP_UART4_ENABLE
extern UART_HandleTypeDef huart4;
#define H_UART4    (&huart4)
#else
#define H_UART4    NULL
#endif

#if BSP_UART5_ENABLE
extern UART_HandleTypeDef huart5;
#define H_UART5    (&huart5)
#else
#define H_UART5    NULL
#endif

#if BSP_UART7_ENABLE
extern UART_HandleTypeDef huart7;
#define H_UART7    (&huart7)
#else
#define H_UART7    NULL
#endif

#if BSP_UART8_ENABLE
extern UART_HandleTypeDef huart8;
#define H_UART8    (&huart8)
#else
#define H_UART8    NULL
#endif

#if BSP_UART9_ENABLE
extern UART_HandleTypeDef huart9;
#define H_UART9    (&huart9)
#else
#define H_UART9    NULL
#endif

/*---------- USART ----------*/
#if BSP_USART1_ENABLE
extern UART_HandleTypeDef huart1;
#define H_USART1   (&huart1)
#else
#define H_USART1   NULL
#endif

#if BSP_USART2_ENABLE
extern UART_HandleTypeDef huart2;
#define H_USART2   (&huart2)
#else
#define H_USART2   NULL
#endif

#if BSP_USART3_ENABLE
extern UART_HandleTypeDef huart3;
#define H_USART3   (&huart3)
#else
#define H_USART3   NULL
#endif

#if BSP_USART6_ENABLE
extern UART_HandleTypeDef huart6;
#define H_USART6   (&huart6)
#else
#define H_USART6   NULL
#endif

#if BSP_USART10_ENABLE
extern UART_HandleTypeDef huart10;
#define H_USART10  (&huart10)
#else
#define H_USART10  NULL
#endif

//================== 发送缓冲区(未使能的不占内存) ==================
#if BSP_UART4_ENABLE
static uint8_t s_uart4_tx_buf[BSP_UART4_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_UART5_ENABLE
static uint8_t s_uart5_tx_buf[BSP_UART5_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_UART7_ENABLE
static uint8_t s_uart7_tx_buf[BSP_UART7_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_UART8_ENABLE
static uint8_t s_uart8_tx_buf[BSP_UART8_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_UART9_ENABLE
static uint8_t s_uart9_tx_buf[BSP_UART9_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_USART1_ENABLE
static uint8_t s_usart1_tx_buf[BSP_USART1_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_USART2_ENABLE
static uint8_t s_usart2_tx_buf[BSP_USART2_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_USART3_ENABLE
static uint8_t s_usart3_tx_buf[BSP_USART3_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_USART6_ENABLE
static uint8_t s_usart6_tx_buf[BSP_USART6_TX_SIZE] IN_AXI_SRAM;
#endif
#if BSP_USART10_ENABLE
static uint8_t s_usart10_tx_buf[BSP_USART10_TX_SIZE] IN_AXI_SRAM;
#endif

//设备表：没写的项全部为 0，handle = NULL 表示未使能
static bsp_art_t s_art_dev[BSP_ART_NUM] =
{
#if BSP_UART4_ENABLE
    [BSP_UART4]   = { .id = BSP_UART4,   .handle = H_UART4,   .tx_buf = s_uart4_tx_buf,   .tx_buf_size = BSP_UART4_TX_SIZE },
#endif
#if BSP_UART5_ENABLE
    [BSP_UART5]   = { .id = BSP_UART5,   .handle = H_UART5,   .tx_buf = s_uart5_tx_buf,   .tx_buf_size = BSP_UART5_TX_SIZE },
#endif
#if BSP_UART7_ENABLE
    [BSP_UART7]   = { .id = BSP_UART7,   .handle = H_UART7,   .tx_buf = s_uart7_tx_buf,   .tx_buf_size = BSP_UART7_TX_SIZE },
#endif
#if BSP_UART8_ENABLE
    [BSP_UART8]   = { .id = BSP_UART8,   .handle = H_UART8,   .tx_buf = s_uart8_tx_buf,   .tx_buf_size = BSP_UART8_TX_SIZE },
#endif
#if BSP_UART9_ENABLE
    [BSP_UART9]   = { .id = BSP_UART9,   .handle = H_UART9,   .tx_buf = s_uart9_tx_buf,   .tx_buf_size = BSP_UART9_TX_SIZE },
#endif
#if BSP_USART1_ENABLE
    [BSP_USART1]  = { .id = BSP_USART1,  .handle = H_USART1,  .tx_buf = s_usart1_tx_buf,  .tx_buf_size = BSP_USART1_TX_SIZE },
#endif
#if BSP_USART2_ENABLE
    [BSP_USART2]  = { .id = BSP_USART2,  .handle = H_USART2,  .tx_buf = s_usart2_tx_buf,  .tx_buf_size = BSP_USART2_TX_SIZE },
#endif
#if BSP_USART3_ENABLE
    [BSP_USART3]  = { .id = BSP_USART3,  .handle = H_USART3,  .tx_buf = s_usart3_tx_buf,  .tx_buf_size = BSP_USART3_TX_SIZE },
#endif
#if BSP_USART6_ENABLE
    [BSP_USART6]  = { .id = BSP_USART6,  .handle = H_USART6,  .tx_buf = s_usart6_tx_buf,  .tx_buf_size = BSP_USART6_TX_SIZE },
#endif
#if BSP_USART10_ENABLE
    [BSP_USART10] = { .id = BSP_USART10, .handle = H_USART10, .tx_buf = s_usart10_tx_buf, .tx_buf_size = BSP_USART10_TX_SIZE },
#endif
};

//================== 内部函数 ==================
//一次发送结束(成功/出错/超时都走这里)：记录状态、释放 busy、通知上层
static void art_finish(bsp_art_t *dev, bsp_art_status_t st)
{
    dev->last_status = st;
    if (st != BSP_USART_OK)
    {
        dev->err_count++;
    }
    dev->busy = 0U;     //先释放 busy，上层在回调里就能发下一帧

    if (dev->callback != NULL)
    {
        bsp_art_event_t evt = (st == BSP_USART_OK) ? BSP_ART_EVT_TX_DONE : BSP_ART_EVT_ERROR;
        dev->callback(dev, evt, st, dev->user_arg);
    }
}

//HAL 返回值 -> 本层状态码
static bsp_art_status_t art_map_hal(HAL_StatusTypeDef hs, UART_HandleTypeDef *h)
{
    if (hs == HAL_OK)      { return BSP_USART_OK; }
    if (hs == HAL_BUSY)    { return BSP_USART_ERR_BUSY; }
    if (hs == HAL_TIMEOUT) { return BSP_USART_ERR_TIMEOUT; }
    return ((h->ErrorCode & HAL_UART_ERROR_DMA) != 0U) ? BSP_USART_ERR_DMA : BSP_USART_ERR_HAL;
}

//DMA 超时检测：超过 timeout_ms 还没完成，就中止发送并通知上层
static bsp_art_status_t art_check_timeout(bsp_art_t *dev)
{
    uint32_t primask;

    if (dev->busy == 0U)
    {
        return BSP_USART_OK;
    }
    if ((HAL_GetTick() - dev->start_tick) < dev->timeout_ms)
    {
        return BSP_USART_ERR_BUSY;                  //还没超时
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (dev->busy == 0U)              //恰好在中断里完成了
    {
        __set_PRIMASK(primask);
        return BSP_USART_OK;
    }
    dev->aborting = 1U;               //让中断回调忽略中止过程中的事件
    __set_PRIMASK(primask);

    //只中止发送，不影响以后可能在跑的接收
    (void)HAL_UART_AbortTransmit((UART_HandleTypeDef *)dev->handle);
    dev->aborting = 0U;
    art_finish(dev, BSP_USART_ERR_TIMEOUT);
    return BSP_USART_ERR_TIMEOUT;
}

//通过 HAL 句柄找到设备对象(HAL 回调里用)
static bsp_art_t *art_find_dev(UART_HandleTypeDef *huart)
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)BSP_ART_NUM; i++)
    {
        if ((s_art_dev[i].handle != NULL) && (s_art_dev[i].handle == (void *)huart))
        {
            return &s_art_dev[i];
        }
    }
    return NULL;
}

//================== 对外接口 ==================
bsp_art_t *BSP_ART_Get(bsp_art_id_t id)
{
    if ((uint32_t)id >= (uint32_t)BSP_ART_NUM)
    {
        return NULL;
    }
    return &s_art_dev[id];
}

bsp_art_status_t BSP_ART_Init(bsp_art_t *dev, const bsp_art_cfg_t *cfg)
{
    UART_HandleTypeDef *h;
    uint32_t addr;

    //1. 参数检查
    if (dev == NULL)
    {
        return BSP_USART_ERR_PARAM;
    }
    if (dev->handle == NULL)          //handle 为 NULL 表示该串口未使能
    {
        return BSP_USART_ERR_DISABLED;
    }
    if (dev->busy)
    {
        return BSP_USART_ERR_BUSY;
    }
    h = (UART_HandleTypeDef *)dev->handle;

    //2. 波特率、数据位等都由 CubeMX 的 MX_xxx_Init() 配置，这里只检查它有没有被调用
    if (h->gState == HAL_UART_STATE_RESET)
    {
        return BSP_USART_ERR_NOT_INIT;   //先在 main 里调用 MX_UARTx_Init()
    }

    //3. 检查缓冲区确实在 AXI SRAM(DMA 能访问)
    addr = (uint32_t)dev->tx_buf;
    if ((addr < AXI_SRAM_START) || ((addr + dev->tx_buf_size) > AXI_SRAM_END))
    {
        return BSP_USART_ERR_PARAM;      //链接脚本没生效(.axi_sram 段没放进 RAM_D1)
    }

    //4. 确认 CubeMX 已经把 TX DMA 连到句柄上
    if (h->hdmatx == NULL)
    {
        return BSP_USART_ERR_DMA;
    }

    //5. 保存配置
    dev->timeout_ms = ((cfg != NULL) && (cfg->timeout_ms != 0U)) ? cfg->timeout_ms : BSP_ART_DEFAULT_TIMEOUT_MS;
    dev->callback   = (cfg != NULL) ? cfg->callback : NULL;
    dev->user_arg   = (cfg != NULL) ? cfg->user_arg : NULL;

    memset(dev->tx_buf, 0, dev->tx_buf_size);   //NOLOAD 段，手动清零

    dev->busy        = 0U;
    dev->aborting    = 0U;
    dev->err_count   = 0U;
    dev->last_status = BSP_USART_OK;
    dev->initialized = 1U;
    return BSP_USART_OK;
}

//注意：DeInit 后 HAL 状态变成 RESET，想再用要重新调用 MX_UARTx_Init() + BSP_ART_Init()
bsp_art_status_t BSP_ART_DeInit(bsp_art_t *dev)
{
    UART_HandleTypeDef *h;

    if (dev == NULL)
    {
        return BSP_USART_ERR_PARAM;
    }
    if (dev->handle == NULL)
    {
        return BSP_USART_ERR_DISABLED;
    }
    h = (UART_HandleTypeDef *)dev->handle;

    if (dev->busy)
    {
        dev->aborting = 1U;
        (void)HAL_UART_Abort(h);
    }
    dev->busy        = 0U;
    dev->aborting    = 0U;
    dev->initialized = 0U;

    return (HAL_UART_DeInit(h) == HAL_OK) ? BSP_USART_OK : BSP_USART_ERR_HAL;
}

bsp_art_status_t BSP_ART_Transmit(bsp_art_t *dev, const uint8_t *data, uint16_t len)
{
    UART_HandleTypeDef *h;
    HAL_StatusTypeDef   hs;
    bsp_art_status_t    st;
    uint32_t            primask;

    //1. 参数检查
    if ((dev == NULL) || (data == NULL) || (len == 0U))
    {
        return BSP_USART_ERR_PARAM;
    }
    if (dev->handle == NULL)
    {
        return BSP_USART_ERR_DISABLED;
    }
    if (dev->initialized == 0U)
    {
        return BSP_USART_ERR_NOT_INIT;
    }
    if (len > dev->tx_buf_size)
    {
        return BSP_USART_ERR_FULL;
    }
    h = (UART_HandleTypeDef *)dev->handle;

    //2. 占用串口(关中断保证"检查 busy + 置 busy"不被打断)
    //   忙：立即返回 BUSY；上一帧超过 timeout_ms 未完成，则中止它并返回 TIMEOUT
    primask = __get_PRIMASK();
    __disable_irq();
    if (dev->busy)
    {
        __set_PRIMASK(primask);
        return (art_check_timeout(dev) == BSP_USART_ERR_TIMEOUT) ? BSP_USART_ERR_TIMEOUT : BSP_USART_ERR_BUSY;
    }
    dev->busy       = 1U;
    dev->start_tick = HAL_GetTick();
    __set_PRIMASK(primask);

    //3. 拷贝到 DMA 缓冲区
    if (data != dev->tx_buf)          //零拷贝时数据已在缓冲区，不用再拷
    {
        memcpy(dev->tx_buf, data, len);
    }

    //4. AXI SRAM 默认可缓存：开了 DCache 就自动 Clean，没开就跳过
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)dev->tx_buf, (int32_t)(((uint32_t)len + 31U) & ~31UL));
    }
#endif

    //5. 启动 DMA 发送，发完后在 HAL_UART_TxCpltCallback 里释放 busy
    hs = HAL_UART_Transmit_DMA(h, dev->tx_buf, len);
    if (hs != HAL_OK)                 //启动失败：立即释放(不调回调)
    {
        st = art_map_hal(hs, h);
        dev->last_status = st;
        dev->err_count++;
        dev->busy = 0U;
        return st;
    }
    return BSP_USART_OK;
}

//接收：目前是简单的阻塞接收，最多等 timeout_ms
//TODO：以后改成 HAL_UARTEx_ReceiveToIdle_DMA + RX 缓冲区(BSP_xxx_RX_SIZE)，用 RX_DONE 事件通知
bsp_art_status_t BSP_ART_Receive(bsp_art_t *dev, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef hs;

    if ((dev == NULL) || (data == NULL) || (len == 0U))
    {
        return BSP_USART_ERR_PARAM;
    }
    if (dev->handle == NULL)
    {
        return BSP_USART_ERR_DISABLED;
    }
    if (dev->initialized == 0U)
    {
        return BSP_USART_ERR_NOT_INIT;
    }

    //UART 是全双工，接收不受发送 busy 影响
    hs = HAL_UART_Receive((UART_HandleTypeDef *)dev->handle, data, len, dev->timeout_ms);
    return art_map_hal(hs, (UART_HandleTypeDef *)dev->handle);
}

//主循环里周期调用：检测 DMA 是否超时
bsp_art_status_t BSP_ART_Poll(bsp_art_t *dev)
{
    if (dev == NULL)
    {
        return BSP_USART_ERR_PARAM;
    }
    if (dev->busy)
    {
        return art_check_timeout(dev);   //BUSY 或 TIMEOUT
    }
    return dev->last_status;
}

uint8_t BSP_ART_IsBusy(bsp_art_t *dev)
{
    return (dev != NULL) ? dev->busy : 0U;
}

//零拷贝：先拿到缓冲区直接写，再调用 BSP_ART_Transmit(dev, buf, len)
//缓冲区大小看 dev->tx_buf_size；busy 期间不要写，否则会破坏正在发送的数据
uint8_t *BSP_ART_GetTxBuf(bsp_art_t *dev)
{
    return (dev != NULL) ? dev->tx_buf : NULL;
}

//================== HAL 回调(覆盖 HAL 的 weak 函数，工程其它地方不要再定义) ==================
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_art_t *dev = art_find_dev(huart);

    if ((dev == NULL) || (dev->busy == 0U) || (dev->aborting != 0U))
    {
        return;
    }
    art_finish(dev, BSP_USART_OK);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    bsp_art_t *dev = art_find_dev(huart);

    if ((dev == NULL) || (dev->busy == 0U) || (dev->aborting != 0U))
    {
        return;
    }
    //UART 的接收错误(ORE/FE/NE)也会进这里；发送还在进行说明不是发送出错，忽略
    if (huart->gState == HAL_UART_STATE_BUSY_TX)
    {
        return;
    }
    art_finish(dev, art_map_hal(HAL_ERROR, huart));   //自动区分 DMA 错误 / 其它错误
}