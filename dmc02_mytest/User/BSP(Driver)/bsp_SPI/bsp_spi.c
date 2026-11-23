//
// Created by DQJ on 2026/11/17.
//

#include "bsp_spi.h"
#include "bsp_spi.h"
#include "stm32h7xx_hal.h"
#include <string.h>

//================== 内核时钟：0 = 自动从 RCC 读取；读不到时手动填写 ==================
#define BSP_SPI123_KERNEL_CLK_HZ   0UL     //SPI1/2/3 共用
#define BSP_SPI5_KERNEL_CLK_HZ     0UL     //SPI5(HAL 里时钟名叫 SPI45)
#define BSP_SPI6_KERNEL_CLK_HZ     0UL

//================== SRAM1 ==================
//H723 的 SRAM1 = 0x30000000~0x30003FFF(16KB)，DMA1/DMA2 能访问；DTCM(0x20000000) 不能
//链接脚本里需要有 .sram1 段(NOLOAD)，放在 0x30000000
#define SRAM1_START   0x30000000UL
#define SRAM1_END     0x30004000UL
#define IN_SRAM1      __attribute__((section(".sram1"), aligned(32)))

//================== 各路 SPI 的句柄和缓冲区(未使能的不占内存) ==================
#if BSP_SPI1_ENABLE
extern SPI_HandleTypeDef hspi1;
static uint8_t s_spi1_tx_buf[BSP_SPI1_TX_BUF_SIZE] IN_SRAM1;
#endif
#if BSP_SPI2_ENABLE
extern SPI_HandleTypeDef hspi2;
static uint8_t s_spi2_tx_buf[BSP_SPI2_TX_BUF_SIZE] IN_SRAM1;
#endif
#if BSP_SPI3_ENABLE
extern SPI_HandleTypeDef hspi3;
static uint8_t s_spi3_tx_buf[BSP_SPI3_TX_BUF_SIZE] IN_SRAM1;
#endif
#if BSP_SPI5_ENABLE
extern SPI_HandleTypeDef hspi5;
static uint8_t s_spi5_tx_buf[BSP_SPI5_TX_BUF_SIZE] IN_SRAM1;
#endif
#if BSP_SPI6_ENABLE
extern SPI_HandleTypeDef hspi6;
static uint8_t s_spi6_tx_buf[BSP_SPI6_TX_BUF_SIZE];   //SPI6 阻塞发送，普通 RAM 即可
#endif

//设备表：没写的项全部为 0，handle = NULL 表示未使能，所有接口都会返回 PARAM_ERROR
static bsp_spi_t s_spi_dev[BSP_SPI_NUM] =
{
#if BSP_SPI1_ENABLE
    [BSP_SPI_1] = { .id = BSP_SPI_1, .handle = &hspi1, .tx_buf = s_spi1_tx_buf, .tx_buf_size = BSP_SPI1_TX_BUF_SIZE },
#endif
#if BSP_SPI2_ENABLE
    [BSP_SPI_2] = { .id = BSP_SPI_2, .handle = &hspi2, .tx_buf = s_spi2_tx_buf, .tx_buf_size = BSP_SPI2_TX_BUF_SIZE },
#endif
#if BSP_SPI3_ENABLE
    [BSP_SPI_3] = { .id = BSP_SPI_3, .handle = &hspi3, .tx_buf = s_spi3_tx_buf, .tx_buf_size = BSP_SPI3_TX_BUF_SIZE },
#endif
#if BSP_SPI5_ENABLE
    [BSP_SPI_5] = { .id = BSP_SPI_5, .handle = &hspi5, .tx_buf = s_spi5_tx_buf, .tx_buf_size = BSP_SPI5_TX_BUF_SIZE },
#endif
#if BSP_SPI6_ENABLE
    [BSP_SPI_6] = { .id = BSP_SPI_6, .handle = &hspi6, .tx_buf = s_spi6_tx_buf, .tx_buf_size = BSP_SPI6_TX_BUF_SIZE },
#endif
};

//================== 内部函数 ==================
//一次传输结束(成功/出错/超时都走这里)：记录状态、释放 busy、通知上层
static void spi_finish(bsp_spi_t *dev, bsp_spi_status_t st)
{
    dev->last_status = st;
    if (st != BSP_SPI_OK)
    {
        dev->err_count++;
    }
    dev->busy = 0U;     //先释放 busy，上层在回调里就能发下一帧

    if (dev->callback != NULL)
    {
        bsp_spi_event_t evt = (st == BSP_SPI_OK) ? BSP_SPI_EVT_TX_DONE : BSP_SPI_EVT_ERROR;
        dev->callback(dev, evt, st, dev->user_arg);
    }
}

//HAL 返回值 -> 本层状态码
static bsp_spi_status_t spi_map_hal(HAL_StatusTypeDef hs, SPI_HandleTypeDef *h)
{
    if (hs == HAL_OK)      { return BSP_SPI_OK; }
    if (hs == HAL_BUSY)    { return BSP_SPI_BUSY; }
    if (hs == HAL_TIMEOUT) { return BSP_SPI_TIMEOUT; }
    return ((h->ErrorCode & HAL_SPI_ERROR_DMA) != 0U) ? BSP_SPI_DMA_ERROR : BSP_SPI_HAL_ERROR;
}

//DMA 超时检测：超过 timeout_ms 还没完成，就中止传输并通知上层
static bsp_spi_status_t spi_check_timeout(bsp_spi_t *dev)
{
    uint32_t primask;

    if ((dev->id == BSP_SPI_6) || (dev->busy == 0U))   //只检查 DMA 方式
    {
        return BSP_SPI_OK;
    }
    if ((HAL_GetTick() - dev->start_tick) < dev->timeout_ms)
    {
        return BSP_SPI_BUSY;                            //还没超时
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (dev->busy == 0U)              //恰好在中断里完成了
    {
        __set_PRIMASK(primask);
        return BSP_SPI_OK;
    }
    dev->aborting = 1U;               //让中断回调忽略中止过程中的事件
    __set_PRIMASK(primask);

    (void)HAL_SPI_Abort((SPI_HandleTypeDef *)dev->handle);
    dev->aborting = 0U;
    spi_finish(dev, BSP_SPI_TIMEOUT);
    return BSP_SPI_TIMEOUT;
}

//读取 SPI 内核时钟
static uint32_t spi_get_kernel_clk(bsp_spi_id_t id)
{
    if ((id == BSP_SPI_1) || (id == BSP_SPI_2) || (id == BSP_SPI_3))
    {
        return (BSP_SPI123_KERNEL_CLK_HZ != 0UL) ? BSP_SPI123_KERNEL_CLK_HZ
               : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
    }
    else if (id == BSP_SPI_5)
    {
        return (BSP_SPI5_KERNEL_CLK_HZ != 0UL) ? BSP_SPI5_KERNEL_CLK_HZ
               : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
    }
    else
    {
        return (BSP_SPI6_KERNEL_CLK_HZ != 0UL) ? BSP_SPI6_KERNEL_CLK_HZ
               : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
    }
}

//在 2,4,8...256 分频中选出最接近目标速率的一个
static uint32_t spi_calc_prescaler(uint32_t clk, uint32_t target, uint32_t *actual)
{
    static const uint32_t reg[8] =
    {
        SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
    };
    uint32_t i, best = 0U, best_err = 0xFFFFFFFFUL;

    for (i = 0U; i < 8U; i++)
    {
        uint32_t f   = clk / (2U << i);    //2U << i 依次为 2,4,8...256
        uint32_t err = (f > target) ? (f - target) : (target - f);
        if (err < best_err)
        {
            best_err = err;
            best     = i;
        }
    }
    *actual = clk / (2U << best);
    return reg[best];
}

//通过 HAL 句柄找到设备对象(HAL 回调里用)
static bsp_spi_t *spi_find_dev(SPI_HandleTypeDef *hspi)
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)BSP_SPI_NUM; i++)
    {
        if ((s_spi_dev[i].handle != NULL) && (s_spi_dev[i].handle == (void *)hspi))
        {
            return &s_spi_dev[i];
        }
    }
    return NULL;
}

//================== 对外接口 ==================
bsp_spi_t *BSP_SPI_Get(bsp_spi_id_t id)
{
    if ((uint32_t)id >= (uint32_t)BSP_SPI_NUM)
    {
        return NULL;
    }
    return &s_spi_dev[id];
}

bsp_spi_status_t BSP_SPI_Init(bsp_spi_t *dev, const bsp_spi_cfg_t *cfg)
{
    SPI_HandleTypeDef *h;
    uint32_t clk;
    uint32_t target;
    uint32_t actual = 0U;

    if ((dev == NULL) || (dev->handle == NULL))   //handle 为 NULL 表示该 SPI 未使能
    {
        return BSP_SPI_PARAM_ERROR;
    }
    if (dev->busy)
    {
        return BSP_SPI_BUSY;
    }
    h = (SPI_HandleTypeDef *)dev->handle;

    //1. 用 if / else 确认 SPI 编号(只编译已使能的，没用到的外设不会被引用)
    if (0) { }
#if BSP_SPI1_ENABLE
    else if (dev->id == BSP_SPI_1) { h->Instance = SPI1; }
#endif
#if BSP_SPI2_ENABLE
    else if (dev->id == BSP_SPI_2) { h->Instance = SPI2; }
#endif
#if BSP_SPI3_ENABLE
    else if (dev->id == BSP_SPI_3) { h->Instance = SPI3; }
#endif
#if BSP_SPI5_ENABLE
    else if (dev->id == BSP_SPI_5) { h->Instance = SPI5; }
#endif
#if BSP_SPI6_ENABLE
    else if (dev->id == BSP_SPI_6) { h->Instance = SPI6; }
#endif
    else { return BSP_SPI_PARAM_ERROR; }

    //2. 读内核时钟
    clk = spi_get_kernel_clk(dev->id);
    if (clk == 0U)
    {
        return BSP_SPI_PARAM_ERROR;    //读不到时，在文件开头手动填 BSP_SPIxxx_KERNEL_CLK_HZ
    }

    //3. DMA 方式(SPI1/2/3/5)：打开 SRAM1 时钟，并检查缓冲区确实在 SRAM1
    if (dev->id != BSP_SPI_6)
    {
#ifdef __HAL_RCC_D2SRAM1_CLK_ENABLE
        __HAL_RCC_D2SRAM1_CLK_ENABLE();
#endif
        uint32_t addr = (uint32_t)dev->tx_buf;
        if ((addr < SRAM1_START) || ((addr + dev->tx_buf_size) > SRAM1_END))
        {
            return BSP_SPI_PARAM_ERROR;   //链接脚本没生效，或缓冲区总和超过 16KB
        }
    }

    //4. 保存配置
    target          = ((cfg != NULL) && (cfg->baudrate_hz != 0U)) ? cfg->baudrate_hz : BSP_SPI_DEFAULT_BITS;
    dev->timeout_ms = ((cfg != NULL) && (cfg->timeout_ms  != 0U)) ? cfg->timeout_ms  : BSP_SPI_DEFAULT_TIMEOUT_MS;
    dev->callback   = (cfg != NULL) ? cfg->callback : NULL;
    dev->user_arg   = (cfg != NULL) ? cfg->user_arg : NULL;

    
    if (h->State == HAL_SPI_STATE_RESET)
    {
        return BSP_SPI_PARAM_ERROR;   //MX_SPIx_Init() 还没调用，先在 main 里初始化 CubeMX 外设
    }
    h->Init.BaudRatePrescaler = spi_calc_prescaler(clk, target, &actual);

    if (HAL_SPI_Init(h) != HAL_OK)
    {
        return BSP_SPI_HAL_ERROR;
    }

    if (HAL_SPI_Init(h) != HAL_OK)
    {
        return BSP_SPI_HAL_ERROR;
    }

    //6. DMA 方式要确认 CubeMX 已经把 TX DMA 连到句柄上
    if ((dev->id != BSP_SPI_6) && (h->hdmatx == NULL))
    {
        return BSP_SPI_DMA_ERROR;
    }

    memset(dev->tx_buf, 0, dev->tx_buf_size);   //SRAM1 是 NOLOAD 段，手动清零

    dev->baudrate_hz = actual;
    dev->busy        = 0U;
    dev->aborting    = 0U;
    dev->err_count   = 0U;
    dev->last_status = BSP_SPI_OK;
    dev->initialized = 1U;
    return BSP_SPI_OK;
}

bsp_spi_status_t BSP_SPI_DeInit(bsp_spi_t *dev)
{
    SPI_HandleTypeDef *h;

    if ((dev == NULL) || (dev->handle == NULL))
    {
        return BSP_SPI_PARAM_ERROR;
    }
    h = (SPI_HandleTypeDef *)dev->handle;

    if (dev->busy)
    {
        dev->aborting = 1U;
        (void)HAL_SPI_Abort(h);
    }
    dev->busy        = 0U;
    dev->aborting    = 0U;
    dev->initialized = 0U;

    return (HAL_SPI_DeInit(h) == HAL_OK) ? BSP_SPI_OK : BSP_SPI_HAL_ERROR;
}

bsp_spi_status_t BSP_SPI_Transmit(bsp_spi_t *dev, const uint8_t *data, uint16_t len)
{
    SPI_HandleTypeDef *h;
    HAL_StatusTypeDef  hs;
    bsp_spi_status_t   st;
    uint32_t           primask;

    //1. 参数检查
    if ((dev == NULL) || (dev->handle == NULL) || (dev->initialized == 0U) ||
        (data == NULL) || (len == 0U))
    {
        return BSP_SPI_PARAM_ERROR;
    }
    h = (SPI_HandleTypeDef *)dev->handle;

    //2. 占用 SPI(关中断保证"检查 busy + 置 busy"不被打断)
    //   忙：立即返回 BUSY；上一帧超过 timeout_ms 未完成，则中止它并返回 TIMEOUT
    primask = __get_PRIMASK();
    __disable_irq();
    if (dev->busy)
    {
        __set_PRIMASK(primask);
        return (spi_check_timeout(dev) == BSP_SPI_TIMEOUT) ? BSP_SPI_TIMEOUT : BSP_SPI_BUSY;
    }
    dev->busy       = 1U;
    dev->start_tick = HAL_GetTick();
    __set_PRIMASK(primask);

    //3. 用 if / else 确认 SPI 编号，选择发送方式
    if (dev->id == BSP_SPI_6)
    {
        //SPI6：阻塞发送，函数返回时已经发完
        hs = HAL_SPI_Transmit(h, (uint8_t *)data, len, dev->timeout_ms);
        st = spi_map_hal(hs, h);
        spi_finish(dev, st);
        return st;
    }
    else
    {
        //SPI1/2/3/5：DMA 发送，发完后在 HAL_SPI_TxCpltCallback 里释放 busy
        if (len > dev->tx_buf_size)
        {
            dev->busy = 0U;
            return BSP_SPI_PARAM_ERROR;
        }
        if (data != dev->tx_buf)          //零拷贝时数据已在缓冲区，不用再拷
        {
            memcpy(dev->tx_buf, data, len);
        }
        //现在没开 DCache，不用 Clean；以后开了要在这里 SCB_CleanDCache_by_Addr

        hs = HAL_SPI_Transmit_DMA(h, dev->tx_buf, len);
        if (hs != HAL_OK)                 //启动失败：立即释放(不调回调)
        {
            st = spi_map_hal(hs, h);
            dev->last_status = st;
            dev->err_count++;
            dev->busy = 0U;
            return st;
        }
        return BSP_SPI_OK;
    }
}

//接收接口预留：目前只实现了 Master TX only
bsp_spi_status_t BSP_SPI_Receive(bsp_spi_t *dev, uint8_t *data, uint16_t len)
{
    if ((dev == NULL) || (dev->handle == NULL) || (data == NULL) || (len == 0U))
    {
        return BSP_SPI_PARAM_ERROR;
    }
    //TODO：SPI1/2/3/5 用 HAL_SPI_Receive_DMA，SPI6 用 HAL_SPI_Receive
    //      需要把 Direction 改成 SPI_DIRECTION_2LINES，并加上 RX 缓冲区
    return BSP_SPI_NOT_SUPPORT;
}

//主循环里周期调用：检测 DMA 是否超时
bsp_spi_status_t BSP_SPI_Poll(bsp_spi_t *dev)
{
    if (dev == NULL)
    {
        return BSP_SPI_PARAM_ERROR;
    }
    if (dev->busy)
    {
        return spi_check_timeout(dev);   //BUSY 或 TIMEOUT
    }
    return dev->last_status;
}

uint8_t BSP_SPI_IsBusy(bsp_spi_t *dev)
{
    return (dev != NULL) ? dev->busy : 0U;
}

//零拷贝：先拿到缓冲区直接写，再调用 BSP_SPI_Transmit(dev, buf, len)
//busy 期间不要写，否则会破坏正在发送的数据
uint8_t *BSP_SPI_GetTxBuf(bsp_spi_t *dev, uint16_t *size)
{
    if (dev == NULL)
    {
        return NULL;
    }
    if (size != NULL)
    {
        *size = dev->tx_buf_size;
    }
    return dev->tx_buf;
}

//================== HAL 回调(覆盖 HAL 的 weak 函数，工程其它地方不要再定义) ==================
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_t *dev = spi_find_dev(hspi);

    if ((dev == NULL) || (dev->busy == 0U) || (dev->aborting != 0U))
    {
        return;
    }
    spi_finish(dev, BSP_SPI_OK);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_t *dev = spi_find_dev(hspi);

    if ((dev == NULL) || (dev->busy == 0U) || (dev->aborting != 0U))
    {
        return;
    }
    spi_finish(dev, spi_map_hal(HAL_ERROR, hspi));   //自动区分 DMA 错误 / 其它错误
}