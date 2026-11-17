//
// Created by DQJ on 2026/11/17.
//

#include "bsp_spi.h"
#include "bsp_spi.h"
#include "stm32h7xx_hal.h"
#include <string.h>

//================== 内核时钟(0 表示自动从 RCC 读取，读不到时手动填写) ==================
#ifndef BSP_SPI123_KERNEL_CLK_HZ
#define BSP_SPI123_KERNEL_CLK_HZ   0UL     //SPI1/2/3 共用时钟源
#endif
#ifndef BSP_SPI45_KERNEL_CLK_HZ
#define BSP_SPI45_KERNEL_CLK_HZ    0UL     //SPI4/5 共用时钟源
#endif
#ifndef BSP_SPI6_KERNEL_CLK_HZ
#define BSP_SPI6_KERNEL_CLK_HZ     0UL
#endif

//================== SRAM1 段 ==================
//H723 的 SRAM1 地址为 0x30000000~0x30003FFF(16KB)，属于 D2 域，DMA1/DMA2 可以访问
//DTCM(0x20000000) 不能被 DMA1/DMA2 访问，所以 DMA 缓冲区必须放在 SRAM1
#define BSP_SRAM1_START   0x30000000UL
#define BSP_SRAM1_END     0x30004000UL

#if defined(__ICCARM__)
  #define BSP_SRAM1_ATTR  _Pragma("location=\".sram1\"") _Pragma("data_alignment=32")
  #define BSP_SRAM1_TAIL
#else   //GCC / ARMCLANG / ARMCC
  #define BSP_SRAM1_ATTR
  #define BSP_SRAM1_TAIL  __attribute__((section(".sram1"), aligned(32)))
#endif

//编译期检查：已使能的 DMA 缓冲区总和 <= 16KB
#define BSP_SRAM1_USED  ( (BSP_SPI1_ENABLE ? (BSP_SPI1_TX_BUF_SIZE + BSP_SPI1_RX_BUF_SIZE) : 0U) \
                        + (BSP_SPI2_ENABLE ? (BSP_SPI2_TX_BUF_SIZE + BSP_SPI2_RX_BUF_SIZE) : 0U) \
                        + (BSP_SPI3_ENABLE ? (BSP_SPI3_TX_BUF_SIZE + BSP_SPI3_RX_BUF_SIZE) : 0U) \
                        + (BSP_SPI4_ENABLE ? (BSP_SPI4_TX_BUF_SIZE + BSP_SPI4_RX_BUF_SIZE) : 0U) \
                        + (BSP_SPI5_ENABLE ? (BSP_SPI5_TX_BUF_SIZE + BSP_SPI5_RX_BUF_SIZE) : 0U) )
typedef char bsp_sram1_size_check[(BSP_SRAM1_USED <= (16U * 1024U)) ? 1 : -1];

//================== 操作函数表 ==================
static const bsp_spi_ops_t s_spi_ops =
{
    .init       = BSP_SPI_Init,
    .deinit     = BSP_SPI_DeInit,
    .transmit   = BSP_SPI_Transmit,
    .receive    = BSP_SPI_Receive,
    .poll       = BSP_SPI_Poll,
    .is_busy    = BSP_SPI_IsBusy,
    .get_tx_buf = BSP_SPI_GetTxBuf,
};

//设备对象初始化宏
#define BSP_SPI_DEV(_id, _mode, _h, _tx, _txs, _rx, _rxs)                  \
    { .ops = &s_spi_ops, .id = (_id), .mode = (_mode), .handle = (_h),     \
      .tx_buf = (_tx), .tx_buf_size = (_txs), .rx_buf = (_rx), .rx_buf_size = (_rxs) }

//================== 各路 SPI 的句柄和缓冲区(未使能的不引用 hspiX，也不占内存) ==================
#if BSP_SPI1_ENABLE
extern SPI_HandleTypeDef hspi1;
BSP_SRAM1_ATTR static uint8_t s_spi1_tx_buf[BSP_SPI1_TX_BUF_SIZE] BSP_SRAM1_TAIL;
BSP_SRAM1_ATTR static uint8_t s_spi1_rx_buf[BSP_SPI1_RX_BUF_SIZE] BSP_SRAM1_TAIL;
#define BSP_SPI1_DEV BSP_SPI_DEV(BSP_SPI_1, BSP_SPI_XFER_DMA, &hspi1, s_spi1_tx_buf, BSP_SPI1_TX_BUF_SIZE, s_spi1_rx_buf, BSP_SPI1_RX_BUF_SIZE)
#else
#define BSP_SPI1_DEV BSP_SPI_DEV(BSP_SPI_1, BSP_SPI_XFER_DMA, NULL, NULL, 0U, NULL, 0U)
#endif

#if BSP_SPI2_ENABLE
extern SPI_HandleTypeDef hspi2;
BSP_SRAM1_ATTR static uint8_t s_spi2_tx_buf[BSP_SPI2_TX_BUF_SIZE] BSP_SRAM1_TAIL;
BSP_SRAM1_ATTR static uint8_t s_spi2_rx_buf[BSP_SPI2_RX_BUF_SIZE] BSP_SRAM1_TAIL;
#define BSP_SPI2_DEV BSP_SPI_DEV(BSP_SPI_2, BSP_SPI_XFER_DMA, &hspi2, s_spi2_tx_buf, BSP_SPI2_TX_BUF_SIZE, s_spi2_rx_buf, BSP_SPI2_RX_BUF_SIZE)
#else
#define BSP_SPI2_DEV BSP_SPI_DEV(BSP_SPI_2, BSP_SPI_XFER_DMA, NULL, NULL, 0U, NULL, 0U)
#endif

#if BSP_SPI3_ENABLE
extern SPI_HandleTypeDef hspi3;
BSP_SRAM1_ATTR static uint8_t s_spi3_tx_buf[BSP_SPI3_TX_BUF_SIZE] BSP_SRAM1_TAIL;
BSP_SRAM1_ATTR static uint8_t s_spi3_rx_buf[BSP_SPI3_RX_BUF_SIZE] BSP_SRAM1_TAIL;
#define BSP_SPI3_DEV BSP_SPI_DEV(BSP_SPI_3, BSP_SPI_XFER_DMA, &hspi3, s_spi3_tx_buf, BSP_SPI3_TX_BUF_SIZE, s_spi3_rx_buf, BSP_SPI3_RX_BUF_SIZE)
#else
#define BSP_SPI3_DEV BSP_SPI_DEV(BSP_SPI_3, BSP_SPI_XFER_DMA, NULL, NULL, 0U, NULL, 0U)
#endif

#if BSP_SPI4_ENABLE
extern SPI_HandleTypeDef hspi4;
BSP_SRAM1_ATTR static uint8_t s_spi4_tx_buf[BSP_SPI4_TX_BUF_SIZE] BSP_SRAM1_TAIL;
BSP_SRAM1_ATTR static uint8_t s_spi4_rx_buf[BSP_SPI4_RX_BUF_SIZE] BSP_SRAM1_TAIL;
#define BSP_SPI4_DEV BSP_SPI_DEV(BSP_SPI_4, BSP_SPI_XFER_DMA, &hspi4, s_spi4_tx_buf, BSP_SPI4_TX_BUF_SIZE, s_spi4_rx_buf, BSP_SPI4_RX_BUF_SIZE)
#else
#define BSP_SPI4_DEV BSP_SPI_DEV(BSP_SPI_4, BSP_SPI_XFER_DMA, NULL, NULL, 0U, NULL, 0U)
#endif

#if BSP_SPI5_ENABLE
extern SPI_HandleTypeDef hspi5;
BSP_SRAM1_ATTR static uint8_t s_spi5_tx_buf[BSP_SPI5_TX_BUF_SIZE] BSP_SRAM1_TAIL;
BSP_SRAM1_ATTR static uint8_t s_spi5_rx_buf[BSP_SPI5_RX_BUF_SIZE] BSP_SRAM1_TAIL;
#define BSP_SPI5_DEV BSP_SPI_DEV(BSP_SPI_5, BSP_SPI_XFER_DMA, &hspi5, s_spi5_tx_buf, BSP_SPI5_TX_BUF_SIZE, s_spi5_rx_buf, BSP_SPI5_RX_BUF_SIZE)
#else
#define BSP_SPI5_DEV BSP_SPI_DEV(BSP_SPI_5, BSP_SPI_XFER_DMA, NULL, NULL, 0U, NULL, 0U)
#endif

#if BSP_SPI6_ENABLE
extern SPI_HandleTypeDef hspi6;
//SPI6 在 D3 域，DMA 只能用 BDMA 且缓冲区要放 SRAM4；这里用阻塞方式，放普通 RAM 即可
static uint8_t s_spi6_tx_buf[BSP_SPI6_TX_BUF_SIZE];
static uint8_t s_spi6_rx_buf[BSP_SPI6_RX_BUF_SIZE];
#define BSP_SPI6_DEV BSP_SPI_DEV(BSP_SPI_6, BSP_SPI_XFER_BLOCKING, &hspi6, s_spi6_tx_buf, BSP_SPI6_TX_BUF_SIZE, s_spi6_rx_buf, BSP_SPI6_RX_BUF_SIZE)
#else
#define BSP_SPI6_DEV BSP_SPI_DEV(BSP_SPI_6, BSP_SPI_XFER_BLOCKING, NULL, NULL, 0U, NULL, 0U)
#endif

//所有 SPI 共用同一个结构体类型
static bsp_spi_t s_spi_dev[BSP_SPI_NUM] =
{
    BSP_SPI1_DEV, BSP_SPI2_DEV, BSP_SPI3_DEV,
    BSP_SPI4_DEV, BSP_SPI5_DEV, BSP_SPI6_DEV
};

//================== 内部工具函数 ==================
//临界区，保证 busy 的"检查 + 置位"是原子操作
static inline uint32_t spi_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void spi_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

//通过函数指针通知上层
static void spi_notify(bsp_spi_t *dev, bsp_spi_event_t evt, bsp_spi_status_t st)
{
    if (dev->callback != NULL)
    {
        dev->callback(dev, evt, st, dev->user_arg);
    }
}

//尝试占用：忙则立即返回 BUSY，不等待
static bsp_spi_status_t spi_try_lock(bsp_spi_t *dev)
{
    uint32_t p = spi_enter_critical();
    if (dev->busy)
    {
        spi_exit_critical(p);
        return BSP_SPI_BUSY;
    }
    dev->busy       = 1U;
    dev->aborting   = 0U;
    dev->start_tick = HAL_GetTick();
    spi_exit_critical(p);
    return BSP_SPI_OK;
}

//HAL 返回值 -> 本层状态码
static bsp_spi_status_t spi_map_hal(HAL_StatusTypeDef hs, SPI_HandleTypeDef *h)
{
    if (hs == HAL_OK)      { return BSP_SPI_OK; }
    if (hs == HAL_BUSY)    { return BSP_SPI_BUSY; }
    if (hs == HAL_TIMEOUT) { return BSP_SPI_TIMEOUT; }
    //HAL_ERROR：根据 ErrorCode 区分是不是 DMA 错误
    if ((h->ErrorCode & HAL_SPI_ERROR_DMA) != 0U)
    {
        return BSP_SPI_DMA_ERROR;
    }
    return BSP_SPI_HAL_ERROR;
}

//DMA 超时检测：超时后中止传输、释放 busy、通知上层
static bsp_spi_status_t spi_check_timeout(bsp_spi_t *dev)
{
    uint32_t p;

    if ((dev->mode != BSP_SPI_XFER_DMA) || (dev->busy == 0U))
    {
        return BSP_SPI_OK;
    }
    if ((HAL_GetTick() - dev->start_tick) < dev->timeout_ms)
    {
        return BSP_SPI_BUSY;
    }

    p = spi_enter_critical();
    if (dev->busy == 0U)              //恰好在中断里完成了
    {
        spi_exit_critical(p);
        return BSP_SPI_OK;
    }
    dev->aborting = 1U;               //让回调忽略中止过程中的事件
    spi_exit_critical(p);

    (void)HAL_SPI_Abort((SPI_HandleTypeDef *)dev->handle);

    dev->last_status = BSP_SPI_TIMEOUT;
    dev->err_count++;
    dev->aborting = 0U;
    dev->busy     = 0U;
    spi_notify(dev, BSP_SPI_EVT_ERROR, BSP_SPI_TIMEOUT);
    return BSP_SPI_TIMEOUT;
}

//选择最接近目标速率的分频
static uint32_t spi_calc_prescaler(uint32_t clk, uint32_t target, uint32_t *actual)
{
    static const struct { uint16_t div; uint32_t reg; } tbl[] =
    {
        {  2U, SPI_BAUDRATEPRESCALER_2   }, {  4U, SPI_BAUDRATEPRESCALER_4   },
        {  8U, SPI_BAUDRATEPRESCALER_8   }, { 16U, SPI_BAUDRATEPRESCALER_16  },
        { 32U, SPI_BAUDRATEPRESCALER_32  }, { 64U, SPI_BAUDRATEPRESCALER_64  },
        {128U, SPI_BAUDRATEPRESCALER_128 }, {256U, SPI_BAUDRATEPRESCALER_256 },
    };
    uint32_t i, best = 0U, best_err = 0xFFFFFFFFUL;

    for (i = 0U; i < (sizeof(tbl) / sizeof(tbl[0])); i++)
    {
        uint32_t f   = clk / tbl[i].div;
        uint32_t err = (f > target) ? (f - target) : (target - f);
        if (err < best_err)
        {
            best_err = err;
            best     = i;
        }
    }
    *actual = clk / tbl[best].div;
    return tbl[best].reg;
}

//检查缓冲区是否真的在 SRAM1(链接脚本没生效时，DMA 会出错)
static uint8_t spi_buf_in_sram1(const uint8_t *buf, uint32_t size)
{
    uint32_t addr = (uint32_t)buf;
    if ((buf == NULL) || (size == 0U))
    {
        return 1U;
    }
    return ((addr >= BSP_SRAM1_START) && ((addr + size) <= BSP_SRAM1_END)) ? 1U : 0U;
}

//通过 HAL 句柄找设备对象(HAL 回调用)
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

//DMA 发送(SPI1~SPI5)
static bsp_spi_status_t spi_tx_dma(bsp_spi_t *dev, const uint8_t *data, uint16_t len)
{
    SPI_HandleTypeDef *h = (SPI_HandleTypeDef *)dev->handle;
    HAL_StatusTypeDef  hs;
    bsp_spi_status_t   st;

    if (len > dev->tx_buf_size)
    {
        dev->busy = 0U;
        return BSP_SPI_PARAM_ERROR;
    }
    if (data != dev->tx_buf)          //上层已直接写好缓冲区时(零拷贝)不再拷贝
    {
        memcpy(dev->tx_buf, data, len);
    }
    //未开 DCache，无需 Clean；以后开启 DCache 要在这里 SCB_CleanDCache_by_Addr，或用 MPU 把 SRAM1 设为不缓存

    hs = HAL_SPI_Transmit_DMA(h, dev->tx_buf, len);
    if (hs != HAL_OK)
    {
        st = spi_map_hal(hs, h);
        dev->busy        = 0U;        //启动失败，立即释放
        dev->last_status = st;
        dev->err_count++;
        return st;
    }
    return BSP_SPI_OK;                //busy 在回调中释放
}

//阻塞发送(SPI6)
static bsp_spi_status_t spi_tx_blocking(bsp_spi_t *dev, const uint8_t *data, uint16_t len)
{
    SPI_HandleTypeDef *h = (SPI_HandleTypeDef *)dev->handle;
    HAL_StatusTypeDef  hs;
    bsp_spi_status_t   st;

    hs = HAL_SPI_Transmit(h, (uint8_t *)data, len, dev->timeout_ms);
    st = spi_map_hal(hs, h);

    dev->busy        = 0U;
    dev->last_status = st;
    if (st == BSP_SPI_OK)
    {
        spi_notify(dev, BSP_SPI_EVT_TX_DONE, BSP_SPI_OK);   //与 DMA 方式保持一致
    }
    else
    {
        dev->err_count++;
        spi_notify(dev, BSP_SPI_EVT_ERROR, st);
    }
    return st;
}

//================== 对外接口实现 ==================
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
    uint32_t kernel_clk;
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

    //用 if / else 确认 SPI 编号：外设实例、传输方式、内核时钟
    if (dev->id == BSP_SPI_1)
    {
        h->Instance = SPI1;
        dev->mode   = BSP_SPI_XFER_DMA;
        kernel_clk  = (BSP_SPI123_KERNEL_CLK_HZ != 0UL) ? BSP_SPI123_KERNEL_CLK_HZ
                    : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
    }
    else if (dev->id == BSP_SPI_2)
    {
        h->Instance = SPI2;
        dev->mode   = BSP_SPI_XFER_DMA;
        kernel_clk  = (BSP_SPI123_KERNEL_CLK_HZ != 0UL) ? BSP_SPI123_KERNEL_CLK_HZ
                    : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
    }
    else if (dev->id == BSP_SPI_3)
    {
        h->Instance = SPI3;
        dev->mode   = BSP_SPI_XFER_DMA;
        kernel_clk  = (BSP_SPI123_KERNEL_CLK_HZ != 0UL) ? BSP_SPI123_KERNEL_CLK_HZ
                    : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
    }
    else if (dev->id == BSP_SPI_4)
    {
        h->Instance = SPI4;
        dev->mode   = BSP_SPI_XFER_DMA;
        kernel_clk  = (BSP_SPI45_KERNEL_CLK_HZ != 0UL) ? BSP_SPI45_KERNEL_CLK_HZ
                    : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
    }
    else if (dev->id == BSP_SPI_5)
    {
        h->Instance = SPI5;
        dev->mode   = BSP_SPI_XFER_DMA;
        kernel_clk  = (BSP_SPI45_KERNEL_CLK_HZ != 0UL) ? BSP_SPI45_KERNEL_CLK_HZ
                    : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
    }
    else if (dev->id == BSP_SPI_6)
    {
        h->Instance = SPI6;
        dev->mode   = BSP_SPI_XFER_BLOCKING;   //SPI6 使用阻塞发送
        kernel_clk  = (BSP_SPI6_KERNEL_CLK_HZ != 0UL) ? BSP_SPI6_KERNEL_CLK_HZ
                    : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
    }
    else
    {
        return BSP_SPI_PARAM_ERROR;
    }

    if (kernel_clk == 0U)
    {
        return BSP_SPI_PARAM_ERROR;    //读不到时钟时，手动定义 BSP_SPIxxx_KERNEL_CLK_HZ
    }

    //DMA 方式：打开 SRAM1 时钟，并检查缓冲区是否在 SRAM1
    if (dev->mode == BSP_SPI_XFER_DMA)
    {
#ifdef __HAL_RCC_D2SRAM1_CLK_ENABLE
        __HAL_RCC_D2SRAM1_CLK_ENABLE();
#endif
        if ((spi_buf_in_sram1(dev->tx_buf, dev->tx_buf_size) == 0U) ||
            (spi_buf_in_sram1(dev->rx_buf, dev->rx_buf_size) == 0U))
        {
            return BSP_SPI_PARAM_ERROR;
        }
    }

    //保存配置
    target          = ((cfg != NULL) && (cfg->baudrate_hz != 0U)) ? cfg->baudrate_hz : BSP_SPI_DEFAULT_BAUD_HZ;
    dev->timeout_ms = ((cfg != NULL) && (cfg->timeout_ms  != 0U)) ? cfg->timeout_ms  : BSP_SPI_DEFAULT_TIMEOUT_MS;
    dev->callback   = (cfg != NULL) ? cfg->callback : NULL;
    dev->user_arg   = (cfg != NULL) ? cfg->user_arg : NULL;

    //主机、只发送、8bit、MSB 先发、Mode0
    h->Init.Mode                       = SPI_MODE_MASTER;
    h->Init.Direction                  = SPI_DIRECTION_2LINES_TXONLY;
    h->Init.DataSize                   = SPI_DATASIZE_8BIT;
    h->Init.CLKPolarity                = SPI_POLARITY_LOW;
    h->Init.CLKPhase                   = SPI_PHASE_1EDGE;
    h->Init.NSS                        = SPI_NSS_SOFT;
    h->Init.BaudRatePrescaler          = spi_calc_prescaler(kernel_clk, target, &actual);
    h->Init.FirstBit                   = SPI_FIRSTBIT_MSB;
    h->Init.TIMode                     = SPI_TIMODE_DISABLE;
    h->Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
    h->Init.CRCPolynomial              = 0x7U;
    h->Init.NSSPMode                   = SPI_NSS_PULSE_DISABLE;
    h->Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    h->Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    h->Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    h->Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    h->Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    h->Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;  //字节间无空隙，WS2812 必须
    h->Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    h->Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_ENABLE;        //空闲时保持 IO 电平，不浮空
    h->Init.IOSwap                     = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(h) != HAL_OK)
    {
        return BSP_SPI_HAL_ERROR;
    }

    //DMA 方式要确认 CubeMX 已经把 TX DMA 链接到句柄上
    if ((dev->mode == BSP_SPI_XFER_DMA) && (h->hdmatx == NULL))
    {
        return BSP_SPI_DMA_ERROR;
    }

    if (dev->tx_buf != NULL) { memset(dev->tx_buf, 0, dev->tx_buf_size); }   //SRAM1 是 NOLOAD 段，手动清零
    if (dev->rx_buf != NULL) { memset(dev->rx_buf, 0, dev->rx_buf_size); }

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
    //参数检查
    if ((dev == NULL) || (dev->handle == NULL) || (data == NULL) ||
        (len == 0U) || (dev->initialized == 0U))
    {
        return BSP_SPI_PARAM_ERROR;
    }

    //忙：立即返回 BUSY；超过 timeout_ms 则中止上一次传输并返回 TIMEOUT
    if (spi_try_lock(dev) != BSP_SPI_OK)
    {
        return (spi_check_timeout(dev) == BSP_SPI_TIMEOUT) ? BSP_SPI_TIMEOUT : BSP_SPI_BUSY;
    }

    //用 if / else 确认 SPI 编号，选择发送方式
    if ((dev->id == BSP_SPI_1) || (dev->id == BSP_SPI_2) || (dev->id == BSP_SPI_3) ||
        (dev->id == BSP_SPI_4) || (dev->id == BSP_SPI_5))
    {
        return spi_tx_dma(dev, data, len);        //SPI1~SPI5：DMA
    }
    else if (dev->id == BSP_SPI_6)
    {
        return spi_tx_blocking(dev, data, len);   //SPI6：阻塞
    }
    else
    {
        dev->busy = 0U;
        return BSP_SPI_PARAM_ERROR;
    }
}

//接收接口预留：目前只实现 Master TX only
bsp_spi_status_t BSP_SPI_Receive(bsp_spi_t *dev, uint8_t *data, uint16_t len)
{
    if ((dev == NULL) || (dev->handle == NULL) || (data == NULL) || (len == 0U))
    {
        return BSP_SPI_PARAM_ERROR;
    }

    if ((dev->id == BSP_SPI_1) || (dev->id == BSP_SPI_2) || (dev->id == BSP_SPI_3) ||
        (dev->id == BSP_SPI_4) || (dev->id == BSP_SPI_5))
    {
        //TODO：Direction 改为 SPI_DIRECTION_2LINES / 2LINES_RXONLY
        //TODO：HAL_SPI_Receive_DMA(h, dev->rx_buf, len)，在 RxCpltCallback 中拷贝到 data 并释放 busy
        return BSP_SPI_NOT_SUPPORT;
    }
    else if (dev->id == BSP_SPI_6)
    {
        //TODO：HAL_SPI_Receive(h, data, len, dev->timeout_ms)
        return BSP_SPI_NOT_SUPPORT;
    }
    else
    {
        return BSP_SPI_PARAM_ERROR;
    }
}

//主循环中周期调用：检测 DMA 是否超过 timeout_ms 未完成
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

//零拷贝：上层直接写发送缓冲区，再调用 transmit(dev, buf, len)
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

    if ((dev == NULL) || (dev->mode != BSP_SPI_XFER_DMA) ||
        (dev->busy == 0U) || (dev->aborting != 0U))
    {
        return;
    }
    dev->last_status = BSP_SPI_OK;
    dev->busy        = 0U;                               //先释放 busy，上层在回调里就能发下一帧
    spi_notify(dev, BSP_SPI_EVT_TX_DONE, BSP_SPI_OK);    //函数指针通知上层
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_t *dev = spi_find_dev(hspi);
    bsp_spi_status_t st;

    if ((dev == NULL) || (dev->mode != BSP_SPI_XFER_DMA) ||
        (dev->busy == 0U) || (dev->aborting != 0U))
    {
        return;
    }
    st = ((hspi->ErrorCode & HAL_SPI_ERROR_DMA) != 0U) ? BSP_SPI_DMA_ERROR : BSP_SPI_HAL_ERROR;

    dev->last_status = st;
    dev->err_count++;
    dev->busy = 0U;
    spi_notify(dev, BSP_SPI_EVT_ERROR, st);
}

//接收完成回调预留
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) { }