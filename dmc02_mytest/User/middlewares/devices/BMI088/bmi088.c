/**
 * @file    bmi088.c
 * @brief   BMI088 六轴 IMU 驱动实现（与 bmi088.h 配套）
 * @version 1.0
 * @note    源文件编码: UTF-8
 *
 * 实现要点：
 *   1. 加速度计和陀螺仪是两颗独立的 die，寄存器地址空间各自独立，
 *      因此内部用 accel_read/accel_write 与 gyro_read/gyro_write 分开访问。
 *   2. 加速度计上电后没有额外的初始化序列（0x59/0x2A 在 BMI088 上属保留区）：
 *      软复位 -> 等 50ms -> PWR_CONF=0x00 -> PWR_CTRL=0x04 -> 等 50ms -> 写配置。
 *   3. 陀螺仪不支持多字节连续写，内部拆成单字节写；连续两次写之间至少间隔 2us。
 *   4. 加速度计 SPI 读会多出一个 dummy 字节，驱动读 len+1 字节后丢弃第 1 字节。
 */

#include "bmi088.h"

/*=========================================================================
 * 内部常量
 *=========================================================================*/

/* 单次传输最大字节数（驱动内部使用，够覆盖最长的一次读） */
#define BMI088_MAX_XFER_LEN             UINT16_C(16)

/* 各步骤等待时间（微秒） */
#define BMI088_POWER_ON_DELAY_US        UINT32_C(5000)   /* 上电稳定 */
#define BMI088_ACCEL_RESET_DELAY_US     UINT32_C(50000)  /* 加速度计软复位后等待（手册 1ms，留裕量） */
#define BMI088_GYRO_RESET_DELAY_US      UINT32_C(30000)  /* 陀螺仪软复位后等待 30ms */
#define BMI088_ACCEL_PWR_DELAY_US       UINT32_C(5000)   /* 加速度计 PWR_CONF 后等待 */
#define BMI088_ACCEL_ENABLE_DELAY_US    UINT32_C(50000)  /* 0x7D=0x04 使能后等待 50ms（手册 Quick Start） */
#define BMI088_GYRO_PWR_DELAY_US        UINT32_C(30000)  /* 陀螺仪电源切换等待 30ms */
#define BMI088_ACCEL_CONF_DELAY_US      UINT32_C(40000)  /* 加速度计配置生效等待 */
#define BMI088_GYRO_CONF_DELAY_US       UINT32_C(10000)  /* 陀螺仪配置生效等待 */
#define BMI088_WRITE_GAP_US             UINT32_C(2)      /* 陀螺仪连续两次写之间的最小间隔 */

/*=========================================================================
 * 内部函数声明
 *=========================================================================*/
static int8_t bmi088_dev_check(const struct bmi088_dev *dev);
static int8_t bmi088_accel_read(struct bmi088_dev *dev, uint8_t reg, uint8_t *data, uint16_t len);
static int8_t bmi088_accel_write(struct bmi088_dev *dev, uint8_t reg, const uint8_t *data, uint16_t len);
static int8_t bmi088_gyro_read(struct bmi088_dev *dev, uint8_t reg, uint8_t *data, uint16_t len);
static int8_t bmi088_gyro_write(struct bmi088_dev *dev, uint8_t reg, const uint8_t *data, uint16_t len);
static int8_t bmi088_read_accel_with_time(struct bmi088_dev *dev,
                                          struct bmi088_sensor_data *accel,
                                          uint32_t *sensor_time);

/*=========================================================================
 * 内部函数实现
 *=========================================================================*/

/*!
 * @brief 检查设备实例和回调是否可用
 */
static int8_t bmi088_dev_check(const struct bmi088_dev *dev)
{
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        return BMI088_E_NULL_PTR;
    }

    if ((dev->intf != BMI088_I2C) && (dev->intf != BMI088_SPI))
    {
        return BMI088_E_INVALID_PARAM;
    }

    return BMI088_OK;
}

/*!
 * @brief 读加速度计寄存器
 *
 * SPI 模式下，加速度计在地址字节之后会先输出 1 个 dummy 字节，
 * 所以这里多读 1 字节再丢弃。
 */
static int8_t bmi088_accel_read(struct bmi088_dev *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((data == NULL) || (len == 0u) || (len > BMI088_MAX_XFER_LEN))
    {
        return BMI088_E_INVALID_PARAM;
    }

    if (dev->intf == BMI088_SPI)
    {
        uint8_t buf[BMI088_MAX_XFER_LEN + 1u];

        if (dev->read(dev->accel_id, (uint8_t)(reg | BMI088_SPI_READ_BIT), buf, (uint16_t)(len + 1u)) != 0)
        {
            return BMI088_E_COM_FAIL;
        }

        for (i = 0u; i < len; i++)
        {
            data[i] = buf[i + 1u];
        }
    }
    else
    {
        if (dev->read(dev->accel_id, reg, data, len) != 0)
        {
            return BMI088_E_COM_FAIL;
        }
    }

    return BMI088_OK;
}

/*!
 * @brief 写加速度计寄存器
 */
static int8_t bmi088_accel_write(struct bmi088_dev *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t addr;

    if ((data == NULL) || (len == 0u) || (len > BMI088_MAX_XFER_LEN))
    {
        return BMI088_E_INVALID_PARAM;
    }

    addr = (dev->intf == BMI088_SPI) ? (uint8_t)(reg & BMI088_SPI_WRITE_MASK) : reg;

    if (dev->write(dev->accel_id, addr, data, len) != 0)
    {
        return BMI088_E_COM_FAIL;
    }

    /* 手册要求正常模式下两次写之间至少 2us（SPI 高速时尤其要留余量）。
     * suspend 模式下要求 1000us，由各调用处的毫秒级延时保证。 */
    dev->delay_us(BMI088_WRITE_GAP_US);

    return BMI088_OK;
}

/*!
 * @brief 读陀螺仪寄存器（SPI 无 dummy 字节）
 */
static int8_t bmi088_gyro_read(struct bmi088_dev *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr;

    if ((data == NULL) || (len == 0u) || (len > BMI088_MAX_XFER_LEN))
    {
        return BMI088_E_INVALID_PARAM;
    }

    addr = (dev->intf == BMI088_SPI) ? (uint8_t)(reg | BMI088_SPI_READ_BIT) : reg;

    if (dev->read(dev->gyro_id, addr, data, len) != 0)
    {
        return BMI088_E_COM_FAIL;
    }

    return BMI088_OK;
}

/*!
 * @brief 写陀螺仪寄存器
 *
 * 陀螺仪不支持多字节连续写（burst write），这里拆成逐字节写，
 * 且每字节之间留 2us 间隔（手册要求正常模式下两次写间隔 >= 2us）。
 */
static int8_t bmi088_gyro_write(struct bmi088_dev *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint8_t addr;

    if ((data == NULL) || (len == 0u) || (len > BMI088_MAX_XFER_LEN))
    {
        return BMI088_E_INVALID_PARAM;
    }

    for (i = 0u; i < len; i++)
    {
        addr = (dev->intf == BMI088_SPI) ?
               (uint8_t)((reg + i) & BMI088_SPI_WRITE_MASK) : (uint8_t)(reg + i);

        if (dev->write(dev->gyro_id, addr, &data[i], 1u) != 0)
        {
            return BMI088_E_COM_FAIL;
        }

        /* 手册要求连续两次写之间至少 2us（SPI 高速时尤其要留余量） */
        dev->delay_us(BMI088_WRITE_GAP_US);
    }

    return BMI088_OK;
}

/*!
 * @brief 一次突发读出加速度三轴 + Sensor time（0x12~0x1A，共 9 字节）
 *
 * 放在同一次总线传输里，保证加速度采样值和时间戳来自同一时刻，
 * 对姿态融合（尤其是积分的时标）很有用。
 */
static int8_t bmi088_read_accel_with_time(struct bmi088_dev *dev,
                                          struct bmi088_sensor_data *accel,
                                          uint32_t *sensor_time)
{
    int8_t rslt;
    uint8_t data[9];

    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_X_LSB, data, 9u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    accel->x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    accel->y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    accel->z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    *sensor_time = ((uint32_t)data[8] << 16) | ((uint32_t)data[7] << 8) | (uint32_t)data[6];

    return BMI088_OK;
}

/*=========================================================================
 * 公共函数实现
 *=========================================================================*/

int8_t bmi088_soft_reset(struct bmi088_dev *dev)
{
    int8_t rslt;
    uint8_t cmd;
    uint8_t dummy;

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* ---- 加速度计软复位 ---- */
    cmd = BMI088_SOFT_RESET_CMD;
    rslt = bmi088_accel_write(dev, BMI088_REG_ACCEL_SOFTRESET, &cmd, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->delay_us(BMI088_ACCEL_RESET_DELAY_US);

    /* SPI 模式：复位后第一次访问先读一次丢弃（软复位不会退出 SPI 模式，此步非必需但无害） */
    if (dev->intf == BMI088_SPI)
    {
        (void)bmi088_accel_read(dev, BMI088_REG_ACCEL_CHIP_ID, &dummy, 1u);
    }

    /* ---- 陀螺仪软复位 ---- */
    cmd = BMI088_SOFT_RESET_CMD;
    rslt = bmi088_gyro_write(dev, BMI088_REG_GYRO_SOFTRESET, &cmd, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->delay_us(BMI088_GYRO_RESET_DELAY_US);

    /* 复位后芯片量程回到默认值：加速度计 ±6g，陀螺仪 ±2000dps。
     * 同步到 dev，避免只调用本函数、不重新配置时用旧量程换算。 */
    dev->accel_range = BMI088_ACCEL_RANGE_6G;
    dev->gyro_range = BMI088_GYRO_RANGE_2000DPS;

    return BMI088_OK;
}

int8_t bmi088_init(struct bmi088_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id;

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->accel_chip_id = 0u;
    dev->gyro_chip_id = 0u;

    /* 先清掉配置字段，避免调用者结构体里的随机值参与换算 */
    dev->accel_odr = 0u;
    dev->accel_bw = 0u;
    dev->accel_range = 0u;
    dev->gyro_odr_bw = 0u;
    dev->gyro_range = 0u;

    /* 等电源稳定 */
    dev->delay_us(BMI088_POWER_ON_DELAY_US);

    /* ---- 读加速度计芯片 ID ---- */
    if (dev->intf == BMI088_SPI)
    {
        /* 上电后 SPI 第一次读通常是无效数据，先读一次丢弃 */
        (void)bmi088_accel_read(dev, BMI088_REG_ACCEL_CHIP_ID, &chip_id, 1u);
    }

    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_CHIP_ID, &chip_id, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->accel_chip_id = chip_id;
    if (chip_id != BMI088_ACCEL_CHIP_ID)
    {
        return BMI088_E_DEV_NOT_FOUND;
    }

    /* ---- 读陀螺仪芯片 ID ---- */
    rslt = bmi088_gyro_read(dev, BMI088_REG_GYRO_CHIP_ID, &chip_id, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->gyro_chip_id = chip_id;
    if (chip_id != BMI088_GYRO_CHIP_ID)
    {
        return BMI088_E_DEV_NOT_FOUND;
    }

    /* ---- 软复位 ---- */
    rslt = bmi088_soft_reset(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* ---- 默认配置：加速度 100Hz / +-6g，陀螺仪 400Hz / +-500dps ---- */
    rslt = bmi088_set_accel_config(dev,
                                   BMI088_ACCEL_ODR_100HZ,
                                   BMI088_ACCEL_BW_NORMAL,
                                   BMI088_ACCEL_RANGE_6G);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_set_gyro_config(dev, BMI088_GYRO_BW_47_ODR_400HZ, BMI088_GYRO_RANGE_500DPS);

    return rslt;
}

int8_t bmi088_set_accel_config(struct bmi088_dev *dev, uint8_t odr, uint8_t bw, uint8_t range)
{
    int8_t rslt;
    uint8_t data;

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 参数检查：ODR 0x05~0x0C，BW 0x08~0x0A，量程 0~3 */
    if ((odr < BMI088_ACCEL_ODR_12_5HZ) || (odr > BMI088_ACCEL_ODR_1600HZ) ||
        (bw < BMI088_ACCEL_BW_OSR4) || (bw > BMI088_ACCEL_BW_NORMAL) ||
        (range > BMI088_ACCEL_RANGE_24G))
    {
        return BMI088_E_INVALID_PARAM;
    }

    /* 1. 退出省电模式并使能加速度计 */
    data = BMI088_ACCEL_PM_ACTIVE; /* 0x7C = 0x00 */
    rslt = bmi088_accel_write(dev, BMI088_REG_ACCEL_PWR_CONF, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->delay_us(BMI088_ACCEL_PWR_DELAY_US);

    data = BMI088_ACCEL_POWER_ENABLE; /* 0x7D = 0x04 */
    rslt = bmi088_accel_write(dev, BMI088_REG_ACCEL_PWR_CTRL, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 手册 Quick Start：使能后要等 50ms 才能拿到有效数据 */
    dev->delay_us(BMI088_ACCEL_ENABLE_DELAY_US);

    /* 2. ODR（低 4 位）+ 带宽（高 4 位） */
    data = (uint8_t)((uint8_t)(bw << 4) | (odr & 0x0Fu));
    rslt = bmi088_accel_write(dev, BMI088_REG_ACCEL_CONF, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 3. 量程 */
    data = (uint8_t)(range & 0x03u);
    rslt = bmi088_accel_write(dev, BMI088_REG_ACCEL_RANGE, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->delay_us(BMI088_ACCEL_CONF_DELAY_US);

    dev->accel_odr = odr;
    dev->accel_bw = bw;
    dev->accel_range = range;

    return BMI088_OK;
}

int8_t bmi088_set_gyro_config(struct bmi088_dev *dev, uint8_t odr_bw, uint8_t range)
{
    int8_t rslt;
    uint8_t data;

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* ODR/BW 0x00~0x07，量程 0~4 */
    if ((odr_bw > BMI088_GYRO_BW_32_ODR_100HZ) || (range > BMI088_GYRO_RANGE_125DPS))
    {
        return BMI088_E_INVALID_PARAM;
    }

    /* 1. 切到 normal 模式（挂起状态下不能改量程/带宽） */
    data = BMI088_GYRO_PM_NORMAL;
    rslt = bmi088_gyro_write(dev, BMI088_REG_GYRO_LPM1, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->delay_us(BMI088_GYRO_PWR_DELAY_US);

    /* 2. ODR / 带宽 */
    data = (uint8_t)(odr_bw & 0x0Fu);
    rslt = bmi088_gyro_write(dev, BMI088_REG_GYRO_BANDWIDTH, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 3. 量程 */
    data = (uint8_t)(range & 0x07u);
    rslt = bmi088_gyro_write(dev, BMI088_REG_GYRO_RANGE, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 4. 使能新数据中断（0x15 bit7）；状态位 0x0A.bit7 是否依赖该位，手册未明确，以实测为准 */
    data = BMI088_GYRO_DATA_EN;
    rslt = bmi088_gyro_write(dev, BMI088_REG_GYRO_INT_CTRL, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    dev->delay_us(BMI088_GYRO_CONF_DELAY_US);

    dev->gyro_odr_bw = odr_bw;
    dev->gyro_range = range;

    return BMI088_OK;
}

int8_t bmi088_read_accel(struct bmi088_dev *dev, struct bmi088_sensor_data *accel)
{
    int8_t rslt;
    uint8_t data[6];

    if (accel == NULL)
    {
        return BMI088_E_NULL_PTR;
    }

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_X_LSB, data, 6u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    accel->x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    accel->y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    accel->z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    return BMI088_OK;
}

int8_t bmi088_read_gyro(struct bmi088_dev *dev, struct bmi088_sensor_data *gyro)
{
    int8_t rslt;
    uint8_t data[6];

    if (gyro == NULL)
    {
        return BMI088_E_NULL_PTR;
    }

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_gyro_read(dev, BMI088_REG_GYRO_X_LSB, data, 6u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    gyro->x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    gyro->y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    gyro->z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    return BMI088_OK;
}

int8_t bmi088_read_temperature(struct bmi088_dev *dev, int32_t *temp_mc)
{
    int8_t rslt;
    uint8_t data[2];
    uint16_t raw;
    int16_t signed_raw;

    if (temp_mc == NULL)
    {
        return BMI088_E_NULL_PTR;
    }

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_TEMP_MSB, data, 2u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 手册 5.3.7：MSB = 0x80 表示本次温度值无效 */
    if (data[0] == 0x80u)
    {
        return BMI088_E_INVALID_DATA;
    }

    /* 11 位温度：MSB 占 8 位，LSB 的高 3 位有效 */
    raw = (uint16_t)(((uint16_t)data[0] << 3) | (data[1] >> 5));

    /* 11 位补码转有符号 */
    if (raw > 1023u)
    {
        signed_raw = (int16_t)((int16_t)raw - 2048);
    }
    else
    {
        signed_raw = (int16_t)raw;
    }

    /* 0.125°C/LSB，23°C 偏移，结果单位为 0.001°C */
    *temp_mc = ((int32_t)signed_raw * 125) + 23000;

    return BMI088_OK;
}

int8_t bmi088_read_sensor_time(struct bmi088_dev *dev, uint32_t *sensor_time)
{
    int8_t rslt;
    uint8_t data[3];

    if (sensor_time == NULL)
    {
        return BMI088_E_NULL_PTR;
    }

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_SENSORTIME_0, data, 3u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    *sensor_time = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];

    return BMI088_OK;
}

int8_t bmi088_read_error(struct bmi088_dev *dev, struct bmi088_err_reg *err)
{
    int8_t rslt;
    uint8_t data;

    if (err == NULL)
    {
        return BMI088_E_NULL_PTR;
    }

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_ERR, &data, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    err->fatal_err = (uint8_t)(data & BMI088_ERR_FATAL_MSK);
    err->err_code = (uint8_t)((data & BMI088_ERR_CODE_MSK) >> BMI088_ERR_CODE_POS);

    return BMI088_OK;
}

int8_t bmi088_read_status(struct bmi088_dev *dev, uint8_t *accel_status, uint8_t *gyro_status)
{
    int8_t rslt;
    uint8_t data;

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    if (accel_status != NULL)
    {
        rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_STATUS, &data, 1u);
        if (rslt != BMI088_OK)
        {
            return rslt;
        }

        *accel_status = data;
    }

    if (gyro_status != NULL)
    {
        rslt = bmi088_gyro_read(dev, BMI088_REG_GYRO_INT_STAT_1, &data, 1u);
        if (rslt != BMI088_OK)
        {
            return rslt;
        }

        *gyro_status = data;
    }

    return BMI088_OK;
}

int8_t bmi088_read_all(struct bmi088_dev *dev, struct bmi088_data *out)
{
    int8_t rslt;

    if (out == NULL)
    {
        return BMI088_E_NULL_PTR;
    }

    rslt = bmi088_dev_check(dev);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    out->accel.x = 0;
    out->accel.y = 0;
    out->accel.z = 0;
    out->gyro.x = 0;
    out->gyro.y = 0;
    out->gyro.z = 0;
    out->accel_ms2.x = 0.0f;
    out->accel_ms2.y = 0.0f;
    out->accel_ms2.z = 0.0f;
    out->gyro_dps.x = 0.0f;
    out->gyro_dps.y = 0.0f;
    out->gyro_dps.z = 0.0f;
    out->temperature_mc = 0;
    out->temperature_c = 0.0f;
    out->temperature_valid = 0u;
    out->sensor_time = 0u;
    out->sensor_time_s = 0.0f;
    out->accel_status = 0u;
    out->gyro_status = 0u;
    out->err.fatal_err = 0u;
    out->err.err_code = 0u;

    /* 状态位必须先取：加速度计 DRDY 在数据寄存器被读走后自动清零 */
    rslt = bmi088_accel_read(dev, BMI088_REG_ACCEL_STATUS, &out->accel_status, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_read_accel_with_time(dev, &out->accel, &out->sensor_time);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* 陀螺仪 DRDY 只有约 280~400us 的有效窗口，同样要在读数据前取 */
    rslt = bmi088_gyro_read(dev, BMI088_REG_GYRO_INT_STAT_1, &out->gyro_status, 1u);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    rslt = bmi088_read_gyro(dev, &out->gyro);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }
    
    rslt = bmi088_read_temperature(dev, &out->temperature_mc);
    if (rslt == BMI088_OK)
    {
        out->temperature_valid = 1u;
        out->temperature_c = (float)out->temperature_mc / 1000.0f;
    }
    else if (rslt == BMI088_E_INVALID_DATA)
    {
        out->temperature_mc = 0;
        out->temperature_c = 0.0f;
    }
    else
    {
        return rslt;
    }

    rslt = bmi088_read_error(dev, &out->err);
    if (rslt != BMI088_OK)
    {
        return rslt;
    }

    /* ---- 换算成物理量 ---- */
    out->accel_ms2.x = bmi088_accel_to_mps2(out->accel.x, dev->accel_range);
    out->accel_ms2.y = bmi088_accel_to_mps2(out->accel.y, dev->accel_range);
    out->accel_ms2.z = bmi088_accel_to_mps2(out->accel.z, dev->accel_range);

    out->gyro_dps.x = bmi088_gyro_to_dps(out->gyro.x, dev->gyro_range);
    out->gyro_dps.y = bmi088_gyro_to_dps(out->gyro.y, dev->gyro_range);
    out->gyro_dps.z = bmi088_gyro_to_dps(out->gyro.z, dev->gyro_range);

    out->sensor_time_s = (float)out->sensor_time * BMI088_SENSORTIME_TICK_S;

    return BMI088_OK;
}

float bmi088_accel_to_mps2(int16_t raw, uint8_t range)
{
    float full_scale_g;

    if (range > BMI088_ACCEL_RANGE_24G)
    {
        range = BMI088_ACCEL_RANGE_24G;
    }

    /* 手册 5.3.4：a[mg] = raw / 32768 * 1000 * 2^(range+1) * 1.5
     * 即满量程 = 3g * 2^range，范围 3/6/12/24g。 */
    full_scale_g = 3.0f * (float)(1u << range);

    return ((float)raw / 32768.0f) * full_scale_g * BMI088_GRAVITY;
}

float bmi088_gyro_to_dps(int16_t raw, uint8_t range)
{
    float full_scale_dps;

    switch (range)
    {
        case BMI088_GYRO_RANGE_2000DPS:
            full_scale_dps = 2000.0f;
            break;

        case BMI088_GYRO_RANGE_1000DPS:
            full_scale_dps = 1000.0f;
            break;

        case BMI088_GYRO_RANGE_500DPS:
            full_scale_dps = 500.0f;
            break;

        case BMI088_GYRO_RANGE_250DPS:
            full_scale_dps = 250.0f;
            break;

        case BMI088_GYRO_RANGE_125DPS:
            full_scale_dps = 125.0f;
            break;

        default:
            full_scale_dps = 500.0f;
            break;
    }

    return ((float)raw * full_scale_dps) / 32768.0f;
}
