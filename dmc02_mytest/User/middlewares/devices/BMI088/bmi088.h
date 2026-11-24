/**
 * @file    bmi088.h
 * @brief   BMI088 六轴 IMU 驱动（单 .c + 单 .h 版本）
 * @version 1.0
 * @note    源文件编码: UTF-8
 *
 * ============================ 芯片说明 ============================
 * BMI088 内部是两颗独立的 die（加速度计 + 陀螺仪），所以：
 *   加速度计: CHIP_ID = 0x1E，I2C 地址 0x18(SDO=GND) / 0x19(SDO=VDDIO)，SPI 片选 CSB1
 *   陀螺仪  : CHIP_ID = 0x0F，I2C 地址 0x68(SDO=GND) / 0x69(SDO=VDDIO)，SPI 片选 CSB2
 * 两边寄存器是两套独立的地址空间（例如 0x00 分别是各自的 CHIP_ID），
 * 本驱动内部按 accel / gyro 分开访问。
 *
 * 注意：加速度计上电后没有额外的初始化序列。按手册第 3 章 Quick Start：
 *       上电 -> 等待 1ms -> 写 ACC_PWR_CTRL(0x7D) = 0x04 -> 等待 50ms
 *       之后即可正常读数。bmi088_init() 内部就是按这个顺序做的：
 *       软复位、清 PWR_CONF、使能 PWR_CTRL、再写 ODR/带宽/量程。
 *       （0x59 / 0x2A 在手册 5.2 的寄存器表里属于保留区，本驱动不访问。）
 *
 * ============================ 数据读取 ============================
 * bmi088_read_all() 一次读出：
 *   加速度 X/Y/Z   （原始 LSB + m/s^2）
 *   陀螺仪 X/Y/Z   （原始 LSB + deg/s）
 *   温度           （0.001°C + °C）
 *   Sensor time    （39.0625us/tick + 秒）
 *   加速度状态寄存器 0x03、陀螺仪中断状态寄存器 0x0A、加速度错误寄存器 0x02
 *
 * ============================ 移植方法 ============================
 * 只需要实现 3 个回调函数并填进 struct bmi088_dev：
 *
 *   I2C（STM32 HAL 示例）:
 *   static int8_t my_i2c_read(uint8_t id, uint8_t reg, uint8_t *buf, uint16_t len)
 *   {
 *       return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(id << 1), reg,
 *                                I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) ? 0 : -1;
 *   }
 *   static int8_t my_i2c_write(uint8_t id, uint8_t reg, const uint8_t *buf, uint16_t len)
 *   {
 *       return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(id << 1), reg,
 *                                 I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100) == HAL_OK) ? 0 : -1;
 *   }
 *   static void my_delay_us(uint32_t us) { delay_us(us); }   // 填你自己的微秒延时函数
 *
 *   SPI: id 不再是 I2C 地址，而是"片选编号"（accel_id=0 -> CSB1，gyro_id=1 -> CSB2）。
 *        回调里根据 id 拉低对应片选，然后：发 1 字节寄存器地址 -> 连续收发 len 字节
 *        -> 拉高片选。寄存器地址的 bit7 驱动已经处理好，回调直接发送收到的地址即可。
 *        注意：加速度计 SPI 读时驱动会请求 len+1 字节并丢弃第 1 字节(dummy)，
 *        回调按传入的 len 收发即可，不要再自己加 dummy。
 *
 * ============================ 使用示例 ============================
 *   struct bmi088_dev imu;
 *   struct bmi088_data data;
 *
 *   imu.intf     = BMI088_I2C;
 *   imu.accel_id = BMI088_ACCEL_I2C_ADDR;   // 0x18
 *   imu.gyro_id  = BMI088_GYRO_I2C_ADDR;    // 0x68
 *   imu.read     = my_i2c_read;
 *   imu.write    = my_i2c_write;
 *   imu.delay_us = my_delay_us;
 *
 *   if (bmi088_init(&imu) == BMI088_OK)   // 内部默认配置 100Hz/+-6g、400Hz/+-500dps
 *   {
 *       while (1)
 *       {
 *           if (bmi088_read_all(&imu, &data) == BMI088_OK)
 *           {
 *               // data.accel_ms2.x / data.gyro_dps.z / data.temperature_c ...
 *           }
 *           my_delay_us(2500);               // 400Hz 读取
 *       }
 *   }
 *
 *   需要改配置时（必须在 bmi088_init() 之后调用）：
 *   bmi088_set_accel_config(&imu, BMI088_ACCEL_ODR_200HZ, BMI088_ACCEL_BW_NORMAL, BMI088_ACCEL_RANGE_6G);
 *   bmi088_set_gyro_config(&imu, BMI088_GYRO_BW_47_ODR_400HZ, BMI088_GYRO_RANGE_500DPS);
 */

#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=========================================================================
 * 1. 加速度计寄存器（地址空间 A）
 *
 * 只列出手册 5.2 表中存在、且本驱动会用到的寄存器。
 * 0x59(ACC_INIT_CTRL)、0x2A(ACC_INTERNAL_STAT)、0x1E、0x55~0x57、0x70
 * 在 BMI088 上属于保留区，不定义、不访问。
 *=========================================================================*/
#define BMI088_REG_ACCEL_CHIP_ID        UINT8_C(0x00) /* 固定值 0x1E */
#define BMI088_REG_ACCEL_ERR            UINT8_C(0x02) /* bit0 致命错误, bit4:2 错误码 */
#define BMI088_REG_ACCEL_STATUS         UINT8_C(0x03) /* bit7 数据就绪（读数据后自动清零） */
#define BMI088_REG_ACCEL_X_LSB          UINT8_C(0x12)
#define BMI088_REG_ACCEL_X_MSB          UINT8_C(0x13)
#define BMI088_REG_ACCEL_Y_LSB          UINT8_C(0x14)
#define BMI088_REG_ACCEL_Y_MSB          UINT8_C(0x15)
#define BMI088_REG_ACCEL_Z_LSB          UINT8_C(0x16)
#define BMI088_REG_ACCEL_Z_MSB          UINT8_C(0x17)
#define BMI088_REG_ACCEL_SENSORTIME_0   UINT8_C(0x18) /* 低字节 */
#define BMI088_REG_ACCEL_SENSORTIME_1   UINT8_C(0x19)
#define BMI088_REG_ACCEL_SENSORTIME_2   UINT8_C(0x1A) /* 高字节 */
#define BMI088_REG_ACCEL_INT_STAT_1     UINT8_C(0x1D) /* bit7 数据就绪 */
#define BMI088_REG_ACCEL_TEMP_MSB       UINT8_C(0x22) /* 11 位温度，MSB 在高字节 */
#define BMI088_REG_ACCEL_TEMP_LSB       UINT8_C(0x23)
#define BMI088_REG_ACCEL_CONF           UINT8_C(0x40) /* bit7:4 带宽, bit3:0 ODR */
#define BMI088_REG_ACCEL_RANGE          UINT8_C(0x41) /* bit1:0 量程 */
#define BMI088_REG_ACCEL_INT1_IO_CONF   UINT8_C(0x53)
#define BMI088_REG_ACCEL_INT2_IO_CONF   UINT8_C(0x54)
#define BMI088_REG_ACCEL_INT_MAP_DATA   UINT8_C(0x58)
#define BMI088_REG_ACCEL_SELF_TEST      UINT8_C(0x6D)
#define BMI088_REG_ACCEL_PWR_CONF       UINT8_C(0x7C) /* 0x00 = active */
#define BMI088_REG_ACCEL_PWR_CTRL       UINT8_C(0x7D) /* bit2 = 1 使能加速度计 */
#define BMI088_REG_ACCEL_SOFTRESET      UINT8_C(0x7E) /* 写 0xB6 复位 */

/*=========================================================================
 * 2. 陀螺仪寄存器（地址空间 G）
 *=========================================================================*/
#define BMI088_REG_GYRO_CHIP_ID         UINT8_C(0x00) /* 固定值 0x0F */
#define BMI088_REG_GYRO_X_LSB           UINT8_C(0x02)
#define BMI088_REG_GYRO_X_MSB           UINT8_C(0x03)
#define BMI088_REG_GYRO_Y_LSB           UINT8_C(0x04)
#define BMI088_REG_GYRO_Y_MSB           UINT8_C(0x05)
#define BMI088_REG_GYRO_Z_LSB           UINT8_C(0x06)
#define BMI088_REG_GYRO_Z_MSB           UINT8_C(0x07)
#define BMI088_REG_GYRO_INT_STAT_1      UINT8_C(0x0A) /* bit7 DRDY, bit4 FIFO */
#define BMI088_REG_GYRO_FIFO_STATUS     UINT8_C(0x0E)
#define BMI088_REG_GYRO_RANGE           UINT8_C(0x0F) /* bit2:0 */
#define BMI088_REG_GYRO_BANDWIDTH       UINT8_C(0x10) /* bit3:0 = ODR/BW */
#define BMI088_REG_GYRO_LPM1            UINT8_C(0x11) /* 0x00 正常, 0x80 挂起, 0x20 深度挂起 */
#define BMI088_REG_GYRO_SOFTRESET       UINT8_C(0x14) /* 写 0xB6 复位 */
#define BMI088_REG_GYRO_INT_CTRL        UINT8_C(0x15)
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF UINT8_C(0x16)
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP  UINT8_C(0x18)
#define BMI088_REG_GYRO_FIFO_WM_ENABLE  UINT8_C(0x1E)
#define BMI088_REG_GYRO_SELF_TEST       UINT8_C(0x3C)
#define BMI088_REG_GYRO_FIFO_CONFIG0    UINT8_C(0x3D)
#define BMI088_REG_GYRO_FIFO_CONFIG1    UINT8_C(0x3E)
#define BMI088_REG_GYRO_FIFO_DATA       UINT8_C(0x3F)

/*=========================================================================
 * 3. 芯片 ID / I2C 地址 / 常量
 *=========================================================================*/
#define BMI088_ACCEL_CHIP_ID            UINT8_C(0x1E)
#define BMI088_GYRO_CHIP_ID             UINT8_C(0x0F)

#define BMI088_ACCEL_I2C_ADDR           UINT8_C(0x18) /* SDO 接 GND */
#define BMI088_ACCEL_I2C_ADDR_ALT       UINT8_C(0x19) /* SDO 接 VDDIO */
#define BMI088_GYRO_I2C_ADDR            UINT8_C(0x68) /* SDO 接 GND */
#define BMI088_GYRO_I2C_ADDR_ALT        UINT8_C(0x69) /* SDO 接 VDDIO */

#define BMI088_SOFT_RESET_CMD           UINT8_C(0xB6)

#define BMI088_SPI_READ_BIT             UINT8_C(0x80) /* SPI 读: 地址 bit7 置 1 */
#define BMI088_SPI_WRITE_MASK           UINT8_C(0x7F) /* SPI 写: 地址 bit7 清 0 */

/* Sensor time 每个 tick 的时间（秒） */
#define BMI088_SENSORTIME_TICK_S        (39.0625e-6f)
/* 重力加速度 */
#define BMI088_GRAVITY                  (9.80665f)

/*=========================================================================
 * 4. 加速度计配置值（写入 0x40 / 0x41）
 *=========================================================================*/
/* 输出数据率 ODR（写 0x40 低 4 位） */
#define BMI088_ACCEL_ODR_12_5HZ         UINT8_C(0x05)
#define BMI088_ACCEL_ODR_25HZ           UINT8_C(0x06)
#define BMI088_ACCEL_ODR_50HZ           UINT8_C(0x07)
#define BMI088_ACCEL_ODR_100HZ          UINT8_C(0x08)
#define BMI088_ACCEL_ODR_200HZ          UINT8_C(0x09)
#define BMI088_ACCEL_ODR_400HZ          UINT8_C(0x0A)
#define BMI088_ACCEL_ODR_800HZ          UINT8_C(0x0B)
#define BMI088_ACCEL_ODR_1600HZ         UINT8_C(0x0C)

/* 带宽（写 0x40 高 4 位，驱动内部左移 4 位） */
#define BMI088_ACCEL_BW_OSR4            UINT8_C(0x08)
#define BMI088_ACCEL_BW_OSR2            UINT8_C(0x09)
#define BMI088_ACCEL_BW_NORMAL          UINT8_C(0x0A)

/* 量程（写 0x41 低 2 位）；1 LSB = 3g * 2^range / 32768，
 * 即 ±3g: 0.0916mg/LSB，±6g: 0.183mg/LSB，±12g: 0.366mg/LSB，±24g: 0.732mg/LSB */
#define BMI088_ACCEL_RANGE_3G           UINT8_C(0x00)
#define BMI088_ACCEL_RANGE_6G           UINT8_C(0x01)
#define BMI088_ACCEL_RANGE_12G          UINT8_C(0x02)
#define BMI088_ACCEL_RANGE_24G          UINT8_C(0x03)

/* 加速度计电源控制值 */
#define BMI088_ACCEL_POWER_ENABLE       UINT8_C(0x04) /* 0x7D = 0x04 */
#define BMI088_ACCEL_POWER_DISABLE      UINT8_C(0x00) /* 0x7D = 0x00 */
#define BMI088_ACCEL_PM_ACTIVE          UINT8_C(0x00) /* 0x7C = 0x00 */
#define BMI088_ACCEL_PM_SUSPEND         UINT8_C(0x03) /* 0x7C = 0x03 */

/*=========================================================================
 * 5. 陀螺仪配置值（写入 0x0F / 0x10 / 0x11）
 *=========================================================================*/
/* ODR 与带宽组合（写 0x10 低 4 位） */
#define BMI088_GYRO_BW_532_ODR_2000HZ   UINT8_C(0x00)
#define BMI088_GYRO_BW_230_ODR_2000HZ   UINT8_C(0x01)
#define BMI088_GYRO_BW_116_ODR_1000HZ   UINT8_C(0x02)
#define BMI088_GYRO_BW_47_ODR_400HZ     UINT8_C(0x03)
#define BMI088_GYRO_BW_23_ODR_200HZ     UINT8_C(0x04)
#define BMI088_GYRO_BW_12_ODR_100HZ     UINT8_C(0x05)
#define BMI088_GYRO_BW_64_ODR_200HZ     UINT8_C(0x06)
#define BMI088_GYRO_BW_32_ODR_100HZ     UINT8_C(0x07)

/* 量程（写 0x0F 低 3 位） */
#define BMI088_GYRO_RANGE_2000DPS       UINT8_C(0x00)
#define BMI088_GYRO_RANGE_1000DPS       UINT8_C(0x01)
#define BMI088_GYRO_RANGE_500DPS        UINT8_C(0x02)
#define BMI088_GYRO_RANGE_250DPS        UINT8_C(0x03)
#define BMI088_GYRO_RANGE_125DPS        UINT8_C(0x04)

/* 电源模式（写 0x11） */
#define BMI088_GYRO_PM_NORMAL           UINT8_C(0x00)
#define BMI088_GYRO_PM_DEEP_SUSPEND     UINT8_C(0x20)
#define BMI088_GYRO_PM_SUSPEND          UINT8_C(0x80)

/* 中断控制（写 0x15）：bit7 使能新数据中断，否则 0x0A 的 DRDY 位有效不了 */
#define BMI088_GYRO_DATA_EN             UINT8_C(0x80)

/*=========================================================================
 * 6. 状态位掩码
 *=========================================================================*/
#define BMI088_ACCEL_DRDY_MSK           UINT8_C(0x80) /* 0x03 / 0x1D bit7 */
#define BMI088_GYRO_DRDY_MSK            UINT8_C(0x80) /* 0x0A bit7 */
#define BMI088_GYRO_FIFO_MSK            UINT8_C(0x10) /* 0x0A bit4 */

#define BMI088_ERR_FATAL_MSK            UINT8_C(0x01) /* 0x02 bit0 */
#define BMI088_ERR_CODE_MSK             UINT8_C(0x1C) /* 0x02 bit4:2 */
#define BMI088_ERR_CODE_POS             UINT8_C(0x02)

/*=========================================================================
 * 7. 返回码
 *=========================================================================*/
#define BMI088_OK                       INT8_C(0)
#define BMI088_E_NULL_PTR               INT8_C(-1) /* 空指针 */
#define BMI088_E_COM_FAIL               INT8_C(-2) /* 读写回调返回失败 */
#define BMI088_E_DEV_NOT_FOUND          INT8_C(-3) /* 芯片 ID 不对 */
#define BMI088_E_INVALID_PARAM          INT8_C(-4) /* 参数非法 */
#define BMI088_E_INVALID_DATA           INT8_C(-5) /* 数据无效（如温度 MSB=0x80） */

/*=========================================================================
 * 8. 类型定义
 *=========================================================================*/

/*! 总线类型 */
enum bmi088_intf
{
    BMI088_I2C = 0,
    BMI088_SPI = 1
};

/*! 三轴原始数据（LSB） */
struct bmi088_sensor_data
{
    int16_t x;
    int16_t y;
    int16_t z;
};

/*! 三轴物理量数据 */
struct bmi088_sensor_data_f
{
    float x;
    float y;
    float z;
};

/*! 加速度计错误寄存器 0x02 解析结果 */
struct bmi088_err_reg
{
    uint8_t fatal_err; /* 1: 芯片处于非正常工作状态（需上电复位或软复位） */
    uint8_t err_code;  /* 1: ACCEL_CONF 配置错误 */
};

/*!
 * @brief 一次读出的全部数据
 */
struct bmi088_data
{
    struct bmi088_sensor_data accel;       /*!< 加速度原始值（LSB） */
    struct bmi088_sensor_data gyro;        /*!< 陀螺仪原始值（LSB） */
    struct bmi088_sensor_data_f accel_ms2; /*!< 加速度换算值（m/s^2） */
    struct bmi088_sensor_data_f gyro_dps;  /*!< 陀螺仪换算值（deg/s） */
    int32_t temperature_mc;                /*!< 温度，单位 0.001°C（25000 = 25.000°C） */
    float temperature_c;                   /*!< 温度（°C） */
    uint8_t temperature_valid;             /*!< 1 = 本次温度有效；0 = 温度无效（MSB=0x80） */
    uint32_t sensor_time;                  /*!< Sensor time tick，1 tick = 39.0625us，655.36s 回绕 */
    float sensor_time_s;                   /*!< Sensor time（秒） */
    uint8_t accel_status;                  /*!< 读数据前采样的 0x03，bit7 = 数据就绪 */
    uint8_t gyro_status;                   /*!< 读数据前采样的 0x0A，bit7 = 数据就绪（窗口仅 280~400us，不适合判断新鲜度） */
    struct bmi088_err_reg err;             /*!< 寄存器 0x02 */
};

/*!
 * @brief 总线读回调
 * @param[in]  id   I2C: 7 位从机地址；SPI: 片选编号（0=CSB1, 1=CSB2）
 * @param[in]  reg  寄存器地址（SPI 时 bit7 已按读/写规则处理）
 * @param[out] data 读出的数据
 * @param[in]  len  要读的字节数
 * @return 0 成功，非 0 失败
 */
typedef int8_t (*bmi088_read_fptr_t)(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len);

/*!
 * @brief 总线写回调（参数含义同上）
 */
typedef int8_t (*bmi088_write_fptr_t)(uint8_t id, uint8_t reg, const uint8_t *data, uint16_t len);

/*!
 * @brief 微秒级延时回调
 * @param[in] us 延时时间（微秒）
 */
typedef void (*bmi088_delay_us_fptr_t)(uint32_t us);

/*!
 * @brief 设备实例
 *
 * 使用前必须填：intf、accel_id、gyro_id、read、write、delay_us。
 * accel_odr / accel_bw / accel_range / gyro_odr_bw / gyro_range 由
 * bmi088_init() 或配置函数自动填写，供 LSB 换算使用，用户不需要手动维护。
 */
struct bmi088_dev
{
    enum bmi088_intf intf;    /*!< BMI088_I2C 或 BMI088_SPI */
    uint8_t accel_id;         /*!< I2C: 0x18/0x19；SPI: 片选编号 0 */
    uint8_t gyro_id;          /*!< I2C: 0x68/0x69；SPI: 片选编号 1 */
    uint8_t accel_chip_id;    /*!< 读到的加速度计 ID（应为 0x1E） */
    uint8_t gyro_chip_id;     /*!< 读到的陀螺仪 ID（应为 0x0F） */
    uint8_t accel_odr;        /*!< 当前加速度计 ODR */
    uint8_t accel_bw;         /*!< 当前加速度计带宽 */
    uint8_t accel_range;      /*!< 当前加速度计量程 */
    uint8_t gyro_odr_bw;      /*!< 当前陀螺仪 ODR/带宽 */
    uint8_t gyro_range;       /*!< 当前陀螺仪量程 */
    bmi088_read_fptr_t read;          /*!< 读寄存器回调 */
    bmi088_write_fptr_t write;        /*!< 写寄存器回调 */
    bmi088_delay_us_fptr_t delay_us;  /*!< 微秒延时回调 */
};

/*=========================================================================
 * 9. API
 *=========================================================================*/

/**
 * @brief  初始化 BMI088（推荐入口）
 * @details 依次完成：上电等待 -> 读两个芯片 ID -> 双软复位 ->
 *          加速度计使能并等待 50ms -> 写入默认配置（100Hz/+-6g、400Hz/+-500dps）。
 * @param  dev 设备实例（回调必须已填好）
 * @return BMI088_OK 成功；其他见返回码
 * @note   整个初始化约 220ms（加速度计软复位 50ms、使能后 50ms、配置后 40ms，
 *         陀螺仪复位 30ms、配置 10ms）。
 */
int8_t bmi088_init(struct bmi088_dev *dev);

/**
 * @brief  两个传感器软复位
 * @param  dev 设备实例
 * @return BMI088_OK 成功
 * @note   复位后配置寄存器回到默认值，需要重新调用配置函数；本函数会把
 *         dev 里的量程同步成复位默认值（加速度计 ±6g、陀螺仪 ±2000dps），
 *         避免换算用错量程。
 */
int8_t bmi088_soft_reset(struct bmi088_dev *dev);

/**
 * @brief  配置加速度计（会同时把加速度计切到 active 模式）
 * @param  dev   设备实例（必须是 bmi088_init 之后的实例）
 * @param  odr   BMI088_ACCEL_ODR_xxx
 * @param  bw    BMI088_ACCEL_BW_xxx
 * @param  range BMI088_ACCEL_RANGE_xxx
 * @return BMI088_OK 成功
 */
int8_t bmi088_set_accel_config(struct bmi088_dev *dev, uint8_t odr, uint8_t bw, uint8_t range);

/**
 * @brief  配置陀螺仪（会同时把陀螺仪切到 normal 模式）
 * @param  dev    设备实例
 * @param  odr_bw BMI088_GYRO_BW_xxx
 * @param  range  BMI088_GYRO_RANGE_xxx
 * @return BMI088_OK 成功
 */
int8_t bmi088_set_gyro_config(struct bmi088_dev *dev, uint8_t odr_bw, uint8_t range);

/**
 * @brief  读加速度计三轴原始值
 * @param  dev   设备实例
 * @param  accel 输出
 * @return BMI088_OK 成功
 */
int8_t bmi088_read_accel(struct bmi088_dev *dev, struct bmi088_sensor_data *accel);

/**
 * @brief  读陀螺仪三轴原始值
 * @param  dev  设备实例
 * @param  gyro 输出
 * @return BMI088_OK 成功
 */
int8_t bmi088_read_gyro(struct bmi088_dev *dev, struct bmi088_sensor_data *gyro);

/**
 * @brief  读芯片温度
 * @param  dev     设备实例
 * @param  temp_mc 输出，单位 0.001°C（25000 表示 25.000°C）
 * @return BMI088_OK 成功；BMI088_E_INVALID_DATA 表示本次温度无效（MSB=0x80）
 * @note   温度每 1.28s 才更新一次，高频调用会读到重复值。
 */
int8_t bmi088_read_temperature(struct bmi088_dev *dev, int32_t *temp_mc);

/**
 * @brief  读 Sensor time
 * @param  dev         设备实例
 * @param  sensor_time 输出，单位 tick（1 tick = 39.0625us，24 位，655.36s 回绕）
 * @return BMI088_OK 成功
 */
int8_t bmi088_read_sensor_time(struct bmi088_dev *dev, uint32_t *sensor_time);

/**
 * @brief  读加速度计错误寄存器 0x02
 * @param  dev 设备实例
 * @param  err 输出
 * @return BMI088_OK 成功
 */
int8_t bmi088_read_error(struct bmi088_dev *dev, struct bmi088_err_reg *err);

/**
 * @brief  读两个传感器的状态寄存器
 * @param  dev          设备实例
 * @param  accel_status 输出，寄存器 0x03（bit7 = 数据就绪），可为 NULL
 * @param  gyro_status  输出，寄存器 0x0A（bit7 = 数据就绪），可为 NULL
 * @return BMI088_OK 成功
 * @note   acc_drdy 在数据寄存器被读走后会自动清零，所以要在读数据之前调用。
 *         gyro_drdy 只有约 280~400us 的有效窗口，400Hz 轮询时命中率只有 10%~16%，
 *         不适合用来判断数据新鲜度；陀螺仪请用固定节拍或 INT3/INT4 中断。
 *         bmi088_set_gyro_config 会写 0x15=0x80 打开新数据中断使能，
 *         但 0x0A.bit7 是否依赖该使能位，手册未明确，以实测为准。
 */
int8_t bmi088_read_status(struct bmi088_dev *dev, uint8_t *accel_status, uint8_t *gyro_status);

/**
 * @brief  一次读出全部数据（加速度、陀螺仪、温度、Sensor time、状态、错误）
 * @param  dev 设备实例
 * @param  out 输出（内部会先清零）
 * @return BMI088_OK 成功；通信失败返回错误码
 * @note   共 6 次总线读：状态位都在读数据之前采样（否则 DRDY 已被清掉）；
 *         加速度三轴与 Sensor time 用一次 9 字节突发读完成，保证采样与时间戳同步；
 *         温度无效（MSB=0x80）不算致命错误，此时 temperature_valid = 0。
 *         判断加速度数据是否更新，比较 sensor_time 是否变化最可靠；
 *         陀螺仪没有对应时间戳，靠固定节拍或 INT3/INT4 中断。
 *         100~400Hz 轮询足够；更高 ODR 建议改用 FIFO 批量读取。
 */
int8_t bmi088_read_all(struct bmi088_dev *dev, struct bmi088_data *out);

/**
 * @brief  加速度原始值换算为 m/s^2
 * @param  raw   原始 LSB
 * @param  range BMI088_ACCEL_RANGE_xxx
 * @return m/s^2
 */
float bmi088_accel_to_mps2(int16_t raw, uint8_t range);

/**
 * @brief  陀螺仪原始值换算为 deg/s
 * @param  raw   原始 LSB
 * @param  range BMI088_GYRO_RANGE_xxx
 * @return deg/s
 */
float bmi088_gyro_to_dps(int16_t raw, uint8_t range);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
