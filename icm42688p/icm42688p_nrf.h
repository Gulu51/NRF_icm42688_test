/**
 * @file    icm42688p_nrf.h
 * @brief   ICM42688P 6-axis IMU driver for nRF52832 (SPI)
 *
 * @note    SPI Mode 3 (CPOL=1, CPHA=1), MSB first, max 8MHz SCLK
 *
 * 引脚接线 (可根据实际板子修改):
 *   nRF52832          ICM42688P
 *   --------          ---------
 *   P0.08 / P8 (CS)   -->  CS/SS
 *   P0.07 / P7 (SCK)  -->  SCL/SCK
 *   P0.05 / P5 (MISO) <--  SDO/MISO
 *   P0.06 / P6 (MOSI) -->  SDI/MOSI
 *   3.3V              -->  VDD, VDDIO
 *   GND               -->  GND
 */

#ifndef __ICM42688P_NRF_H__
#define __ICM42688P_NRF_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════
   SPI 引脚配置 (按实际接线修改)
   ═══════════════════════════════════════════════════ */
#define ICM42688P_SPI_INSTANCE    1    // 使用 SPIM1

#define ICM42688P_CS_PIN         8     // P0.08 = P8
#define ICM42688P_SCK_PIN        7     // P0.07 = P7
#define ICM42688P_MISO_PIN       5     // P0.05 = P5
#define ICM42688P_MOSI_PIN       6     // P0.06 = P6

/* ═══════════════════════════════════════════════════
   传感器参数
   ═══════════════════════════════════════════════════ */
#define ICM42688P_WHO_AM_I       0x47

/* 加速度计量程 (g) */
typedef enum {
    ACCEL_FS_16G = 0,   // ±16g
    ACCEL_FS_8G  = 1,   // ±8g
    ACCEL_FS_4G  = 2,   // ±4g
    ACCEL_FS_2G  = 3    // ±2g
} icm42688p_accel_fs_t;

/* 陀螺仪量程 (dps) */
typedef enum {
    GYRO_FS_2000  = 0,  // ±2000 dps
    GYRO_FS_1000  = 1,  // ±1000 dps
    GYRO_FS_500   = 2,  // ±500 dps
    GYRO_FS_250   = 3,  // ±250 dps
    GYRO_FS_125   = 4,  // ±125 dps
    GYRO_FS_62_5  = 5,  // ±62.5 dps
    GYRO_FS_31_25 = 6,  // ±31.25 dps
    GYRO_FS_15_625= 7   // ±15.625 dps
} icm42688p_gyro_fs_t;

/* ODR (输出数据速率) */
typedef enum {
    ODR_32KHZ    = 1,
    ODR_16KHZ    = 2,
    ODR_8KHZ     = 3,
    ODR_4KHZ     = 4,
    ODR_2KHZ     = 5,
    ODR_1KHZ     = 6,   // default
    ODR_200HZ    = 7,
    ODR_100HZ    = 8,
    ODR_50HZ     = 9,
    ODR_25HZ     = 10,
    ODR_12_5HZ   = 11,
    ODR_500HZ    = 15
} icm42688p_odr_t;

/* IMU 数据结构体 */
typedef struct {
    float acc_x;       // 加速度 X (g)
    float acc_y;       // 加速度 Y (g)
    float acc_z;       // 加速度 Z (g)
    float gyro_x;      // 陀螺仪 X (dps)
    float gyro_y;      // 陀螺仪 Y (dps)
    float gyro_z;      // 陀螺仪 Z (dps)
    float temp_c;      // 温度 (°C)
} icm42688p_data_t;

/** APEX pedometer output generated inside the ICM42688P. */
typedef struct {
    uint16_t step_count;       /**< Native 16-bit hardware step counter. */
    uint8_t  cadence_raw;      /**< Samples per step in unsigned 6.2 format. */
    uint8_t  activity;         /**< 0 unknown, 1 walking, 2 running. */
} icm42688p_apex_data_t;

/* ═══════════════════════════════════════════════════
   函数原型
   ═══════════════════════════════════════════════════ */

/**@brief 初始化 SPI 外设和 GPIO */
void icm42688p_spi_init(void);

/**@brief 初始化 ICM42688P 传感器
 * @return true 成功, false 失败
 */
bool icm42688p_init(void);

/**@brief Enable accelerometer and gyroscope in low-noise mode. */
bool icm42688p_power_on(void);

/**@brief Enable only the accelerometer at 200 Hz in low-noise mode.
 *
 * The gyroscope and temperature sensor remain disabled. This mode is intended
 * for short, phase-synchronised vibration acquisition windows.
 */
bool icm42688p_accel_high_rate_on(void);

/**@brief Enable only the accelerometer at 25 Hz in low-power mode.
 *
 * Used as the always-on motion and strong-vibration gate. The gyroscope and
 * temperature sensor remain disabled.
 */
bool icm42688p_accel_low_power_on(void);

/**@brief Start the on-chip APEX pedometer.
 *
 * The accelerometer runs at 50 Hz in low-power mode. The gyroscope and
 * temperature sensor stay off. Pedometer processing is performed by the
 * sensor DMP, so the host only needs to read the result periodically.
 */
bool icm42688p_apex_pedometer_on(void);

/**@brief Disable APEX pedometer processing and all sensing blocks. */
bool icm42688p_apex_pedometer_off(void);

/**@brief Read the APEX step counter, cadence and activity classification. */
bool icm42688p_apex_read(icm42688p_apex_data_t *p_data);

/**@brief Enable accelerometer and gyroscope at 25 Hz for normal motion tracking.
 *
 * The accelerometer uses low-power mode, the gyroscope uses low-noise mode,
 * and the temperature sensor remains disabled.
 */
bool icm42688p_motion_tracking_on(void);

/**@brief Change an already-running six-axis stream to 25 Hz tracking. */
bool icm42688p_motion_tracking_rate_set(void);

/**@brief Change an already-running six-axis stream to 50 Hz cycle capture. */
bool icm42688p_motion_capture_rate_set(void);

/**@brief Enable accelerometer and gyroscope at 200 Hz for a short motion window.
 *
 * The temperature sensor remains disabled.
 */
bool icm42688p_motion_high_rate_on(void);

/**@brief Disable accelerometer and gyroscope while keeping SPI accessible. */
bool icm42688p_power_off(void);

/**@brief 读取传感器数据
 * @param p_data 数据输出指针
 * @return true 成功, false 失败
 */
bool icm42688p_read_data(icm42688p_data_t *p_data);

/**@brief Read only the six accelerometer bytes for low-energy burst sampling. */
bool icm42688p_read_accel(icm42688p_data_t *p_data);

/**@brief Read accelerometer and gyroscope without transferring temperature. */
bool icm42688p_read_motion(icm42688p_data_t *p_data);

/**@brief 设置加速度计量程 */
void icm42688p_set_accel_fs(icm42688p_accel_fs_t fs);

/**@brief 设置陀螺仪量程 */
void icm42688p_set_gyro_fs(icm42688p_gyro_fs_t fs);

/**@brief 设置加速度计 ODR */
bool icm42688p_set_accel_odr(icm42688p_odr_t odr);

/**@brief 设置陀螺仪 ODR */
bool icm42688p_set_gyro_odr(icm42688p_odr_t odr);

/**@brief 获取加速度计量程对应的 scale 值 (用于原始值转换) */
float icm42688p_get_accel_scale(void);

/**@brief 获取陀螺仪量程对应的 scale 值 */
float icm42688p_get_gyro_scale(void);

/**@brief 读取寄存器 */
bool icm42688p_read_reg(uint8_t reg, uint8_t *p_data, uint8_t len);

/**@brief 写入寄存器 */
bool icm42688p_write_reg(uint8_t reg, uint8_t data);

/**@brief 传感器自检
 * @return true 传感器正常, false 异常
 */
bool icm42688p_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688P_NRF_H__ */
