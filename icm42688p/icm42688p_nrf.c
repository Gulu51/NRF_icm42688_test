/**
 * @file    icm42688p_nrf.c
 * @brief   ICM42688P 6-axis IMU driver for nRF52832 (软件 SPI)
 *
 * @note    使用 GPIO 软件模拟 SPI Mode 3 (CPOL=1, CPHA=1)。
 *          已验证硬件 SPIM 不通但软件 SPI 正常，因此全部改用软件模拟。
 */

#include "icm42688p_nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "MyLog.h"

/* ═══════════════════════════════════════════════════
   寄存器地址 (User Bank 0)
   ═══════════════════════════════════════════════════ */
#define REG_BANK_SEL          0x76
#define REG_DEVICE_CONFIG     0x11
#define REG_TEMP_DATA1        0x1D
#define REG_ACCEL_DATA_X1     0x1F
#define REG_GYRO_DATA_X1      0x25
#define REG_APEX_DATA0        0x31
#define REG_SIGNAL_PATH_RESET 0x4B
#define REG_PWR_MGMT0         0x4E
#define REG_GYRO_CONFIG0      0x4F
#define REG_ACCEL_CONFIG0     0x50
#define REG_APEX_CONFIG0      0x56
#define REG_WHO_AM_I          0x75

/* User Bank 4 APEX tuning registers. */
#define REG_APEX_CONFIG1_B4   0x40
#define REG_APEX_CONFIG2_B4   0x41
#define REG_APEX_CONFIG3_B4   0x42
#define REG_APEX_CONFIG9_B4   0x48

#define SIGNAL_DMP_INIT_EN         0x40
#define SIGNAL_DMP_MEM_RESET_EN    0x20
#define APEX_PED_ENABLE            0x20
#define APEX_DMP_ODR_50HZ          0x02

/* TDK's documented default pedometer tuning for ICM-42688-P. */
#define APEX_CONFIG1_PED_DEFAULT   0xA2
#define APEX_CONFIG2_PED_DEFAULT   0x85
#define APEX_CONFIG3_PED_DEFAULT   0x51

/* ═══════════════════════════════════════════════════
   静态变量
   ═══════════════════════════════════════════════════ */
static float s_accel_scale = 0.0f;
static float s_gyro_scale = 0.0f;
static uint8_t s_current_bank = 0;

#define TEMP_DATA_REG_SCALE  132.48f
#define TEMP_OFFSET          25.0f

/* ═══════════════════════════════════════════════════
   SPI 软件驱动 (GPIO bit-bang)
   ═══════════════════════════════════════════════════ */

#define SCK_HI()  nrf_gpio_pin_set(ICM42688P_SCK_PIN)
#define SCK_LO()  nrf_gpio_pin_clear(ICM42688P_SCK_PIN)
#define MOSI_HI() nrf_gpio_pin_set(ICM42688P_MOSI_PIN)
#define MOSI_LO() nrf_gpio_pin_clear(ICM42688P_MOSI_PIN)
#define MISO_RD() nrf_gpio_pin_read(ICM42688P_MISO_PIN)
#define CS_EN()   nrf_gpio_pin_clear(ICM42688P_CS_PIN)
#define CS_DIS()  nrf_gpio_pin_set(ICM42688P_CS_PIN)

void icm42688p_spi_init(void)
{
    nrf_gpio_cfg_output(ICM42688P_CS_PIN);
    nrf_gpio_cfg_output(ICM42688P_SCK_PIN);
    nrf_gpio_cfg_output(ICM42688P_MOSI_PIN);
    nrf_gpio_cfg_input(ICM42688P_MISO_PIN, NRF_GPIO_PIN_NOPULL);

    CS_DIS();
    SCK_LO();  // CPOL=1 → 空闲高电平, 但先拉低再开始传输

    MY_LOG_DEBUG("ICM42688P SW SPI inited (SCK:P0.%d MOSI:P0.%d MISO:P0.%d CS:P0.%d)",
                 ICM42688P_SCK_PIN, ICM42688P_MOSI_PIN,
                 ICM42688P_MISO_PIN, ICM42688P_CS_PIN);
}

/** @brief 软件 SPI 传输一个字节 */
static uint8_t sw_spi_byte(uint8_t tx)
{
    uint8_t rx = 0;
    for (int8_t b = 7; b >= 0; b--) {
        SCK_LO();
        if (tx & (1 << b)) MOSI_HI(); else MOSI_LO();
        nrf_delay_us(2);
        SCK_HI();
        rx <<= 1;
        if (MISO_RD()) rx |= 1;
        nrf_delay_us(2);
    }
    SCK_LO();
    return rx;
}

/** @brief 软件 SPI 全双工传输 */
static void sw_spi_transfer(const uint8_t *p_tx, uint8_t *p_rx, uint8_t len)
{
    CS_EN();
    for (uint8_t i = 0; i < len; i++) {
        p_rx[i] = sw_spi_byte(p_tx[i]);
    }
    CS_DIS();
}

/* Used for write-on-clear/self-clearing registers that cannot be verified by
   reading the value back. */
static void write_reg_unchecked(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg & 0x7F;
    tx[1] = data;
    sw_spi_transfer(tx, rx, sizeof(tx));
}

/* ═══════════════════════════════════════════════════
   寄存器读写
   ═══════════════════════════════════════════════════ */

bool icm42688p_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2], rx[2];
    tx[0] = reg & 0x7F;
    tx[1] = data;
    sw_spi_transfer(tx, rx, 2);

    /* 回读验证 */
    uint8_t rb;
    if (icm42688p_read_reg(reg, &rb, 1) && rb == data) {
        return true;
    }
    return false;
}

bool icm42688p_read_reg(uint8_t reg, uint8_t *p_data, uint8_t len)
{
    uint8_t tx[32], rx[32];
    if (len > 30) return false;

    tx[0] = reg | 0x80;
    for (uint8_t i = 1; i <= len; i++) tx[i] = 0x00;

    sw_spi_transfer(tx, rx, len + 1);

    for (uint8_t i = 0; i < len; i++) {
        p_data[i] = rx[i + 1];
    }
    return true;
}

/* ═══════════════════════════════════════════════════
   Bank 切换
   ═══════════════════════════════════════════════════ */

static bool set_bank(uint8_t bank)
{
    bool ok;

    if (s_current_bank == bank) return true;
    ok = icm42688p_write_reg(REG_BANK_SEL, bank);
    if (ok) {
        s_current_bank = bank;
    }
    return ok;
}

/* ═══════════════════════════════════════════════════
   初始化
   ═══════════════════════════════════════════════════ */

static void soft_reset(void)
{
    set_bank(0);
    write_reg_unchecked(REG_DEVICE_CONFIG, 0x01);
    nrf_delay_ms(2);
    s_current_bank = 0;
}

bool icm42688p_self_test(void)
{
    uint8_t whoami = 0;
    set_bank(0);
    if (!icm42688p_read_reg(REG_WHO_AM_I, &whoami, 1)) {
        MY_LOG_ERROR("ICM42688P SPI read WHO_AM_I failed");
        return false;
    }
    if (whoami != ICM42688P_WHO_AM_I) {
        MY_LOG_ERROR("ICM42688P WHO_AM_I mismatch: 0x%02X (expected 0x%02X)",
                     whoami, ICM42688P_WHO_AM_I);
        return false;
    }
    MY_LOG_DEBUG("ICM42688P WHO_AM_I OK: 0x%02X", whoami);
    return true;
}

bool icm42688p_init(void)
{
    soft_reset();

    if (!icm42688p_self_test()) {
        return false;
    }

    /* Configure ranges and ODRs while all sensing blocks are off. */
    set_bank(0);
    /* PWR_MGMT0: TEMP_DIS=1, gyro off, accelerometer off. */
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x20)) {
        MY_LOG_ERROR("ICM42688P standby configuration failed");
        return false;
    }

    icm42688p_set_accel_fs(ACCEL_FS_8G);
    icm42688p_set_gyro_fs(GYRO_FS_500);
    if (!icm42688p_set_accel_odr(ODR_25HZ) ||
        !icm42688p_set_gyro_odr(ODR_25HZ)) {
        MY_LOG_ERROR("ICM42688P initial ODR configuration failed");
        return false;
    }

    MY_LOG_DEBUG("ICM42688P init OK");
    return true;
}

bool icm42688p_power_on(void)
{
    set_bank(0);
    /* Temperature remains disabled in the legacy six-axis mode. */
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x2F)) {
        return false;
    }

    /* Gyroscope low-noise mode needs time to produce valid samples. */
    nrf_delay_ms(50);
    return true;
}

bool icm42688p_accel_high_rate_on(void)
{
    /* Keep the vibration channel at 200 Hz, but do not pay the gyro cost.
       PWR_MGMT0: TEMP_DIS=1, GYRO_MODE=OFF, ACCEL_MODE=LOW_NOISE. */
    if (!icm42688p_set_accel_odr(ODR_200HZ)) {
        return false;
    }
    set_bank(0);
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x23)) {
        return false;
    }

    /* Allow the accelerometer output to settle before the sampling timer runs. */
    nrf_delay_ms(10);
    return true;
}

bool icm42688p_accel_low_power_on(void)
{
    if (!icm42688p_set_accel_odr(ODR_25HZ)) {
        return false;
    }
    set_bank(0);
    /* PWR_MGMT0: TEMP_DIS=1, GYRO_MODE=OFF, ACCEL_MODE=LOW_POWER. */
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x22)) {
        return false;
    }

    nrf_delay_ms(20);
    return true;
}

bool icm42688p_apex_pedometer_on(void)
{
    uint8_t apex_config0;
    uint8_t apex_config9;

    if (!set_bank(0)) {
        return false;
    }

    /* Configure the input stream before initializing the DMP. APEX pedometer
       supports 25/50 Hz; 50 Hz is selected for reliable hand-held walking. */
    if (!icm42688p_set_accel_odr(ODR_50HZ)) {
        return false;
    }
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x22)) {
        return false;
    }
    nrf_delay_ms(1);

    /* Select 50 Hz DMP operation, with all APEX algorithms disabled during
       initialization. DMP power-save is deliberately cleared because it
       requires a separately configured WOM source to keep counting. */
    if (!icm42688p_read_reg(REG_APEX_CONFIG0, &apex_config0, 1)) {
        return false;
    }
    apex_config0 &= 0x04;
    apex_config0 |= APEX_DMP_ODR_50HZ;
    if (!icm42688p_write_reg(REG_APEX_CONFIG0, apex_config0)) {
        return false;
    }

    write_reg_unchecked(REG_SIGNAL_PATH_RESET, SIGNAL_DMP_MEM_RESET_EN);
    nrf_delay_ms(1);

    if (!set_bank(4)) {
        return false;
    }
    if (!icm42688p_write_reg(REG_APEX_CONFIG1_B4, APEX_CONFIG1_PED_DEFAULT) ||
        !icm42688p_write_reg(REG_APEX_CONFIG2_B4, APEX_CONFIG2_PED_DEFAULT) ||
        !icm42688p_write_reg(REG_APEX_CONFIG3_B4, APEX_CONFIG3_PED_DEFAULT)) {
        (void)set_bank(0);
        return false;
    }

    /* Normal sensitivity is the documented starting point for hand-held
       walking. Preserve reserved bits in APEX_CONFIG9. */
    if (!icm42688p_read_reg(REG_APEX_CONFIG9_B4, &apex_config9, 1)) {
        (void)set_bank(0);
        return false;
    }
    apex_config9 &= (uint8_t)~0x01U;
    if (!icm42688p_write_reg(REG_APEX_CONFIG9_B4, apex_config9) ||
        !set_bank(0)) {
        return false;
    }

    write_reg_unchecked(REG_SIGNAL_PATH_RESET, SIGNAL_DMP_INIT_EN);
    nrf_delay_ms(50);

    apex_config0 |= APEX_PED_ENABLE;
    if (!icm42688p_write_reg(REG_APEX_CONFIG0, apex_config0)) {
        return false;
    }
    nrf_delay_ms(1);
    MY_LOG_DEBUG("ICM42688P APEX pedometer ON (accel LP 50 Hz, gyro off)");
    return true;
}

bool icm42688p_apex_pedometer_off(void)
{
    uint8_t apex_config0;

    if (!set_bank(0)) {
        return false;
    }
    if (icm42688p_read_reg(REG_APEX_CONFIG0, &apex_config0, 1)) {
        apex_config0 &= (uint8_t)~APEX_PED_ENABLE;
        if (!icm42688p_write_reg(REG_APEX_CONFIG0, apex_config0)) {
            return false;
        }
    }
    return icm42688p_power_off();
}

bool icm42688p_apex_read(icm42688p_apex_data_t *p_data)
{
    uint8_t raw[4];

    if (!p_data || !set_bank(0)) {
        return false;
    }
    if (!icm42688p_read_reg(REG_APEX_DATA0, raw, sizeof(raw))) {
        return false;
    }

    /* APEX_DATA0 is the low counter byte; APEX_DATA1 is the high byte. */
    p_data->step_count = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    p_data->cadence_raw = raw[2];
    p_data->activity = raw[3] & 0x03U;
    return true;
}

bool icm42688p_motion_tracking_rate_set(void)
{
    return icm42688p_set_accel_odr(ODR_25HZ) &&
           icm42688p_set_gyro_odr(ODR_25HZ);
}

bool icm42688p_motion_capture_rate_set(void)
{
    return icm42688p_set_accel_odr(ODR_50HZ) &&
           icm42688p_set_gyro_odr(ODR_50HZ);
}

bool icm42688p_motion_tracking_on(void)
{
    if (!icm42688p_motion_tracking_rate_set()) {
        return false;
    }
    set_bank(0);
    /* PWR_MGMT0: TEMP_DIS=1, GYRO_MODE=LN, ACCEL_MODE=LOW_POWER. */
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x2E)) {
        return false;
    }

    /* The gyroscope needs about 45 ms before its first usable sample. */
    nrf_delay_ms(50);
    return true;
}

bool icm42688p_motion_high_rate_on(void)
{
    if (!icm42688p_set_accel_odr(ODR_200HZ) ||
        !icm42688p_set_gyro_odr(ODR_200HZ)) {
        return false;
    }
    set_bank(0);
    /* PWR_MGMT0: TEMP_DIS=1, GYRO_MODE=LN, ACCEL_MODE=LN. */
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x2F)) {
        return false;
    }

    /* The gyroscope needs about 45 ms before its samples are usable. */
    nrf_delay_ms(50);
    return true;
}

bool icm42688p_power_off(void)
{
    set_bank(0);
    /* Keep TEMP_DIS set; 0x00 would leave the temperature block enabled. */
    if (!icm42688p_write_reg(REG_PWR_MGMT0, 0x20)) {
        return false;
    }

    nrf_delay_us(200);
    return true;
}

/* ═══════════════════════════════════════════════════
   满量程和 ODR 设置
   ═══════════════════════════════════════════════════ */

void icm42688p_set_accel_fs(icm42688p_accel_fs_t fs)
{
    set_bank(0);
    uint8_t reg;
    if (!icm42688p_read_reg(REG_ACCEL_CONFIG0, &reg, 1)) return;
    reg = (reg & 0x1F) | ((uint8_t)fs << 5);
    icm42688p_write_reg(REG_ACCEL_CONFIG0, reg);
    s_accel_scale = (float)(16 >> (uint8_t)fs) / 32768.0f;
}

void icm42688p_set_gyro_fs(icm42688p_gyro_fs_t fs)
{
    set_bank(0);
    uint8_t reg;
    if (!icm42688p_read_reg(REG_GYRO_CONFIG0, &reg, 1)) return;
    reg = (reg & 0x1F) | ((uint8_t)fs << 5);
    icm42688p_write_reg(REG_GYRO_CONFIG0, reg);
    s_gyro_scale = (2000.0f / (float)(1 << (uint8_t)fs)) / 32768.0f;
}

bool icm42688p_set_accel_odr(icm42688p_odr_t odr)
{
    set_bank(0);
    uint8_t reg;
    if (!icm42688p_read_reg(REG_ACCEL_CONFIG0, &reg, 1)) return false;
    reg = (reg & 0xF0) | ((uint8_t)odr & 0x0F);
    return icm42688p_write_reg(REG_ACCEL_CONFIG0, reg);
}

bool icm42688p_set_gyro_odr(icm42688p_odr_t odr)
{
    set_bank(0);
    uint8_t reg;
    if (!icm42688p_read_reg(REG_GYRO_CONFIG0, &reg, 1)) return false;
    reg = (reg & 0xF0) | ((uint8_t)odr & 0x0F);
    return icm42688p_write_reg(REG_GYRO_CONFIG0, reg);
}

float icm42688p_get_accel_scale(void) { return s_accel_scale; }
float icm42688p_get_gyro_scale(void)  { return s_gyro_scale; }

/* ═══════════════════════════════════════════════════
   数据读取
   ═══════════════════════════════════════════════════ */

bool icm42688p_read_data(icm42688p_data_t *p_data)
{
    if (!p_data) return false;

    uint8_t buf[14] = {0};
    if (!icm42688p_read_reg(REG_TEMP_DATA1, buf, 14)) {
        return false;
    }

    int16_t raw_temp   = ((int16_t)buf[0]  << 8) | buf[1];
    int16_t raw_acc_x  = ((int16_t)buf[2]  << 8) | buf[3];
    int16_t raw_acc_y  = ((int16_t)buf[4]  << 8) | buf[5];
    int16_t raw_acc_z  = ((int16_t)buf[6]  << 8) | buf[7];
    int16_t raw_gyro_x = ((int16_t)buf[8]  << 8) | buf[9];
    int16_t raw_gyro_y = ((int16_t)buf[10] << 8) | buf[11];
    int16_t raw_gyro_z = ((int16_t)buf[12] << 8) | buf[13];

    p_data->temp_c  = ((float)raw_temp / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
    p_data->acc_x   = (float)raw_acc_x  * s_accel_scale;
    p_data->acc_y   = (float)raw_acc_y  * s_accel_scale;
    p_data->acc_z   = (float)raw_acc_z  * s_accel_scale;
    p_data->gyro_x  = (float)raw_gyro_x * s_gyro_scale;
    p_data->gyro_y  = (float)raw_gyro_y * s_gyro_scale;
    p_data->gyro_z  = (float)raw_gyro_z * s_gyro_scale;

    return true;
}

bool icm42688p_read_accel(icm42688p_data_t *p_data)
{
    uint8_t buf[6] = {0};
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;

    if (!p_data) return false;
    if (!icm42688p_read_reg(REG_ACCEL_DATA_X1, buf, sizeof(buf))) {
        return false;
    }

    raw_acc_x = ((int16_t)buf[0] << 8) | buf[1];
    raw_acc_y = ((int16_t)buf[2] << 8) | buf[3];
    raw_acc_z = ((int16_t)buf[4] << 8) | buf[5];

    p_data->acc_x = (float)raw_acc_x * s_accel_scale;
    p_data->acc_y = (float)raw_acc_y * s_accel_scale;
    p_data->acc_z = (float)raw_acc_z * s_accel_scale;
    p_data->gyro_x = 0.0f;
    p_data->gyro_y = 0.0f;
    p_data->gyro_z = 0.0f;
    p_data->temp_c = 0.0f;
    return true;
}

bool icm42688p_read_motion(icm42688p_data_t *p_data)
{
    uint8_t buf[12] = {0};
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    if (!p_data) return false;
    if (!icm42688p_read_reg(REG_ACCEL_DATA_X1, buf, sizeof(buf))) {
        return false;
    }

    raw_acc_x  = ((int16_t)buf[0]  << 8) | buf[1];
    raw_acc_y  = ((int16_t)buf[2]  << 8) | buf[3];
    raw_acc_z  = ((int16_t)buf[4]  << 8) | buf[5];
    raw_gyro_x = ((int16_t)buf[6]  << 8) | buf[7];
    raw_gyro_y = ((int16_t)buf[8]  << 8) | buf[9];
    raw_gyro_z = ((int16_t)buf[10] << 8) | buf[11];

    p_data->acc_x  = (float)raw_acc_x * s_accel_scale;
    p_data->acc_y  = (float)raw_acc_y * s_accel_scale;
    p_data->acc_z  = (float)raw_acc_z * s_accel_scale;
    p_data->gyro_x = (float)raw_gyro_x * s_gyro_scale;
    p_data->gyro_y = (float)raw_gyro_y * s_gyro_scale;
    p_data->gyro_z = (float)raw_gyro_z * s_gyro_scale;
    p_data->temp_c = 0.0f;
    return true;
}
