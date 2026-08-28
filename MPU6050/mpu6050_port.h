#ifndef __MPU6050_PORT_H__
#define __MPU6050_PORT_H__
/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"
#include "nrf_drv_gpiote.h"
#include "lis2dw12_reg.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "MyLog.h"
/*********************************************************************
 * DEFINITIONS
 */
// TWI驱动程序实例ID,ID和外设编号对应，0:TWI0 1:TWI1
#define MPU6050_TWI_INSTANCE_ID                 0

#define MPU6050_SWITCH_IO               4       // SWITCH引脚
#define MPU6050_INT_IO                  8       // INT引脚  
// #define MPU6050_TWI_SCL_IO                15       // 时钟线引脚  
// #define MPU6050_TWI_SDA_IO                16       // 数据线引脚  //测试版

#define MPU6050_TWI_SCL_IO                25       // 时钟线引脚  
#define MPU6050_TWI_SDA_IO                27       // 数据线引脚   //mini版

#define MPU6050_TWI_MAX_NUM_TX_BYTES            32      // TWI TX buffer size
#define MPU6050_TWI_TIMEOUT                     60000 
#define BOOT_TIME            20 //ms
#define SENSOR_BUS  &s_twiHandle  // I2C版本
/*********************************************************************
 * API FUNCTIONS
 */
void MPU6050_I2C_Init(void);
uint16_t MPU6050_I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);
uint16_t MPU6050_I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);
void MPU6050_I2C_Enable(void);
void MPU6050_I2C_Disable(void);
void MPU6050_IntInit(uint32_t pin,nrfx_gpiote_evt_handler_t callback);
void MPU6050_IntEnable(uint32_t pin);
void MPU6050_IntDisable(uint32_t pin);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                             uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                            uint16_t len);
void platform_delay(uint32_t ms);
void platform_init(void);
void lis2dw12_init(void);

void minimal_test(void);

#endif
