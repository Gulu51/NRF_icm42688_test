#ifndef __LIS2DW12_PORT_H__
#define __LIS2DW12_PORT_H__

#include "lis2dw12_reg.h"
#include "nrf_drv_gpiote.h"
#include "MyLog.h"
// TWI驱动程序实例ID,ID和外设编号对应，0:TWI0 1:TWI1
#define LIS2DW12_TWI_INSTANCE_ID            0

// #define LIS2DW12_CTRL1_IO                6       // 控制1引脚  
// #define LIS2DW12_CTRL2_IO                7       // 控制2引脚  

#define LIS2DW12_CTRL_IO                    29       // 控制引脚
#define LIS2DW12_INT1_IO                    8       // INT引脚  
#define LIS2DW12_INT2_IO                    6       // INT引脚  

#define LIS2DW12_TWI_SCL_IO                31      // 时钟线引脚  
#define LIS2DW12_TWI_SDA_IO                30       // 数据线引脚   //mini版

#define LIS2DW12_TWI_MAX_NUM_TX_BYTES            32      // TWI TX buffer size
#define LIS2DW12_TWI_TIMEOUT                     10000 

#define LIS2DW12_TWI_ADDR  (LIS2DW12_I2C_ADD_H>>1) //内部默认SA上拉

extern stmdev_ctx_t g_lis2dw12_dev_ctx;
void LIS2DW12_I2C_Init(void);
void LIS2DW12_I2C_Enable(void);
void LIS2DW12_I2C_Disable(void);
void LIS2DW12_IntInit(uint32_t pin,nrfx_gpiote_evt_handler_t callback);
void LIS2DW12_IntEnable(uint32_t pin);
void LIS2DW12_IntDisable(uint32_t pin);
void Lis2dw12_para_set(void);
void Lis2dw12_data_send(void);
#endif

