#ifndef __MAX30102_PORT_H__
#define __MAX30102_PORT_H__

#include "nrf_drv_gpiote.h"
#include "MyLog.h"
#include "nrf_delay.h"
#include <stdint.h>
// TWI驱动程序实例ID,ID和外设编号对应，0:TWI0 1:TWI1
#define PORT_TWI_INSTANCE_ID            0
#define TWI_INSTANCE_ID     1

// #define LIS2DW12_CTRL1_IO                6       // 控制1引脚  
// #define LIS2DW12_CTRL2_IO                7       // 控制2引脚  

#define PORT_CTRL_IO                    4       // 控制引脚
#define PORT_INT1_IO                    8       // INT引脚  
#define PORT_INT2_IO                    6       // INT引脚  

#define PORT_TWI_SCL_IO                16       // 时钟线引脚  
#define PORT_TWI_SDA_IO                15       // 数据线引脚   //mini版

#define PORT_TWI_MAX_NUM_TX_BYTES            32      // TWI TX buffer size
#define PORT_TWI_TIMEOUT                     10000 

//设备地址，右移一位去掉读写位
#define MAX30102_ADDRESS 0x57
#define max30102_WR_address (0xAE>>1)
//register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

#define I2C_WRITE_ADDR 0xAE
#define I2C_READ_ADDR 0xAF

#define INTERRUPT_STATUS1 0X00
#define INTERRUPT_STATUS2 0X01
#define INTERRUPT_ENABLE1 0X02
#define INTERRUPT_ENABLE2 0X03

#define FIFO_WR_POINTER 0X04
#define FIFO_OV_COUNTER 0X05
#define FIFO_RD_POINTER 0X06
#define FIFO_DATA 0X07

#define FIFO_CONFIGURATION 0X08
#define MODE_CONFIGURATION 0X09
#define SPO2_CONFIGURATION 0X0A
#define LED1_PULSE_AMPLITUDE 0X0C
#define LED2_PULSE_AMPLITUDE 0X0D

#define MULTILED1_MODE 0X11
#define MULTILED2_MODE 0X12

#define TEMPERATURE_INTEGER 0X1F
#define TEMPERATURE_FRACTION 0X20
#define TEMPERATURE_CONFIG 0X21

#define VERSION_ID 0XFE
#define PART_ID 0XFF

#define FS 100
#define BUFFER_SIZE  (FS* 5) 
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4 // DO NOT CHANGE
#define HAMMING_SIZE  5// DO NOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))
#define SAMPLES_PER_SECOND 					100	//检测频率


void MAX30102_Init(void);
void twi_master_init(void);
void max30102_reset(void);
uint8_t max30102_read_reg(uint8_t reg,uint8_t len);
//bool max30102_read_fifo(void);
bool max30102_fifo_read(float *output_data);
//max30102_fifo_read(max30102_data);		
//// 寄存器读写
//bool max30102_write_register(uint8_t reg, uint8_t value);
//bool max30102_read_register(uint8_t reg, uint8_t *value);

//// FIFO 数据读取
//bool max30102_read_fifo(uint32_t *red, uint32_t *ir);

//心率血氧算法所有函数

 
#endif

