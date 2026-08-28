/*********************************************************************
 * INCLUDES
 */
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "string.h"
#include "mpu6050_port.h"
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "lis2dw12_reg.h"

static void twi_handleEvent(nrf_drv_twi_evt_t const *pEvent, void *pContext);
static void mergeRegisterAndData(uint8_t *pTxBuf, uint8_t regAddr, uint8_t *pData, uint16_t dataLen);

/*********************************************************************
 * LOCAL VARIABLES
 */
static  nrf_drv_twi_t s_twiHandle = NRF_DRV_TWI_INSTANCE(MPU6050_TWI_INSTANCE_ID);

volatile static bool s_twiTxDone = false;                   // 发送完成标志
volatile static bool s_twiRxDone = false;                   // 接收完成标志	

static uint8_t s_twiWriteDataBuffer[MPU6050_TWI_MAX_NUM_TX_BYTES];

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief TWI(I2C)驱动初始化
 @param 无
 @return 无
*/
void MPU6050_I2C_Init(void)
{
    uint32_t errCode;
	
    // 初始化TWI配置结构体
    const nrf_drv_twi_config_t twiConfig =
    {
        .scl                = MPU6050_TWI_SCL_IO,             // 配置TWI SCL引脚
        .sda                = MPU6050_TWI_SDA_IO,             // 配置TWI SDA引脚
        .frequency          = NRF_DRV_TWI_FREQ_400K,            // 配置TWI时钟频率
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH         // TWI中断优先级设置
    };
    
    // 初始化TWI
    errCode = nrf_drv_twi_init(&s_twiHandle, &twiConfig, twi_handleEvent, NULL);
    APP_ERROR_CHECK(errCode);
    
    // 使能TWI
    nrf_drv_twi_enable(&s_twiHandle);
}

/**
 @brief TWI(I2C)写数据函数
 @param slaveAddr -[in] 从设备地址
 @param regAddr -[in] 寄存器地址
 @param pData -[in] 写入数据
 @param dataLen -[in] 写入数据长度
 @return 错误码
*/
uint16_t MPU6050_I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t errCode;
    uint32_t timeout = MPU6050_TWI_TIMEOUT;
    // Merging MPU register address and p_data into one buffer.
    mergeRegisterAndData(s_twiWriteDataBuffer, regAddr, pData, dataLen);
    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xferDesc;
    xferDesc.address = slaveAddr;
    xferDesc.type = NRF_DRV_TWI_XFER_TX;
    xferDesc.primary_length = dataLen + 1;
    xferDesc.p_primary_buf = s_twiWriteDataBuffer;

    // Transferring
    errCode = nrf_drv_twi_xfer(&s_twiHandle, &xferDesc, 0);

    while((!s_twiTxDone) && --timeout);
    if(!timeout)
    {
        return NRF_ERROR_TIMEOUT;
    }
    s_twiTxDone = false;

    return errCode;	
}

/**
 @brief TWI(I2C)读数据函数
 @param slaveAddr -[in] 从设备地址
 @param regAddr -[in] 寄存器地址
 @param pData -[in] 读出数据
 @param dataLen -[in] 读出数据长度
 @return 错误码
*/
uint16_t MPU6050_I2C_ReadData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    uint32_t errCode;
    uint32_t timeout = MPU6050_TWI_TIMEOUT;

    errCode = nrf_drv_twi_tx(&s_twiHandle, slaveAddr, &regAddr, 1, false);
    //dummy read 第一个字节数据，写入reg地址，fasle表示不要停止位
    if(errCode != NRF_SUCCESS)
    {
        return errCode;
    }

    while((!s_twiTxDone) && --timeout);	
    if(!timeout)
    {
        return NRF_ERROR_TIMEOUT;
    }
    s_twiTxDone = false;

    errCode = nrf_drv_twi_rx(&s_twiHandle, slaveAddr, pData, dataLen);
    if(errCode != NRF_SUCCESS)
    {
        return errCode;
    }

    timeout = MPU6050_TWI_TIMEOUT;
    while((!s_twiRxDone) && --timeout);
    if(!timeout)
    {
        return NRF_ERROR_TIMEOUT;
    }
    s_twiRxDone = false;
	
    return errCode;
}

/**
 @brief 开启TWI(I2C)
 @param 无
 @return 无
*/
void MPU6050_I2C_Enable(void)
{
    nrf_drv_twi_enable(&s_twiHandle);
}

/**
 @brief 禁用TWI(I2C)
 @param 无
 @return 无
*/
void MPU6050_I2C_Disable(void)
{
    nrf_drv_twi_disable(&s_twiHandle);
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 @brief TWI事件处理函数
 @param pEvent -[in] TWI事件
 @return 无
*/
static void twi_handleEvent(nrf_drv_twi_evt_t const *pEvent, void *pContext)
{
    // 判断TWI事件类型
    switch(pEvent->type)
    {
        // 传输完成事件
        case NRF_DRV_TWI_EVT_DONE:
            switch(pEvent->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    s_twiTxDone = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    s_twiTxDone = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    s_twiRxDone = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    s_twiRxDone = true;
                    break;
                default:
                    break;
            }
            break;
		
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
		
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
		
        default:
            break;
    }	
}

/**
 @brief 合并寄存器地址和待写入数据
 @param pTxBuf -[in] 写入缓冲区
 @param regAddr -[in] 寄存器地址
 @param pData -[in] 写入数据
 @param dataLen -[in] 写入数据长度
 @return 无
*/
static void mergeRegisterAndData(uint8_t *pTxBuf, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    pTxBuf[0] = regAddr;
    memcpy((pTxBuf + 1), pData, dataLen);
}

void MPU6050_IntInit(uint32_t pin,nrfx_gpiote_evt_handler_t callback)
{
    ret_code_t err_code;
    if (!nrf_drv_gpiote_is_init())
    {// GPIOE驱动初始化，如有其它GPIO中断只调用一次
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    nrf_drv_gpiote_in_config_t inConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false); // 默认初始化，双边沿、上拉中断触发，false表示非high accuracy
    inConfig.pull = NRF_GPIO_PIN_PULLUP;      //上拉输入						
    inConfig.sense = NRF_GPIOTE_POLARITY_HITOLO;// 下降沿

    err_code = nrf_drv_gpiote_in_init(pin, &inConfig, callback);
    APP_ERROR_CHECK(err_code);				
}

void MPU6050_IntEnable(uint32_t pin)
{
    nrf_drv_gpiote_in_event_enable(pin, true);	
}
void MPU6050_IntDisable(uint32_t pin)
{
    nrf_drv_gpiote_in_event_disable(pin);	
}



void platform_delay(uint32_t ms)
{

    nrf_delay_ms(ms);
}
void platform_init(void)
{
    uint32_t errCode;
    // 初始化TWI配置结构体
    const nrf_drv_twi_config_t twiConfig =
    {
        .scl                = MPU6050_TWI_SCL_IO,             // 配置TWI SCL引脚
        .sda                = MPU6050_TWI_SDA_IO,             // 配置TWI SDA引脚
        .frequency          = NRF_DRV_TWI_FREQ_400K,            // 配置TWI时钟频率
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH         // TWI中断优先级设置
			
    };
    
    // 初始化TWI
    errCode = nrf_drv_twi_init(&s_twiHandle, &twiConfig, twi_handleEvent, NULL);
    APP_ERROR_CHECK(errCode);
    
    // 使能TWI
    nrf_drv_twi_enable(&s_twiHandle);
		platform_delay(20);
}
// 写函数（确保时序严格）
static int32_t  platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
	{
//    uint32_t errCode = NRF_SUCCESS;
//    uint32_t timeout = LIS2DW12_TWI_TIMEOUT;

//    errCode = nrf_drv_twi_tx(&s_twiHandle, slaveAddr, &regAddr, 1, false);
//    //dummy read 第一个字节数据，写入reg地址，fasle表示不要停止位
//    if(errCode != NRF_SUCCESS)
//    {
//        return errCode;
//    }

//    while((!s_twiTxDone) && --timeout);	
//    if(!timeout)
//    {
//        return NRF_ERROR_TIMEOUT;
//    }
//    s_twiTxDone = false;

//    errCode = nrf_drv_twi_rx(&s_twiHandle, slaveAddr, pData, dataLen);
//    if(errCode != NRF_SUCCESS)
//    {
//        return errCode;
//    }

//    timeout = LIS2DW12_TWI_TIMEOUT;
//    while((!s_twiRxDone) && --timeout);
//    if(!timeout)
//    {
//        return NRF_ERROR_TIMEOUT;
//    }
//    s_twiRxDone = false;
//	
//    return errCode;
}

/**
  * @brief  Read generic device register (platform dependent) - NRF52832版本
  * @param  handle  通信接口句柄（TWI/SPI实例指针）
  * @param  reg     要读取的寄存器地址
  * @param  bufp    数据存储缓冲区指针
  * @param  len     要读取的数据长度
  * @retval 0成功，非0失败
  */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{

    uint32_t errCode = NRF_SUCCESS;
//  uint32_t timeout = LIS2DW12_TWI_TIMEOUT;

    errCode = nrf_drv_twi_tx(&s_twiHandle, LIS2DW12_I2C_ADD_H, &reg, 1, true);
    //dummy read 第一个字节数据，写入reg地址，fasle表示不要停止位
    if(errCode != NRF_SUCCESS)
    {		
			MY_LOG_DEBUG("write_ fall");
        return errCode;
    }
    errCode = nrf_drv_twi_rx(&s_twiHandle, LIS2DW12_I2C_ADD_H, bufp, len);
    if(errCode != NRF_SUCCESS)
    {
      MY_LOG_DEBUG("read_ fall");  
			return errCode;
    }

    return 0;
}


void i2c_scan(void) {
    MY_LOG_DEBUG("Scanning I2C bus...");
    for(uint8_t addr = 0x08; addr <= 0x77; addr++) {
        ret_code_t err = nrf_drv_twi_rx(&s_twiHandle, addr, NULL, 0);
        if(err == NRF_SUCCESS) {
            MY_LOG_DEBUG("Found device at 0x%02X", addr);
        }
    }
}
void lis2dw12_init()
	{
		uint8_t whoamI,ret;
		/* Initialize mems driver interface */
		stmdev_ctx_t dev_ctx;
		dev_ctx.write_reg = platform_write;
		dev_ctx.read_reg = platform_read;
		dev_ctx.mdelay = platform_delay;
		dev_ctx.handle = &s_twiHandle;
		/* Initialize platform specific hardware */
		platform_init();
		/* Wait sensor boot time */
		platform_delay(BOOT_TIME);
		/* Check device ID */
//		i2c_scan();
		lis2dw12_device_id_get(&dev_ctx, &whoamI);

		MY_LOG_DEBUG("WHO_AM_I read attempt, ret=%d, value=0x%02X", ret, whoamI);

		if (whoamI != LIS2DW12_ID) {
				MY_LOG_DEBUG("Expected ID: 0x%02X", LIS2DW12_ID);
				// 尝试扫描I2C总线
//				i2c_scan();
				while(1);

		}
		
	}

		
/****************************************************END OF FILE****************************************************/
