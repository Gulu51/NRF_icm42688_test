#include "app_util_platform.h"
//#include "nrf_gpio.h"
#include "max30102_port.h"
#include "app_util_platform.h"
//#include "nrf_gpio.h"
#include "arm_math.h"
#include "nrf_drv_twi.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//¶¨ÒåI2CÒý½Å
#define TWI_SCL_M      16    //SCLÒý½Å
#define TWI_SDA_M      15     //SDAÒý½Å
#define TWI_INSTANCE_ID     0


//TWIÇý¶¯³ÌÐòÊµÀýID
uint16_t fifo_red;  //定义FIFO中的红光数据
uint16_t fifo_ir;   //定义FIFO中的红外光数据
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

//TWIÇý¶¯³ÌÐòÊµÀý£¬IDÎª1¶ÔÓ¦TWI1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWIÊÂ¼þ´¦Àíº¯Êý
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
				
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
					  nrf_gpio_pin_clear(LED_2);
					  break;
				case NRF_DRV_TWI_EVT_DATA_NACK:
					  nrf_gpio_pin_clear(LED_1);		
						
        default:
            break;
    }
}

void twi_master_init(void)
{
    ret_code_t err_code;
    //¶¨Òå²¢³õÊ¼»¯TWIÅäÖÃ½á¹¹Ìå
    const nrf_drv_twi_config_t twi_max30102_config = {
       .scl                = TWI_SCL_M,  //¶¨ÒåTWI SCLÒý½Å
       .sda                = TWI_SDA_M,  //¶¨ÒåTWI SDAÒý½Å
       .frequency          = NRF_DRV_TWI_FREQ_100K, //TWIËÙÂÊ
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWIÓÅÏÈ¼¶
       .clear_bus_init     = false//³õÊ¼»¯ÆÚ¼ä²»·¢ËÍ9¸öSCLÊ±ÖÓ
    };
    //³õÊ¼»¯TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_max30102_config, twi_handler, NULL);
	  //¼ì²é·µ»ØµÄ´íÎó´úÂë
    APP_ERROR_CHECK(err_code);
    //Ê¹ÄÜTWI
    nrf_drv_twi_enable(&m_twi);
}

//Ð´MAX30102¼Ä´æÆ÷
void max30102_write_reg(uint8_t reg,uint8_t val)
{
	uint8_t tx_packet[2];
	
	tx_packet[0] = reg;
	tx_packet[1] = val;
	
	m_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, tx_packet, sizeof(tx_packet),false);
  APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}
//¶ÁMAX30102¼Ä´æÆ÷
uint8_t max30102_read_reg(uint8_t reg,uint8_t len)
{
	  ret_code_t err_code;
	  uint8_t tx_packet[1];
	  uint8_t rx_packet;
	  tx_packet[0] = reg;
	
	  m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, tx_packet, sizeof(tx_packet),false);
    APP_ERROR_CHECK(err_code);
	  while (m_xfer_done == false){};

		m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDRESS, &rx_packet, 1);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false){};
			
		return rx_packet;
}
//¶ÁMAX30102 FIFO
bool max30102_fifo_read(float *output_data)
{
	uint16_t*p_red_led = &fifo_red;
	uint16_t*p_ir_led  =  &fifo_ir;
	ret_code_t err_code;
	uint32_t un_temp;
	//  unsigned char uch_temp;
	uint8_t tx_packet[1];
	*p_red_led=0;
	*p_ir_led=0;
	//char ach_i2c_data[6];
	char ach_i2c_data[6];

	//¶Á³öÇå³ý×´Ì¬¼Ä´æÆ÷
	(void)max30102_read_reg(REG_INTR_STATUS_1, 1);
	(void)max30102_read_reg(REG_INTR_STATUS_2, 1);

	//Ð´
	tx_packet[0] = REG_FIFO_DATA;
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, tx_packet, sizeof(tx_packet),false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false){};
	//¶Á
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDRESS, ach_i2c_data, 6);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false){};
	
	un_temp=(unsigned char) ach_i2c_data[0];
	  un_temp<<=16;
	  *p_red_led+=un_temp;
	  un_temp=(unsigned char) ach_i2c_data[1];
	  un_temp<<=8;
	  *p_red_led+=un_temp;
	  un_temp=(unsigned char) ach_i2c_data[2];
	  *p_red_led+=un_temp;
	  
	  un_temp=(unsigned char) ach_i2c_data[3];
	  un_temp<<=16;
	  *p_ir_led+=un_temp;
	  un_temp=(unsigned char) ach_i2c_data[4];
	  un_temp<<=8;
	  *p_ir_led+=un_temp;
	  un_temp=(unsigned char) ach_i2c_data[5];
	  *p_ir_led+=un_temp;
	  *p_red_led&=0x03FFFF;  //Mask MSB [23:18]
	  *p_ir_led&=0x03FFFF;  //Mask MSB [23:18]
	
	*output_data = *p_red_led;
	*(output_data+1) = *p_ir_led;


	NRF_LOG_INFO("FIFO ok");	
	return true;
}
void max30102_reset(void)
{
   max30102_write_reg(REG_MODE_CONFIG,0x40);
}


void MAX30102_Init(void)
{   

	  //Ê¹ÄÜA_FULL_EN ºÍ PPG_RDY_EN ÖÐ¶Ï
		max30102_write_reg(REG_INTR_ENABLE_1,0xc0); 

		max30102_write_reg(REG_INTR_ENABLE_2,0x00);

		max30102_write_reg(REG_FIFO_WR_PTR,0x00);  //FIFO_WR_PTR[4:0]

		max30102_write_reg(REG_OVF_COUNTER,0x00);  //OVF_COUNTER[4:0]

		max30102_write_reg(REG_FIFO_RD_PTR,0x00); //FIFO_RD_PTR[4:0]

		max30102_write_reg(REG_FIFO_CONFIG,0x0f);  //sample avg = 1, fifo rollover=false, fifo almost full = 17

		max30102_write_reg(REG_MODE_CONFIG,0x02);  //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED

		max30102_write_reg(REG_SPO2_CONFIG,0x00);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
		
		max30102_write_reg(REG_LED1_PA,0x01);   //Choose value for ~ 7mA for LED1

		max30102_write_reg(REG_LED2_PA,0x00);  // Choose value for ~ 7mA for LED2

		max30102_write_reg(REG_PILOT_PA,0x01);   // Choose value for ~ 25mA for Pilot LED

}


uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums)
{
		float input_data_sum_aver = 0;
		uint16_t i,temp;
		
		
		for(i=0;i<cache_nums;i++)
		{
		input_data_sum_aver += *(input_data+i);
		}
		input_data_sum_aver = input_data_sum_aver/cache_nums;
		for(i=0;i<cache_nums;i++)
		{
				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
				{
					temp = i;
					break;
				}
		}
		i++;
		for(;i<cache_nums;i++)
		{
				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
				{
					temp = i - temp;
					break;
				}
		}
		if((temp>14)&&(temp<100))
		{
			return 3000/temp;
		}
		else
		{
			return 0;
		}
}

float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums)
{
			float ir_max=*ir_input_data,ir_min=*ir_input_data;
			float red_max=*red_input_data,red_min=*red_input_data;
			float R;
			uint16_t i;
			for(i=1;i<cache_nums;i++)
			{
				if(ir_max<*(ir_input_data+i))
				{
					ir_max=*(ir_input_data+i);
				}
				if(ir_min>*(ir_input_data+i))
				{
					ir_min=*(ir_input_data+i);
				}
				if(red_max<*(red_input_data+i))
				{
					red_max=*(red_input_data+i);
				}
				if(red_min>*(red_input_data+i))
				{
					red_min=*(red_input_data+i);
				}
			}
			
			 R=((ir_max+ir_min)*(red_max-red_min))/((red_max+red_min)*(ir_max-ir_min));
			return ((-45.060)*R*R + 30.354*R + 94.845);
}