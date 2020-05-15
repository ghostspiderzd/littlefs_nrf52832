/*******************************************************************************
-------------------------------------------------------------------------------
* @file    ADS1292R.C
* @author  Ye Junjie
* @date    2018-12-16  
*******************************************************************************/
#include "ADS1292R.h"
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
uint8_t ADS_RDATE_BUFFER[20];
uint8_t ADS_TDATE_BUFFER[20];
extern DataPacket_t   DataPack;
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

/*ADS1292R REG_SETTING*/
//uint8_t 	ADS1292R_Default_Register_Settings[15] = {

//	//Device ID read Ony
//		0x00,
//   	//CONFIG1
//		0x02,
//    //CONFIG2
//     0xE0,
//    //LOFF
//     0xF0,
//	 //CH1SET (PGA gain = 6)
//     0x00,
//	 //CH2SET (PGA gain = 6)
//     0x00,
//	 //RLD_SENS (default)
//	 0x2C,
//	 //LOFF_SENS (default)
//	 0x0F,    
//    //LOFF_STAT
//     0x00,
//    //RESP1
//     0xEA,
//	//RESP2
//	 0x03,
//	//GPIO
//     0x0C 
//};	

/*ADS1292R_SPI_ENENT*/
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.\r\n");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received: \r\n");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}

/*ADS1292 SPI INIT*/
void ADS_SPI_INIT(void)
{
	
		/*ADS_1292R相关IO口设置#define  ADS_CLKSEL  ADS_RESET  ADS_DRAY   ADS_COLCK  ADS_STAT	*/
		nrf_gpio_cfg_output(ADS_CLKSEL);															//选择内部时钟
		nrf_gpio_cfg_input(ADS_COLCK,NRF_GPIO_PIN_NOPULL);						//使用内部时钟将时钟线控制
		nrf_gpio_cfg_input(ADS_DRAY,NRF_GPIO_PIN_NOPULL);							//就绪端低电平有效
		nrf_gpio_cfg_output(ADS_RESET);																//选择复位接口
		nrf_gpio_cfg_output(ADS_STAT);																//转换使能端
		nrf_gpio_cfg_output(SPI_SS_PIN);															//配置片选接口
	
		nrf_gpio_pin_set(ADS_RESET);	
		nrf_gpio_pin_set(ADS_CLKSEL);
		nrf_gpio_pin_set(ADS_STAT);	
		nrf_gpio_pin_set(SPI_SS_PIN);	
		ADS_MOD_RESET();
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
		spi_config.mode =	NRF_DRV_SPI_MODE_2;
		spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		/*等待配置完成*/
		//nrf_delay_ms(2000);
}
/*ADS1292R */
void ADS_SPI_Command_date(uint8_t command)
{
		uint8_t retvar[2],txvar[2];
		nrf_delay_ms(2);
		txvar[0] = command;
		nrf_gpio_pin_clear(SPI_SS_PIN);	
		nrf_delay_ms(2);
		nrf_drv_spi_transfer(&spi, txvar, 1, retvar, 1);
		nrf_gpio_pin_set(SPI_SS_PIN);	
		
}
/*ADS1292R */
void ADS_SPI_WREG_Command(uint8_t Start_Address,uint8_t Wreg_Date)
{
		uint8_t SPI_TX_BUF[3],SPI_RX_BUF[3];
		SPI_TX_BUF[0] = ADS1292R_WREG|Start_Address;
		SPI_TX_BUF[1] = 0x00;
		SPI_TX_BUF[2] = Wreg_Date;
		nrf_delay_ms(2);
		nrf_gpio_pin_clear(SPI_SS_PIN);	
		nrf_delay_ms(2);
		nrf_drv_spi_transfer(&spi, SPI_TX_BUF, 3,SPI_RX_BUF, 3);
		nrf_gpio_pin_set(SPI_SS_PIN);			
}

/*ADS1292R */
uint8_t ADS_SPI_RREG_Command(uint8_t Start_Address)
{
		uint8_t SPI_TX_BUF[3],SPI_RX_BUF[3];
		SPI_TX_BUF[0] = Start_Address|ADS1292R_RREG;
		SPI_TX_BUF[1] = 0x00;
		SPI_TX_BUF[2] = 0x00;
		nrf_delay_ms(2);
		nrf_gpio_pin_clear(SPI_SS_PIN);	
		nrf_delay_ms(2);
		nrf_drv_spi_transfer(&spi, SPI_TX_BUF, 3,SPI_RX_BUF, 3);
		nrf_gpio_pin_set(SPI_SS_PIN);	
		return SPI_RX_BUF[2];
}

/*ADS1292R_RESET*/
void ADS_MOD_RESET(void)
{
		nrf_delay_ms(10);
		nrf_gpio_pin_clear(ADS_RESET);											//复位端低电平有效
		nrf_delay_ms(1000);
		nrf_gpio_pin_set(ADS_RESET);
}
/*ADS1292R_CONFIG */
void ADS_Default_Register_Settings(void)
{
	  uint8_t rx_buff[20];
		uint8_t ADS1292R_Default_Settings[11] = 
		{
				//REG_START_ADDRESS
				 0x42,
				//WRREG NUM
				 0x08,
			
				//CONFIG2
				 0xE0,
				//LOFF
				 0xF0,
			 //CH1SET (PGA gain = 6)
				 0x20,
			 //CH2SET (PGA gain = 6)
				 0x10,
			 //RLD_SENS (default)
//				0x3c,
			0x2C,
			 //LOFF_SENS (default)
				 0x0f,    
				//LOFF_STAT
				 0x00,
				//RESP1
//				 0xe2,
					0xEA,

				//RESP2
				 0x03,
		};
		nrf_delay_ms(2);
		nrf_gpio_pin_clear(SPI_SS_PIN);	
		nrf_delay_ms(2);
		nrf_drv_spi_transfer(&spi, ADS1292R_Default_Settings, 11, rx_buff, 11);
		nrf_gpio_pin_set(SPI_SS_PIN);
		nrf_delay_ms(2);		
}
/**********************************************************************************************************
* STOP ADS1292R_READ_DATA 			                                          						  *
**********************************************************************************************************/

void Stop_Read_Data_Continuous (void)
{
   ADS_SPI_Command_date(ADS1292R_SDATAC);					// Send 0x11 to the ADS1x9x
	
}

/**********************************************************************************************************
* START ADS1292R_READ_DATA 			                                          						  *
**********************************************************************************************************/

void Start_Read_Data_Continuous (void)
{
   ADS_SPI_Command_date(ADS1292R_RDATAC);					// Send 0x10 to the ADS1x9x
}
/**********************************************************************************************************
* ADS1292R_READ_DATA_ONCE 			                                          						  *
**********************************************************************************************************/
/*每次读取一次通道状态+CH1\CH2数值*/
void Read_Data_Once(void)
{
		
		memset(ADS_TDATE_BUFFER,0,20);
		ADS_TDATE_BUFFER[0] = ADS1292R_RDATA;
		nrf_drv_spi_transfer(&spi,ADS_TDATE_BUFFER, 10, ADS_RDATE_BUFFER+8,10);
//		ADS_RDATE_BUFFER;
		
		
}

void ADS_ID_PAIR(void)
{
		memset(ADS_TDATE_BUFFER,0,20);
		ADS_TDATE_BUFFER[0] = 0x20;
//		ADS_TDATE_BUFFER[1] = 0x00;
//		ADS_TDATE_BUFFER[2] = 0x00;
		while( ADS_RDATE_BUFFER[2] != 0x73)
			{
					
					nrf_delay_ms(2);
					nrf_gpio_pin_clear(SPI_SS_PIN);	
					nrf_delay_ms(2);
					nrf_drv_spi_transfer(&spi,ADS_TDATE_BUFFER, 3, ADS_RDATE_BUFFER, 3);
					nrf_gpio_pin_set(SPI_SS_PIN);	
			}
}
void ADS_DATA_AVG()
{
		uint8_t     DATA_H,DATA_L;
		uint16_t		ads_avg = 0,rem = 0,medin = 0,i;
//		float ads_avg=0.0;
		for(i = 0; i < 96*2;i+=2)
		{
					medin = DataPack.EcgData.L1Value[i]*256+(DataPack.EcgData.L1Value[i+1])+rem;
					ads_avg+=medin/96;
					rem = medin%96;
		}
//			DataPack.RefData.Value = ads_avg;
		DATA_H= (ads_avg)&0xff;
		DATA_L= ((ads_avg)>>8)&0xff;
		DataPack.RefData.Value = DATA_H*256+DATA_L;
}
