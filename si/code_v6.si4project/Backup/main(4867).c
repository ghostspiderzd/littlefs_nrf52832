/******************************Copyright (c)***********************************
*                Jiangsu Zhihai Electronic Technology Co., Ltd.
*                      Research & Development Department
-------------------------------------------------------------------------------
* @file    main.c
* @author  Gu Dongdong
* @date    2018-10-16  
*******************************************************************************/
#include "BLE_SET.h"
#include "nrf_drv_timer.h"
#include "mpu6050.h"
#include "nrf_delay.h"
#include "twi_master.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "BLE_CRC.h"
#include "nrf_drv_spi.h"
#include "ADS1292R.h"
#include "mlx90615.h"
#include "DATA_FILTER.h"
#include "ble_nus.h"
#include "STM32SPI.h"
#include "Flash_driver.h"
#include "fs_config.h"

extern unsigned char gCmdflag;

unsigned char send_test_flag;

#define PACKET_NUM		888




//int ecg_data_buf[302],ecg_data_new[98];
//int ecg_data_cout = 0,ecg_new_cont=0;
int EcgData_ex,ResData_ex;
double Res_medianValue_F=0,Res_medianValue_S=0,ECG_medianValue = 0;
uint8_t data_first_flag =0;
uint16_t start_cont = 0;
//double sum = 0;
const nrf_drv_timer_t TIMER_ADC   =   NRF_DRV_TIMER_INSTANCE(3);

const nrf_drv_timer_t TIMER_SPI   =   NRF_DRV_TIMER_INSTANCE(4);


uint8_t tempbuff[2];
extern ble_volmeass_t                 m_volmeass;
DataPacket_t   DataPack,DataPack1,DataPack2,DataPack3;

uint8_t send_choose,send_flag = 0;
volatile uint8_t Open_flag = 0;
#define SAMPLES_IN_BUFFER 1             /**< ADC²ÉÑùÊý¾Ý»º´æ´óÐ¡(×Ö½ÚÊý)  */
#define PWR_START_TIME 											3000u  												/*¿ª»ú°´¼ü³ÖÐøÊ±¼äMS*/
#define PWR_SAMP_TIME												50u														/*¿ª»ú°´¼ü²ÉÑùÊ±¼äMS*/
#define	PWR_SAMP_CONT												(PWR_START_TIME/PWR_SAMP_TIME)				/*¿ª»ú¼ü²ÉÑù´ÎÊý*/
APP_TIMER_DEF(m_volmeas_timer_id);          /**< adc²ÉÑù¶¨Ê±Æ÷ */
APP_TIMER_DEF(m_SonPackSend_timer_id);      /**< ·¢°ü¶¨Ê±Æ÷ */
APP_TIMER_DEF(m_Packing_timer_id);          /**< ·¢°ü¶¨Ê±Æ÷ */
APP_TIMER_DEF(m_PWR_ON_OFF_timer_id);       /*°´¼üIO¶¨Ê±¶ÁÈ¡*/
APP_TIMER_DEF(m_BLE_LED_timer_id);					/*À¶ÑÀÖ¸Ê¾µÆ¶¨Ê±Æ÷*/


uint8_t  PWR_Cont = PWR_SAMP_CONT; 						//°´¼üÅÐ¶¨Ê±¼äÊ±¼ä PWR_Cont * 50 = 2000ms
volatile uint8_t  MPU6050_INIT_ERR;  //MPU6050³õÊ¼»¯Ê§°Ü

static nrf_saadc_value_t                    m_buffer_pool[2][SAMPLES_IN_BUFFER];
adc_value_t adc_value;

adc_value_Big_buf_t                         adc_value_Big_buf;//Ë«»º³å·¢ËÍ
adc_value_three_buf_t                         adc_value_Res_buf;//Ë«»º³å·¢ËÍ
imu_value_Big_buf_t                         imu_value_Big_buf;//Ë«»º³å·¢ËÍ
uint8_t ref_adc_buf[4];																				//²âÁ¿µÄ»ù×¼µçÑ¹Öµ
int8_t bat_percent_buf[2]={10,10};												//uint8_t bat_adc_buf[4];//²âÁ¿µÄµç³ØµçÑ¹Öµ
unsigned short temp_va=0;
extern uint32_t saadc_callback_Count;
uint8_t  SonPackCon = 0;
uint8_t  SB = 0,IMU_TICK=0;
uint32_t ADC_TICK=0;
uint16_t  connect_cont = 0;
uint16_t cont_ble = 0,con_dely = 0;
uint8_t t_flag = 0;
LED_STATE m_stable_led_state = BLE_LED_IDLE;
void application_timers_start(void);
void SonPackSend_timers_start(void);
void Packing_timers_start(void);

void application_timers_stop(void);
void SonPackSend_timers_stop(void);
void Packing_timers_stop(void);

void Led_Display(LED_STATE ble_led_id);
static void DataPacking(unsigned char  *pADC_Tab,unsigned char  *pADC2_Tab,accgyr_t* pIMU_Tab);//Êý¾Ý´ò°ü
static void timer_adc_event_handler(nrf_timer_event_t event_type, void* p_context);//TimerÊÂ¼þ»Øµ÷º¯Êý
#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define SPI_BUF_SIXE	192 + 1
#define PACKET_LEN		12


extern uint8_t ADS_RDATE_BUFFER[20];

uint8_t spi1rxbuff[SPI_BUF_SIXE] = 0;
uint8_t spi1txbuff[SPI_BUF_SIXE] = 0;
unsigned char rxflag = 0;

uint8_t usertxbuff[20] = {0x20,0x02,0x00,0x0D};
uint8_t userrxbuff[20] = 0;

uint8_t guserconfig = 0xff;	//ç”¨æˆ·é…ç½®
uint8_t guserdataflag = 0x00;	//ç”¨æˆ·æ•°æ®æ ‡å¿—



volatile unsigned int packetcnt = 0;
extern volatile bool spi1_xfer_done;

void uart_event_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
int8_t bat_va_per(int16_t bat_value_adc)
{
		float bat_value_phy,va_change;
		int8_t bat_value_per;
		bat_value_phy=(float)(bat_value_adc&0x0fff)/413.7374;
		bat_value_per=(int8_t)(1476.4*bat_value_phy-174.31*bat_value_phy*bat_value_phy-3024.5);
		if(bat_value_per<0)
				bat_value_per = 0;
		else if(bat_value_per>100)
				bat_value_per = 100;
		return bat_value_per;
}
//´®¿Ú³õÊ¼»¯
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
 
//À¶ÑÀÖ¸Ê¾µÆÏÔÊ¾³ÌÐò
void Led_Display(LED_STATE ble_led_id)
{
		 switch (ble_led_id)
		 {
			 case BLE_LED_IDLE:
						nrf_gpio_pin_clear(LED_RUN);
						nrf_gpio_pin_clear(LED_BLE_STATE);
						m_stable_led_state = ble_led_id;
						break;
			 
			 case BLE_LED_ADVERTISING:
						if(MPU6050_INIT_ERR)
						{
								nrf_gpio_pin_toggle(LED_RUN);
						}
						else
						{
								nrf_gpio_pin_set(LED_RUN);
						}
						
						nrf_gpio_pin_toggle(LED_BLE_STATE);
						m_stable_led_state = ble_led_id;
						app_timer_start(m_BLE_LED_timer_id, BLE_LED_ADV_INTERVAL, NULL);
						break;
			 case BLE_LED_CONNECTED:
						if(MPU6050_INIT_ERR)
						{
								nrf_gpio_pin_toggle(LED_RUN);
						}
						else
						{
								nrf_gpio_pin_set(LED_RUN);
						}
						nrf_gpio_pin_set(LED_BLE_STATE);
						m_stable_led_state = ble_led_id;
						break;
			 default:
						break;
		 }
		
}

int swapInt32(unsigned int value)
{
	return     ((value & 0x000000FF) << 24) |
               ((value & 0x0000FF00) << 8) |
               ((value & 0x00FF0000) >> 8) |
               ((value & 0xFF000000) >> 24);
}


uint16_t Triangle_wave = 0;	//Êä³öÈý½Ç²¨

//Timer2ÊÂ¼þ»Øµ÷º¯Êý
static void timer_adc_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	   uint16_t Ecg_date = 0,Temp_date = 0;
		 uint32_t Res_date = 0;
		 uint8_t  Temp_h,Temp_l;
		 uint8_t buff_cont;
		unsigned char debug_buff[4] = "test";
		unsigned char debug_buff1[5] = "test1";
	
    switch (event_type)
    {
				case NRF_TIMER_EVENT_COMPARE3:
					  //·­×ªÖ¸Ê¾µÆD2×´Ì¬
//				    NRF_LOG_INFO("Timer2ÊÂ¼þ»Øµ÷º¯Êý\n\r\r");
//				    APP_ERROR_CHECK(nrf_drv_saadc_sample());//adc²ÉÑù
						/*Ò»´ÎÊý¾Ý°ü²É¼¯Ò»´ÎÌåÎÂ*/
					 
				/*if(t_flag == 0)
						{
								con_dely++;
								if(con_dely>610)
								{
									t_flag = 1;
								}
						}
						else*/
						{
//						if(start_cont<500)
//						{
//								start_cont++;
//						}
//						else
//						{
//							 start_cont =3000;
						/*if((!(GetSystemFlag(Read_Temp_FLAG))))
						{
								mlx90615_register_read(0x27, tempbuff,2);
								if(tempbuff[1] != 0xff)
								{
										if(tempbuff[1]>0x48)
										{
												DataPack.TemData.Value = 0x9999;
										}
										else
										{
												Temp_date = (tempbuff[1]*256+tempbuff[0])*2-27315;
												Temp_h = Temp_date&0xff;
												Temp_l = (Temp_date>>8)&0xff;
												temp_va = (Temp_h*256+Temp_l)&0xffff;
										}
										
										SetSystemFlag(Read_Temp_FLAG);
								}
						}
						*/
						//Read_Data_Once();
						/*¶ÁÈ¡ÐÄµçÊý¾Ý*/
//						Ecg_date = ((ADS_RDATE_BUFFER[15]*256 + ADS_RDATE_BUFFER[16]+0x8000)>>4)&0x0fff;
						Ecg_date = ((ADS_RDATE_BUFFER[15]*256 + ADS_RDATE_BUFFER[16]+0x8000)>>1)&0x7fff;
						EcgData_ex = (((ADS_RDATE_BUFFER[15]*65536 + ADS_RDATE_BUFFER[16]*256+ADS_RDATE_BUFFER[17]+0x800000))>>3)&0x001fffff;
						/*¶ÁÈ¡ºôÎüÖµ*/
						Res_date = ((ADS_RDATE_BUFFER[12]*65536 + ADS_RDATE_BUFFER[13]*256+ADS_RDATE_BUFFER[14]+0x800000))&0x00ffffff;
						ResData_ex = Res_date;
						//						EcgData[ecg_new_cont] = Ecg_date;
						/*Ê¹ÓÃEcgData[100],ecgbuf[300],dataLen,ecgDatalenp*/
//						if(ecgDatalenp>299)
//						{
//								ecgDatalenp = 0;
//								ecgfull_flag =1;
//						}		
						 
						if(data_first_flag ==0)
						{
								ECG_medianValue = EcgData_ex;

								Res_medianValue_F = ResData_ex;
								Res_medianValue_S = Res_medianValue_F;
							  data_first_flag = 1;
						}
						else
						{
								ECG_medianValue  = (ECG_medianValue*(WIN_LEN - 1)+ EcgData_ex)/300;
							
							
								Res_medianValue_F = (Res_medianValue_F*(WIN_LEN - 1)+ ResData_ex)/300;
								Res_medianValue_S = (Res_medianValue_S*(WIN_LEN - 1)+ Res_medianValue_F)/300;
//								ResData_ex = Res_medianValue;
//								Res_medianValue = (Res_medianValue*(WIN_LEN - 1)+ ResData_ex)/300;
						}
						
//						if(ecgfull_flag ==1)
//						{
//							sum = sum+EcgData[ecg_new_cont]- ecgbuf[ecgDatalenp];
//							medianValue = (sum/300+0.5);
//						}
//						else
//						{
//							sum = sum+EcgData[ecg_new_cont];
//							medianValue = (sum/(ecgDatalenp+1)+0.5);
//						}
						
//						ecgbuf[ecgDatalenp] = EcgData[ecg_new_cont];
						
						Ecg_date = EcgData_ex+512 - ECG_medianValue;
						Res_date = Res_medianValue_F+32768 - Res_medianValue_S;
//						ecgDatalenp++;
//						ecg_new_cont++;
//						if(ecg_new_cont ==96)
//						{
//								ecg_new_cont =0;	
//								
//								
//							/*Ö±½Óµ÷ÓÃÂË²¨º¯Êý*/
////								medianFilter();//(ecg_data_new,96, ecg_data_buf,ecg_data_cout);
////								medianFilter();
////								medianFilter();
////								VAL_TO_T(EcgData,adc_value_Big_buf.L1Value);
//								
//						}
						/*
            			if((ADS_RDATE_BUFFER[9]&0x01)&&(ADS_RDATE_BUFFER[10]&0x80))
						{
								connect_cont = 0;
								
								SetSystemFlag(Connect_Err_FLAG);
						}
						else
						{
								connect_cont++;
								
								if(connect_cont>600)
								{
									
//									if(GetSystemFlag(Connect_Err_FLAG))
//									{
//											data_first_flag = 0;
//									}
									ClrSystemFlag(Connect_Err_FLAG);
										
										connect_cont = 600;
								}
								else if(connect_cont == 400)
								{
										data_first_flag = 0;
								}
								else
								{
										Ecg_date =512;
										Res_date = 32768;
								}
								
						}			*/
						SetSystemFlag(Connect_Err_FLAG);
//						/*¶ÁÈ¡ºôÎüÖµ*/
//						Res_date = ((ADS_RDATE_BUFFER[12]*65536 + ADS_RDATE_BUFFER[13]*256+ADS_RDATE_BUFFER[14]+0x800000))&0x00ffffff;
						adc_value_Big_buf.Byte_cnt++;
						adc_value_Res_buf.Byte_cnt++;
						adc_value_Res_buf.Byte_cnt++;

						//stm32_spi_conn(spi1txbuff,spi1rxbuff,15);
						
						adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt] = spi1rxbuff[1]; //
						adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt-1] = spi1rxbuff[0];
						DataPack.PPGData.Value[adc_value_Big_buf.Byte_cnt]	 = spi1rxbuff[1];
						DataPack.PPGData.Value[adc_value_Big_buf.Byte_cnt-1]	 = spi1rxbuff[0];

						DataPack.SYSTPData.Value =(unsigned short)(spi1rxbuff[4] << 8);
						DataPack.DIATPData.Value =(unsigned short)(spi1rxbuff[5] << 8);
						DataPack.HeartRateData.Value =(unsigned short)(spi1rxbuff[6] << 8);
						DataPack.RWarePosData.Value = (spi1rxbuff[7]) | (spi1rxbuff[8] << 8) | (spi1rxbuff[9] << 16) | (spi1rxbuff[10] << 24);

					
						
						#if 0
						adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt] = (Ecg_date)&0xff;//(uint8_t)((Ecg_date>>4)&0xff);
						adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt-1] = (Ecg_date>>8)&0xff;//12bitµÄADC ×ª»»³É8bit
						adc_value_Res_buf.L1Value[adc_value_Res_buf.Byte_cnt] = 	0x00;	//Res_date&0xff;//(Res_date)&0xff;
						adc_value_Res_buf.L1Value[adc_value_Res_buf.Byte_cnt-1] = 0x80;
						adc_value_Res_buf.L1Value[adc_value_Res_buf.Byte_cnt-2] = Triangle_wave;
						//adc_value_Res_buf.L1Value[adc_value_Res_buf.Byte_cnt-1] =  (Res_date>>8)&0xff;//(Res_date>>8)&0x0f;
						//adc_value_Res_buf.L1Value[adc_value_Res_buf.Byte_cnt-2] =  (Res_date>>16)&0xff + Triangle_wave;
						Triangle_wave++;
						#endif
						adc_value_Big_buf.Byte_cnt++;
						adc_value_Res_buf.Byte_cnt++;
						
						
						if(++ADC_TICK%3==1) 
						{
							/*
							MPU6050_ReadAcc(&imu_value_Big_buf.Value[IMU_TICK].accx,//¼ÓËÙ¶È3Öá²É¼¯
															&imu_value_Big_buf.Value[IMU_TICK].accy,
															&imu_value_Big_buf.Value[IMU_TICK].accz);
              				MPU6050_ReadGyr(&imu_value_Big_buf.Value[IMU_TICK].gyrx,//ÍÓÂÝÒÇ3Öá²É¼¯
															&imu_value_Big_buf.Value[IMU_TICK].gyry,
															&imu_value_Big_buf.Value[IMU_TICK].gyrz);
							*/
						if(++IMU_TICK==(MotDataNum))IMU_TICK=0;
//							
//							imu_value_Big_buf.Value[IMU_TICK].accx=0xAA;//¼ÓËÙ¶È3Öá²É¼¯
//							imu_value_Big_buf.Value[IMU_TICK].accy=0XBB;
//							imu_value_Big_buf.Value[IMU_TICK].accz=0XCC;
////		 
//							imu_value_Big_buf.Value[IMU_TICK].gyrx=0X11;//ÍÓÂÝÒÇ3Öá²É¼¯
//							imu_value_Big_buf.Value[IMU_TICK].gyry=0X22;
//							imu_value_Big_buf.Value[IMU_TICK].gyrz=0X33;
							
						}
						
						/*Ò»´ÎÊý¾Ý°ü²É¼¯Ò»´ÎµçÁ¿*/
						if((!(GetSystemFlag(Read_Bat_FLAG))))
						{
								//APP_ERROR_CHECK(nrf_drv_saadc_sample());//adc²ÉÑù
								
								
						}
						if((adc_value_Big_buf.Byte_cnt == EcgDataNum))	
						{										
									//ble_send_vol_data(&m_volmeass, debug_buff1 , 5);
									Triangle_wave = 0;
									adc_value_Big_buf.Byte_cnt = 0;
									adc_value_Res_buf.Byte_cnt = 0;
									DataPack.Packetserialnum = swapInt32(DataPack.Packetserialnum);
									DataPack.Packetserialnum++;	//ÐòÁÐºÅ
									DataPack.Packetserialnum = swapInt32(DataPack.Packetserialnum);

									DataPack.RWarePosData.Value = swapInt32(DataPack.RWarePosData.Value);
									DataPack.RWarePosData.Value++;
									DataPack.RWarePosData.Value = swapInt32(DataPack.RWarePosData.Value);
									
							if(0 == send_choose)
							{
									if(GetSystemFlag(Connect_Err_FLAG))
									{	
											DataPack.ErrData.Value |=0x0200;		
											/*ÓÚ12ÔÂ4ÈÕÐÞ¸ÄÌí¼ÓÍÑÂäºóÊä³ö¹Ì¶¨Öµ*/		
											#if 0
											for(buff_cont = 0;buff_cont< EcgDataNum;buff_cont++)
											{
													adc_value_Big_buf.L1Value[buff_cont] = 0x02;
													DataPack.PPGData.Value[buff_cont]	 = 0x02;
													buff_cont++;
													adc_value_Big_buf.L1Value[buff_cont] = 0x00 + Triangle_wave;
													DataPack.PPGData.Value[buff_cont]	 = 0x00 + Triangle_wave;
														
													Triangle_wave++;

													
											}
											#endif
									}
														
									else
											DataPack.ErrData.Value &= ~0x0200;

											Triangle_wave = 0;
											//DataPack.Packetserialnum++;	//ÐòÁÐºÅ
											
											DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);
		//									ADS_DATA_AVG();
											DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
											DataPack.TemData.Value = temp_va; 
																				
											memcpy(&DataPack1,&DataPack,sizeof(DataPack));
											DataPack1.Crc= (crc8((unsigned char *)&DataPack1.IdentPrtc,886, 0)<<8)&0xFF00;
											send_choose =1;
											SetSystemFlag(Send_1ofPack_FLAG);
											
									
							}	
							else if(1 == send_choose)
							{
									if(GetSystemFlag(Connect_Err_FLAG))
									{	
											DataPack.ErrData.Value |=0x0200;		
											/*ÓÚ12ÔÂ4ÈÕÐÞ¸ÄÌí¼ÓÍÑÂäºóÊä³ö¹Ì¶¨Öµ*/		
											#if 0
											for(buff_cont = 0;buff_cont< EcgDataNum;buff_cont++)
											{
													adc_value_Big_buf.L1Value[buff_cont] = 0x02;
													DataPack.PPGData.Value[buff_cont]	 = 0x02;
													buff_cont++;
													adc_value_Big_buf.L1Value[buff_cont] = 0x00 + Triangle_wave;
													DataPack.PPGData.Value[buff_cont]	 = 0x00 + Triangle_wave;
														
													Triangle_wave++;

													
											}
											#endif
									}
				
									else
											DataPack.ErrData.Value &= ~0x0200;

											Triangle_wave = 0;
									
											
											DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);
		//									ADS_DATA_AVG();
											DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
											DataPack.TemData.Value = temp_va; 
																
											memcpy(&DataPack2,&DataPack,sizeof(DataPack));
											DataPack2.Crc= (crc8((unsigned char *)&DataPack2.IdentPrtc,886, 0)<<8)&0xFF00;				
											send_choose =2;
											SetSystemFlag(Send_2ofPack_FLAG);
											
							}	
							else
							{
									
										if(GetSystemFlag(Connect_Err_FLAG))
										{	
												DataPack.ErrData.Value |=0x0200;		
												/*ÓÚ12ÔÂ4ÈÕÐÞ¸ÄÌí¼ÓÍÑÂäºóÊä³ö¹Ì¶¨Öµ*/		
												#if 0
											for(buff_cont = 0;buff_cont< EcgDataNum;buff_cont++)
											{
													adc_value_Big_buf.L1Value[buff_cont] = 0x02;
													DataPack.PPGData.Value[buff_cont]	 = 0x02;
													buff_cont++;
													adc_value_Big_buf.L1Value[buff_cont] = 0x00 + Triangle_wave;
													DataPack.PPGData.Value[buff_cont]	 = 0x00 + Triangle_wave;
														
													Triangle_wave++;

													
											}
											#endif
										}
															
										else
												DataPack.ErrData.Value &= ~0x0200;

												Triangle_wave = 0;
										
												//DataPack.Packetserialnum++; //ÐòÁÐºÅ
												DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);
			//									ADS_DATA_AVG();
												DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
												DataPack.TemData.Value = temp_va; 
																					
												memcpy(&DataPack3,&DataPack,sizeof(DataPack));
												DataPack3.Crc= (crc8((unsigned char *)&DataPack3.IdentPrtc,886, 0)<<8)&0xFF00;
												send_choose =0;
												SetSystemFlag(Send_3ofPack_FLAG);
												
							
						 }
								
						}
//						else if(adc_value_Big_buf.Byte_cnt == (EcgDataNum*2))
//						{
//								adc_value_Big_buf.Byte_cnt = 0;
//								adc_value_Res_buf.Byte_cnt = 0;
//								ClrSystemFlag(ADCBuf_HT_Flag);
//								SetSystemFlag(ADCBuf_TC_Flag); 
////								ClrSystemFlag(Read_Temp_FLAG);
//						}
									

//					}
					}
									break;

							default:
									//Do nothing.
									break;
    }
}


/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/********************************************************************************************************
** º¯ÊýÃû³Æ: DataPacking
** ¹¦ÄÜÃèÊö: Êý¾Ý°ü´ò°ü
********************************************************************************************************/
static void DataPacking(unsigned char  *pADC_Tab,unsigned char  *pADC2_Tab,accgyr_t * pIMU_Tab)
{
  unsigned int i;
  controlcmd_data_strict *pControlcmd =  &gControlcmd;
	for(i=0;i<MotDataNum;i++)
  {   
			DataPack.AccData.xValue[i] =  pIMU_Tab[i].accx;
			DataPack.AccData.yValue[i] =  pIMU_Tab[i].accy;
			DataPack.AccData.zValue[i] =  pIMU_Tab[i].accz;
			DataPack.GyrData.xValue[i] =	pIMU_Tab[i].gyrx;
			DataPack.GyrData.yValue[i] =	pIMU_Tab[i].gyry;
			DataPack.GyrData.zValue[i] =	pIMU_Tab[i].gyrz;
  }
	DataPack.userid = pControlcmd->user_id;
	DataPack.start_timer.Value = pControlcmd->clock;
	memcpy(DataPack.EcgData.L1Value,pADC_Tab,EcgDataNum);
	
	//memcpy(DataPack.ResData.L1Value,pADC2_Tab,ResDataNum);


//		DataPack.Crc= (crc8(test,434, 0)<<8);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{	 

	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		ret_code_t err_code;  
	
		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
		APP_ERROR_CHECK(err_code);
		/*RG12²âÁ¿µç³ØµçÁ¿Öµ*/

				SetSystemFlag(Read_Bat_FLAG);
			//adcÊä³öÎªÓÐ·ûºÅÊý½øÐÐÊýÖµÏÞÎ»//							
				if(p_event->data.done.p_buffer[0]<0)
					p_event->data.done.p_buffer[0] = 0;
					bat_percent_buf[0] =bat_va_per(p_event->data.done.p_buffer[0]);
//					DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
				
//				bat_adc_buf[0] =	(uint8_t)(p_event->data.done.p_buffer[2]>>8)&0x0f;
//				bat_adc_buf[1] = 	(uint8_t)(p_event->data.done.p_buffer[2]&0xff);
			
		}
}

/**@brief Function for the volmeas_timeout_handler.
 *
 * @details Initializes volmeas_timeout_handler.
 */
static void volmeas_timeout_handler(void * p_context)
{
	  UNUSED_PARAMETER(p_context);
//    APP_ERROR_CHECK(nrf_drv_saadc_sample());//adc²ÉÑù
}


/**@brief Function for the volmeas_timeout_handler.
 *
 * @details Initializes volmeas_timeout_handler.
 */
/*2ÔÂ13ÈÕÐÞ¸Ä ÌåÎÂÓëµçÁ¿µÄ·¢ËÍÖµ¸³Öµ·ÅÔÚcrcÇ°±£Ö¤¼ÆËãÍêCRCºó²»ÐÞ¸Ä·¢ËÍÊý¾Ý´Ó¶ø±ÜÃâCRCÑéÖ¤Ê§°Ü**/
static void Packing_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	
//	uint8_t buff_cont;
//  if((GetSystemFlag(Send_1ofPack_FLAG)) == 0)
//	{
//				if(GetSystemFlag(Send_2ofPack_FLAG))
//				{
//						if(GetSystemFlag(ADCBuf_HT_Flag))
//						{
//					//		NRF_LOG_INFO("°ëÂú´ò°ü\n\r\r");
//						
//							
//							/*12ÔÂ3ÈÕÐÞ¸Ä ½«Õâ¸ö¸Ä³É¶ÔÓ¦°æ±¾ÏÂµÄ¹Ì¶¨µÄ»ù×¼µçÑ¹Öµ*/
////							DataPack.RefData.Value = ref_adc_buf[1]*256+ref_adc_buf[0]; 
////							DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
//							if(GetSystemFlag(Connect_Err_FLAG))
//							{	
//									DataPack.ErrData.Value |=0x0200;		
//									/*ÓÚ12ÔÂ4ÈÕÐÞ¸ÄÌí¼ÓÍÑÂäºóÊä³ö¹Ì¶¨Öµ*/		
//									for(buff_cont = 0;buff_cont< EcgDataNum;buff_cont++)
//									{
//											adc_value_Big_buf.L1Value[buff_cont] = 0x07;
//											buff_cont++;
//											adc_value_Big_buf.L1Value[buff_cont] = 0x00;
//									}
//							}
//												
//							else
//									DataPack.ErrData.Value &= ~0x0200;
//									DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);
////									ADS_DATA_AVG();
//									DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
//									DataPack.TemData.Value = temp_va; 
//									DataPack.Crc= (crc8((unsigned char *)&DataPack.IdentPrtc,912, 0)<<8)&0xFF00;									
//									send_flag = 0;
//									memcpy(&DataPack1,&DataPack,sizeof(DataPack));
//									SetSystemFlag(SendSonPackFlag);
//									SetSystemFlag(Send_3ofPack_FLAG);
//									ClrSystemFlag (Send_2ofPack_FLAG);
//									ClrSystemFlag (ADCBuf_HT_Flag);
//						}
//				}	
//				else if(GetSystemFlag(Send_3ofPack_FLAG))
//				{	
//						if(GetSystemFlag(ADCBuf_TC_Flag))
//						{
//								//   	NRF_LOG_INFO("È«Âú´ò°ü\n\r\r");
//								
//								/*12ÔÂ3ÈÕÐÞ¸Ä ½«Õâ¸ö¸Ä³É¶ÔÓ¦°æ±¾ÏÂµÄ¹Ì¶¨µÄ»ù×¼µçÑ¹Öµ*/
////								DataPack.RefData.Value = ref_adc_buf[3]*256+ref_adc_buf[2];
////								DataPack.BatData.Value = (bat_percent_buf[1]*256)&0xff00;
//								
//								if(GetSystemFlag(Connect_Err_FLAG))
//							{	
//									DataPack.ErrData.Value |=0x0200;		
//									/*ÓÚ12ÔÂ4ÈÕÐÞ¸ÄÌí¼ÓÍÑÂäºóÊä³ö¹Ì¶¨Öµ*/		
//									for(buff_cont = 0;buff_cont< EcgDataNum;buff_cont++)
//									{
//											adc_value_Big_buf.L1Value[buff_cont+EcgDataNum] = 0x07;
//											buff_cont++;
//											adc_value_Big_buf.L1Value[buff_cont+EcgDataNum] = 0x00;
//									}
//							}
//												
//							else
//									DataPack.ErrData.Value &= ~0x0200;
//									DataPacking(&adc_value_Big_buf.L1Value[EcgDataNum],&adc_value_Res_buf.L1Value[ResDataNum],&imu_value_Big_buf.Value[MotDataNum]);
//									DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
//									DataPack.TemData.Value = temp_va; 
//									DataPack.Crc= (crc8((unsigned char *)&DataPack.IdentPrtc,912, 0)<<8)&0xFF00;
//									send_flag = 1;
//									memcpy(&DataPack2,&DataPack,sizeof(DataPack));
//									SetSystemFlag(SendSonPackFlag);
//									SetSystemFlag(Send_2ofPack_FLAG);
//									ClrSystemFlag (Send_3ofPack_FLAG);
//									ClrSystemFlag (ADCBuf_TC_Flag);
//						}
//						

//				}
//	}
	//·¢ËÍÊý¾Ý°ü
 
}


/**@brief Function for the SonPackSend_timeout_handler.
 *
 * @details Initializes SonPackSend_timeout_handler.
 */
static void SonPackSend_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	uint16_t err_code = NRF_ERROR_NULL;
	uint8_t  send_contn ;
 
//		NRF_LOG_INFO("·Ö°ü·¢ËÍ\n\r\r");
////×îºóÒ»Ö¡Êý¾ÝÖ»ÓÐ16×Ö½Ú//
//		if(SonPackCon<20)
//				SetSystemFlag(Send_1ofPack_FLAG);
				/*ÓÚ11ÔÂ28ÈÕÐÞ¸Ä£¬È¥³ýÌî³äÊý¾Ý×îºóÒ»°ü·¢ËÍ2×Ö½Ú*/
		if(0 == send_flag)
		{
				if(GetSystemFlag(Send_1ofPack_FLAG))
				{
						for(send_contn = 0; send_contn<4;send_contn++)
						{
								if(SonPackCon == PACKET_NUM/20)
								{
										err_code = ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack1.CodeVers+SonPackCon*20,PACKET_NUM%(PACKET_NUM/20));
								}
								else
								{
										err_code = ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack1.CodeVers+SonPackCon*20,20);
								}

								/*ÐÞ¸ÄÓÚ11-13£¬Ìí¼ÓÀ¶ÑÀ·¢ËÍ³É¹¦µÄÅÐ¶Ï£¬±£Ö¤ÔÚ·¢ËÍ¹ý³ÌÖÐ²»»á³öÏÖ¶ª°üµÄÇé¿ö*/
								if(err_code == NRF_SUCCESS)
								{
										SonPackCon++;
								}
								if(SonPackCon > (PACKET_NUM/20))
								{
					
									ClrSystemFlag(SendSonPackFlag);
									ClrSystemFlag(Send_1ofPack_FLAG);
									ClrSystemFlag(Read_Bat_FLAG);
									ClrSystemFlag(Read_Temp_FLAG);
									SonPackCon = 0;
									send_flag = 1;
									send_contn =10;
								}
						}	
						
				}
		}
		else if(1 == send_flag)
		{
				if(GetSystemFlag(Send_2ofPack_FLAG))
				{
						for(send_contn = 0; send_contn<4;send_contn++)
						{
								if(SonPackCon == PACKET_NUM/20)
								{
										err_code = ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack2.CodeVers+SonPackCon*20,PACKET_NUM%(PACKET_NUM/20));
								}
								else
								{
										err_code = ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack2.CodeVers+SonPackCon*20,20);
								}

								/*ÐÞ¸ÄÓÚ11-13£¬Ìí¼ÓÀ¶ÑÀ·¢ËÍ³É¹¦µÄÅÐ¶Ï£¬±£Ö¤ÔÚ·¢ËÍ¹ý³ÌÖÐ²»»á³öÏÖ¶ª°üµÄÇé¿ö*/
								if(err_code == NRF_SUCCESS)
								{
										SonPackCon++;
								}
			//		else
			//				ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack.CodeVers+SonPackCon*20,16);
			//		NRF_LOG_INFO("SonPackCon=%d\n\r\r",SonPackCon); 
								if(SonPackCon > (PACKET_NUM/20))
								{
						//			NRF_LOG_INFO("ClrSystemFlag(SendSonPackFlag)\n\r\r");
									ClrSystemFlag(SendSonPackFlag);
									ClrSystemFlag(Send_2ofPack_FLAG);
									ClrSystemFlag(Read_Bat_FLAG);
									ClrSystemFlag(Read_Temp_FLAG);
									SonPackCon = 0;
									send_flag = 2;
									send_contn =10;
						//			nrf_gpio_pin_toggle(LED_1);
									
								}
						}
				}
		}
		else
		{
				if(GetSystemFlag(Send_3ofPack_FLAG))
				{
						for(send_contn = 0; send_contn<4;send_contn++)
						{
								if(SonPackCon == PACKET_NUM/20)
								{
										err_code = ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack3.CodeVers+SonPackCon*20,PACKET_NUM%(PACKET_NUM/20));
								}
								else
								{
										err_code = ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack3.CodeVers+SonPackCon*20,20);
								}

								/*ÐÞ¸ÄÓÚ11-13£¬Ìí¼ÓÀ¶ÑÀ·¢ËÍ³É¹¦µÄÅÐ¶Ï£¬±£Ö¤ÔÚ·¢ËÍ¹ý³ÌÖÐ²»»á³öÏÖ¶ª°üµÄÇé¿ö*/
								if(err_code == NRF_SUCCESS)
								{
										SonPackCon++;
								}
			//		else
			//				ble_send_vol_data(&m_volmeass, (uint8_t*)&DataPack.CodeVers+SonPackCon*20,16);
			//		NRF_LOG_INFO("SonPackCon=%d\n\r\r",SonPackCon); 
								if(SonPackCon > (PACKET_NUM/20))
								{
						//			NRF_LOG_INFO("ClrSystemFlag(SendSonPackFlag)\n\r\r");
									ClrSystemFlag(SendSonPackFlag);
									ClrSystemFlag(Send_3ofPack_FLAG);
									ClrSystemFlag(Read_Bat_FLAG);
									ClrSystemFlag(Read_Temp_FLAG);
									SonPackCon = 0;
									send_flag = 0;
									send_contn =10;
						//			nrf_gpio_pin_toggle(LED_1);
									
								}
						}
				}
		}
	
    //·¢ËÍÊý¾Ý°ü
}

/**@brief Function for the PWR_ON_OFF_timeout_handler.
 *
 * @details Initializes volmeas_timeout_handler.
 */
static void PWR_ON_OFF_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	if(nrf_gpio_pin_read(PWR_STA))
	{
		 PWR_Cont--;
	}
	else
	{
		 PWR_Cont = PWR_SAMP_CONT;
	}
	
	if(0 == PWR_Cont)
	{
		  if(Open_flag)
			{
//					ClrSystemFlag(OpenDevFlag);
//					Led_Display(BLE_LED_IDLE);
//					nrf_gpio_pin_clear(PWR_CTL);
			}		
			else
			{
					Open_flag = 1;
					nrf_gpio_pin_set(PWR_CTL);
					PWR_Cont = 200;
			}
					
	}	
	
 
}

/**@brief Function for the PWR_ON_OFF_timeout_handler.
 *
 * @details Initializes volmeas_timeout_handler.
 */
static void BLE_LED_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	Led_Display(m_stable_led_state);
	
 
}
/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}

/**@brief Function for the application_timers_start.
 *
 * @details application_timers_start.
 */
void application_timers_start(void)
{
    uint32_t err_code;
    // Start application timers.
    err_code = app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Packing_timers_start.
 *
 * @details SonPackSend_timers_start.
 */
void Packing_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_Packing_timer_id, PACKING_TIM_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
}
/**@brief Function for the SonPackSend_timers_start.
 *
 * @details SonPackSend_timers_start.
 */
void SonPackSend_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
		SonPackCon = 0;							//·¢°ü¿ªÊ¼Ê±´ÓÍ·¿ªÊ¼
    err_code = app_timer_start(m_SonPackSend_timer_id, SonPack_Send_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
}

/**@brief Function for the SonPackSend_timers_start.
 *
 * @details PWR_ON_OFF_timers_start.
 */
void PWR_ON_OFF_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_PWR_ON_OFF_timer_id, PWR_ON_OFF_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
}

///**@brief Function for the SonPackSend_timers_start.
// *
// * @details PWR_ON_OFF_timers_start.
// */
//void BLE_LED_timers_start(void)
//{
//    uint32_t err_code;

//    // Start application timers.
//    err_code = app_timer_start(m_BLE_LED_timer_id, BLE_LED_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
//	
//}
/**@brief Function for the application_timers_start.
 *
 * @details application_timers_start.
 */
void application_timers_stop(void)
{
    uint32_t err_code;

    // Stop application timers.
    err_code =  app_timer_stop(m_volmeas_timer_id);
    APP_ERROR_CHECK(err_code);
	
}
/**@brief Function for the application_timers_start.
 *
 * @details Packing_timers_start.
 */
void Packing_timers_stop(void)
{
    uint32_t err_code;

    // Stop application timers.
    err_code =  app_timer_stop(m_Packing_timer_id);
    APP_ERROR_CHECK(err_code);
	
}

/**@brief Function for the SonPackSend_timers_start.
 *
 * @details SonPackSend_timers_start.
 */
void SonPackSend_timers_stop(void)
{
    uint32_t err_code;

    // Stop application timers.
    err_code =  app_timer_stop(m_SonPackSend_timer_id);
    APP_ERROR_CHECK(err_code);
}



void saadc_init(void)
{
    ret_code_t err_code;
    
	
		//ÅäÖÃADC×ª»»ÉèÖÃ£º12BITÄ£Ê½//
		nrf_drv_saadc_config_t  saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
		saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);
	
		/*ÅäÖÃÍ¨µÀ2(bat)*/
		nrf_saadc_channel_config_t bat_channel_config =
		
		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
		bat_channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
    err_code = nrf_drv_saadc_channel_init(0, &bat_channel_config);
    APP_ERROR_CHECK(err_code);
		
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
	
	  adc_value.cnt = 0;
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;
	
	  // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
 

	  // Create timers.
	  err_code = app_timer_create(&m_SonPackSend_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                SonPackSend_timeout_handler);
		  // Create timers.
	  err_code = app_timer_create(&m_Packing_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                Packing_timeout_handler);
		// ¶¨Ê±¼ì²â°´¼ü
		err_code = app_timer_create(&m_PWR_ON_OFF_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                PWR_ON_OFF_timeout_handler);
		//À¶ÑÀÖ¸Ê¾µÆÏÔÊ¾														
		err_code = app_timer_create(&m_BLE_LED_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                BLE_LED_timeout_handler);

	
	  APP_ERROR_CHECK(err_code);
}


void send_test()
{
	     uint16_t Ecg_date = 0,Temp_date = 0;
		 uint32_t Res_date = 0;
		 uint8_t  Temp_h,Temp_l;
		 uint8_t buff_cont;					
		

		if((adc_value_Big_buf.Byte_cnt == EcgDataNum))	
		{									
			
			//DataPack.SYSTPData.Value =(unsigned short)(spi1rxbuff[4] << 8);
			//DataPack.DIATPData.Value =(unsigned short)(spi1rxbuff[5] << 8);
			//DataPack.HeartRateData.Value =(unsigned short)(spi1rxbuff[6] << 8);
			//DataPack.RWarePosData.Value = (spi1rxbuff[7]) | (spi1rxbuff[8] << 8) | (spi1rxbuff[9] << 16) | (spi1rxbuff[10] << 24);

			adc_value_Big_buf.Byte_cnt = 0;
			adc_value_Res_buf.Byte_cnt = 0;
			DataPack.Packetserialnum = swapInt32(DataPack.Packetserialnum);
			DataPack.Packetserialnum++;	//ÐòÁÐºÅ
			DataPack.Packetserialnum = swapInt32(DataPack.Packetserialnum);

			DataPack.RWarePosData.Value = swapInt32(DataPack.RWarePosData.Value);
			DataPack.RWarePosData.Value++;
			DataPack.RWarePosData.Value = swapInt32(DataPack.RWarePosData.Value);
									
			if(0 == send_choose)
			{
				DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);
				DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
				DataPack.TemData.Value = temp_va; 
																				
				memcpy(&DataPack1,&DataPack,sizeof(DataPack));
				DataPack1.Crc= (crc8((unsigned char *)&DataPack1.IdentPrtc,886, 0)<<8)&0xFF00;
				send_choose =1;
				SetSystemFlag(Send_1ofPack_FLAG);
											
									
			}	
			else if(1 == send_choose)
			{
				DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);
				DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
				DataPack.TemData.Value = temp_va; 
																
				memcpy(&DataPack2,&DataPack,sizeof(DataPack));
				DataPack2.Crc= (crc8((unsigned char *)&DataPack2.IdentPrtc,886, 0)<<8)&0xFF00;				
				send_choose =2;
				SetSystemFlag(Send_2ofPack_FLAG);
											
			}	
			else
			{
				DataPacking(adc_value_Big_buf.L1Value,adc_value_Res_buf.L1Value,&imu_value_Big_buf.Value[0]);

				DataPack.BatData.Value = (bat_percent_buf[0]*256)&0xff00;
				DataPack.TemData.Value = temp_va; 
																					
				memcpy(&DataPack3,&DataPack,sizeof(DataPack));
				DataPack3.Crc= (crc8((unsigned char *)&DataPack3.IdentPrtc,886, 0)<<8)&0xFF00;
				send_choose =0;
				SetSystemFlag(Send_3ofPack_FLAG);
			 }					
		}
}


void spi1_event_handler(nrf_drv_spi_evt_t const * p_event)
{
	unsigned char i = 0;
    spi1_xfer_done = true;
	packetcnt++;
}



void timer_spi_event_handler(void)
{
    unsigned char i = 0;
	rxflag = 1;
	#if 0
	rxflag = 0;
	stm32_spi_once(spi1txbuff,spi1rxbuff,SPI_BUF_SIXE);
	//stm32_spi_conn(spi1txbuff,spi1rxbuff,SPI_BUF_SIXE);
			if(spi1rxbuff[0] == 0xaa)
			{
				
				for(i = 0;i<(24);i++)
				{
					DataPack.PPGData.Value[adc_value_Big_buf.Byte_cnt]	 = spi1rxbuff[(i*4)+1];
					DataPack.PPGData.Value[adc_value_Big_buf.Byte_cnt +1]	 = spi1rxbuff[(i*4) +1+1];
					adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt] = spi1rxbuff[(i*4) +2+1]; 
					adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt+1] = spi1rxbuff[(i*4) + 3+1];
					adc_value_Big_buf.Byte_cnt+=2;
				}
				send_test();
			}
	#endif
}

void spi_timer_init(void)
{
	ret_code_t err_code;
	uint32_t time_ticks;
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	
    err_code = nrf_drv_timer_init(&TIMER_SPI, &timer_cfg, timer_spi_event_handler);
	time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_SPI,100);

	nrf_drv_timer_extended_compare(&TIMER_SPI, NRF_TIMER_CC_CHANNEL4, time_ticks, NRF_TIMER_SHORT_COMPARE4_CLEAR_MASK, true);
}

void spi_sample_loop(void)
{
	unsigned char i = 0;
		if((rxflag == 1) && (gCmdflag == 1))
		{
			rxflag = 0;
			stm32_spi_conn(spi1txbuff,spi1rxbuff,SPI_BUF_SIXE);
			
			if(spi1rxbuff[0] == 0xaa)
			{
				for(i = 0;i<(SPI_BUF_SIXE-1)/PACKET_LEN;i++)
				{
					DataPack.PPGData.Value[adc_value_Big_buf.Byte_cnt]	 = spi1rxbuff[(i*PACKET_LEN)+0+1];
					DataPack.PPGData.Value[adc_value_Big_buf.Byte_cnt +1]	 = spi1rxbuff[(i*PACKET_LEN) +1+1];
					adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt] = spi1rxbuff[(i*PACKET_LEN) +2+1]; 
					adc_value_Big_buf.L1Value[adc_value_Big_buf.Byte_cnt+1] = spi1rxbuff[(i*PACKET_LEN) + 3+1];
					DataPack.SYSTPData.Value = spi1rxbuff[(i*PACKET_LEN) + 4+1];
					DataPack.DIATPData.Value = spi1rxbuff[(i*PACKET_LEN) + 5+1];
					DataPack.HeartRateData.Value = spi1rxbuff[(i*PACKET_LEN) + 6+1];
					DataPack.RWarePosData.Value = (spi1rxbuff[(i*PACKET_LEN) + 7+1]) | (spi1rxbuff[(i*PACKET_LEN)+8+1] << 8) | (spi1rxbuff[(i*PACKET_LEN)+9+1] << 16) | (spi1rxbuff[(i*PACKET_LEN)+10+1] << 24);
					adc_value_Big_buf.Byte_cnt+=2;
					
				}
				send_test();
			}
		}
		if(gCmdflag == 0)
		{
			adc_value_Big_buf.Byte_cnt = 0;
		}
}



void spi_send_userdata(void)
{
	while((guserdataflag == 0x02))
	{
		if((userrxbuff[11] == 0x55) && (userrxbuff[12] == 0xaa))
		{
			//ç­‰å¾…è¿”å›žç¡®è®¤æ•°æ®
			guserconfig = 0x00;
			guserdataflag = 0x00;
		}
		else
		{
			//å‘é€ç”¨æˆ·é…ç½®æ•°æ®
			stm32_spi_conn(usertxbuff,userrxbuff,13);
			nrf_delay_ms(10);
		}
	}
}

int main(void)
{
		ret_code_t err_code;
		uint8_t id;
		volatile	uint8_t closedev_flag = 1; 
		uint8_t  MPU6050_INIT_FAIL_CONT = 0;
		
		/*RG10ÒªÇóecgÊý¾Ý²ÉÑùÂÊÎª200HZ,¹ÊÎª2ms²É¼¯Ò»´ÎÊý¾Ý*/
		uint32_t time_ticks=53328;//¶¨Ê±Ê±¼ä3.3ms tick=53328 TIME3=16MHZ,¹Ê1ms tick=16000 
		//uint32_t time_ticks=16000;
		// CLEAR ALL FLAG
		ClrAllSystemFlag();
		ClrSystemFlag(Send_1ofPack_FLAG);
		ClrSystemFlag(Send_2ofPack_FLAG);
		ClrSystemFlag (Send_3ofPack_FLAG);
	  /*ÉèÖÃLEDÖ¸Ê¾µÆ*/
		nrf_gpio_cfg_output(LED_RUN);
		nrf_gpio_cfg_output(LED_BLE_STATE);
		Led_Display(BLE_LED_IDLE);
		/*³õÊ¼»¯¼ì²âIOºÍ¿ØÖÆIO*/
		nrf_gpio_cfg_output(PWR_CTL);
		nrf_gpio_pin_clear(PWR_CTL);
		nrf_gpio_cfg_input(PWR_STA,NRF_GPIO_PIN_PULLDOWN);
		/*app_time³õÊ¼»¯*/
		timers_init();
		/*ADS1292R Ä£¿é³õÊ¼»¯*/
		//ADS_SPI_INIT();

		flash_dev_spi_init();
		
		while(W25Q80_ReadID() != SPIFLASH_ID)
		{
			nrf_delay_ms(500);
		}
		spi_flash_littlefs_test();
		PWR_ON_OFF_timers_start();
		

		err_code = NRF_LOG_INIT(NULL);
		APP_ERROR_CHECK(err_code);
		//saadc_init();	  
		//twi_master_init();
		//twi_mlx90615_init();
		stm32_spi_init();
		
//		nrf_delay_ms(2);
		CM18_BLE_SERVICES_init();//³õÊ¼»¯CM18 BLEÏà¹Ø·þÎñ 
		/*°´ÕÕÉè¶¨¶ÔADS1292R¼Ä´æÆ÷½øÐÐÉèÖÃ*/
		//ADS_Default_Register_Settings();
		//Stop_Read_Data_Continuous ();
		/*ÅÐ¶ÏADSÊÇ·ñ×¼±¸Íê±Ï*/
		//	ADS_ID_PAIR();

		nrf_delay_ms(2);
		nrf_gpio_pin_clear(SPI_SS_PIN);	
		nrf_delay_ms(100);
	 /*	while((mpu6050_init(0x68) == false)&&(0 == MPU6050_INIT_ERR))
		{
				NRF_LOG_INFO("mpu6050 init fail\r\n");
				MPU6050_INIT_FAIL_CONT++;
				if(MPU6050_INIT_FAIL_CONT>9)
				{
						MPU6050_INIT_ERR = 1;
				}
				
				nrf_delay_ms(500);
		}
	*/
		Led_Display(m_stable_led_state);
 
	  InitDataPack(&DataPack);//³õÊ¼»¯TLV°ü·Ç±äÁ¿
//	  
	  	  //¶¨Òå¶¨Ê±Æ÷ÅäÖÃ½á¹¹Ìå£¬²¢Ê¹ÓÃÄ¬ÈÏÅäÖÃ²ÎÊý³õÊ¼»¯½á¹¹Ìå
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	  //³õÊ¼»¯¶¨Ê±Æ÷£¬³õÊ¼»¯Ê±»á×¢²áTIMER_ADC_event_handlerÊÂ¼þ»Øµ÷º¯Êý
    err_code = nrf_drv_timer_init(&TIMER_ADC, &timer_cfg, timer_adc_event_handler);
    APP_ERROR_CHECK(err_code);
    
	  //¶¨Ê±Ê±¼ä(µ¥Î»ms)×ª»»Îªticks

    //ÉèÖÃ¶¨Ê±Æ÷²¶»ñ/±È½ÏÍ¨µÀ¼°¸ÃÍ¨µÀµÄ±È½ÏÖµ£¬Ê¹ÄÜÍ¨µÀµÄ±È½ÏÖÐ¶Ï
    nrf_drv_timer_extended_compare(
         &TIMER_ADC, NRF_TIMER_CC_CHANNEL3, time_ticks, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK, true);
				 /*ADS1292test function*/
		nrf_delay_ms(2);
			nrf_gpio_pin_clear(SPI_SS_PIN);	
		nrf_delay_ms(100);	

		spi_timer_init();

		
		
    // Enter main loop.
    for (;;)
    {
    	if((guserconfig == 0xff))
    	{
			spi_send_userdata();
		}
		else
		{
			spi_sample_loop();
		}
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
		
		//stm32_spi_conn(spi1txbuff,spi1rxbuff,15);
		/*
		stm32_spi_conn(spi1txbuff,spi1rxbuff,15);
		gadc_data_buff_ecg[gadc_data_buff_len++] = spi1rxbuff[0];
		gadc_data_buff_ecg[gadc_data_buff_len++] = spi1rxbuff[1];
		if(gadc_data_buff_len == 20)
		{
			ble_send_vol_data(&m_volmeass, (uint8_t*)gadc_data_buff_ecg,20);
			gadc_data_buff_len = 0;
			memset(gadc_data_buff_ecg,0,20);
		}
		*/
		
//					nrf_gpio_pin_clear(LED_RUN);
//					nrf_gpio_pin_clear(LED_BLE_STATE);
//					MPU6050_ReadAcc(&imu_value_Big_buf.Value[IMU_TICK].accx,//¼ÓËÙ¶È3Öá²É¼¯
//															&imu_value_Big_buf.Value[IMU_TICK].accy,
//															&imu_value_Big_buf.Value[IMU_TICK].accz);
//					MPU6050_ReadGyr(&imu_value_Big_buf.Value[IMU_TICK].gyrx,//ÍÓÂÝÒÇ3Öá²É¼¯
//															&imu_value_Big_buf.Value[IMU_TICK].gyry,
//															&imu_value_Big_buf.Value[IMU_TICK].gyrz);
//					nrf_gpio_pin_set(LED_RUN);
////					if((!(GetSystemFlag(Read_Temp_FLAG))))
////						{
//									mlx90615_register_read(0x27, tempbuff,2);
//				nrf_gpio_pin_set(LED_BLE_STATE);
////								if(tempbuff[1] != 0xff)
////								{
////										DataPack.TemData.Value = (tempbuff[0]*256+tempbuff[1])&0xffff;
////										SetSystemFlag(Read_Temp_FLAG);
////								}
////						}
//			
//				Read_Data_Once();
//					
//					m_stable_led_state = 	BLE_LED_IDLE;
//				ble_send_vol_data(&m_volmeass, tempbuff,2);
//				nrf_gpio_pin_clear(LED_RUN);
//				nrf_delay_ms(20);
				
    }
}

