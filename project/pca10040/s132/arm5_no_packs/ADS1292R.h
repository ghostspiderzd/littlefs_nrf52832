/*******************************************************************************
-------------------------------------------------------------------------------
* @file    ADS1292R.h
* @author  Ye Junjie
* @date    2018-12-16  
*******************************************************************************/
#ifndef ADS1292R_H
#define ADS1292R_H

#include "nrf_drv_spi.h"
#include "BLE_SET.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nordic_common.h"
#include "nrf.h"
#define ADS1292R									0x73           			
/*ADS1292R ¼Ä´æÆ÷¶ÔÓ¦µØÖ·*/

#define  ADS1292R_ID							0X00
#define  ADS1292R_CONFIG1					0X01
#define  ADS1292R_CONFIG2					0X02
#define  ADS1292R_LOFF						0x03
#define  ADS1292R_CH1SET					0x04	
#define  ADS1292R_CH2SET					0x05	
#define  ADS1292R_RLD_SENS				0x06	
#define  ADS1292R_LOFF_SENS				0x07
#define  ADS1292R_LOFF_STAT				0x08
#define  ADS1292R_RESP1						0x09
#define  ADS1292R_RESP2						0x0A
#define  ADS1292R_GPIO						0x0B

/*ADS1292R CMD DEF*/
#define  ADS1292R_WAKEUP          0x02
#define  ADS1292R_STANDBY         0x04
#define  ADS1292R_RESET           0x06
#define  ADS1292R_START          	0x08
#define  ADS1292R_STOP           	0x0A
#define  ADS1292R_OFFSETCAL       0x1A
#define  ADS1292R_RDATAC          0x10
#define  ADS1292R_SDATAC          0x11
#define  ADS1292R_RDATA           0x12
#define  ADS1292R_WREG            0x40
#define  ADS1292R_RREG            0x20

#define  ADS_CLKSEL      								19
#define  ADS_RESET      								18
#define  ADS_DRAY     								12
#define  ADS_COLCK     								22
#define  ADS_STAT									17

#define SPI_SCK_PIN 										14
#define SPI_MISO_PIN 									13
#define SPI_MOSI_PIN									15
#define SPI_SS_PIN 										11

#define SPI_INSTANCE  2 /**< SPI instance index. */

  /**< SPI instance. */
/*FUNCTION*/
void spi_event_handler(nrf_drv_spi_evt_t const * p_event);
void ADS_SPI_INIT(void);
void ADS_SPI_Command_date(uint8_t command);
void ADS_SPI_WREG_Command(uint8_t Start_Address,uint8_t Wreg_Date);
uint8_t ADS_SPI_RREG_Command(uint8_t Start_Address);
void ADS_MOD_RESET(void);
void ADS_Default_Register_Settings(void);
void Stop_Read_Data_Continuous (void);
void Start_Read_Data_Continuous (void);
void Read_Data_Once(void);
void ADS_ID_PAIR(void);
void ADS_DATA_AVG();
#endif 