#include "Flash_driver.h"

#include "nrf_drv_spi.h"
#include "BLE_SET.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nordic_common.h"
#include "nrf.h"


static const nrf_drv_spi_t flash_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static volatile bool flash_spi_xfer_done; 

uint8_t gspi_tx_buf[8];
uint8_t gspi_rx_buf[8];



void flash_dev_spi_init(void)
{
		nrf_gpio_cfg_input(FLASH_MISO_PIN,NRF_GPIO_PIN_NOPULL);	

		nrf_gpio_cfg_output(FLASH_SCK_PIN);																//ѡ��λ�ӿ�
		nrf_gpio_cfg_output(FLASH_MOSI_PIN);																//ת��ʹ�ܶ�
		nrf_gpio_cfg_output(FLASH_CS_PIN);															//����Ƭѡ�ӿ�
	
		nrf_gpio_pin_set(FLASH_CS_PIN);	
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    	spi_config.miso_pin = FLASH_MISO_PIN;
    	spi_config.mosi_pin = FLASH_MOSI_PIN;
    	spi_config.sck_pin  = FLASH_SCK_PIN;
		//spi_config.ss_pin	= FLASH_CS_PIN;
		spi_config.mode =	NRF_DRV_SPI_MODE_0;
		spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    	APP_ERROR_CHECK(nrf_drv_spi_init(&flash_spi, &spi_config, flash_dev_event_handler));
}

void flash_dev_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    flash_spi_xfer_done = true;
}

/*****************************************************************************
** 文件名称：static uint8_t spi_flash_read_status_reg(void)
** 功    能：“读状态寄存器”命令，retValue为读到的数值
** 修改日志：
** 附    录：当CS拉低之后，把05H从DI引脚输入到Flash芯片，CLK上升沿，数据写入Flash，当Flash收到05H后，会把“状态寄存器”的值，从D0引脚输出，CLK下降沿输出。  
******************************************************************************/



static uint8_t spi_flash_read_one_byte(void) 
{
    	uint8_t len = 1;

        gspi_tx_buf[0] = 0xFF;
        flash_spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&flash_spi, gspi_tx_buf, len, gspi_rx_buf, len));
        while(!flash_spi_xfer_done);
    return (gspi_rx_buf[0]);
}

static void spi_flash_write_one_byte(uint8_t Dat)
{
    	uint8_t len = 1;
        ret_code_t ret_code;

        gspi_tx_buf[0] = Dat;
        flash_spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&flash_spi, gspi_tx_buf, len, gspi_rx_buf, len));
        while(!flash_spi_xfer_done);
}

static uint8_t spi_flash_read_status_reg(void)
{
    uint8_t retValue = 0;

    CS_L;
    spi_flash_write_one_byte(SPIFLASH_READSR_CMD);
    retValue = spi_flash_read_one_byte();
    CS_H;
    return retValue;
}

/*****************************************************************************
** 文件名称：static uint8_t spi_flash_wait_busy(void)
** 功    能：判断Flash是否busy。
** 修改日志：
：SPIFLASH_WRITE_BUSYBIT 写状态寄存器 
******************************************************************************/
static uint8_t spi_flash_wait_busy(void)
{
    uint8_t spi_wait_count = 0;
    while((spi_flash_read_status_reg() & SPIFLASH_WRITE_BUSYBIT) == 0x01)  //状态寄存器0位为Busy位
    {  
       if(spi_wait_count >= 100)
          break;
    }

        //NRF_LOG_INFO("spi_wait_count = %d", spi_wait_count);
    return 1;
}


void spi_flash_write_enable(void)
{
        CS_L;  
        spi_flash_write_one_byte(SPIFLASH_WRITEEN_CMD);
        CS_H;  
}

/*****************************************************************************
** 文件名称：uint8_t spi_flash_read_data(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum)
** 功    能：读Flash的某地址ReadAddr，读多大ReadByteNum，的数值。
** 修改日志：
******************************************************************************/
uint8_t spi_flash_read_data(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t ReadBytesNum)
{
    uint8_t len;

        gspi_tx_buf[0] = SPIFLASH_READDATA_CMD;
        gspi_tx_buf[1] = (uint8_t)((ReadAddr & 0x00ff0000) >> 16);
        gspi_tx_buf[2] = (uint8_t)((ReadAddr & 0x0000ff00) >> 8);
        gspi_tx_buf[3] = (uint8_t)ReadAddr;

        len = ReadBytesNum + 4;

        CS_L;
        flash_spi_xfer_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&flash_spi, gspi_tx_buf, len, gspi_rx_buf, len));
        while(!flash_spi_xfer_done);
        CS_H;

        memcpy(pBuffer, &gspi_rx_buf[4], ReadBytesNum);

        return 1;
}

// 52832 spi flash 的库每次最多只能写或者读取 251个字节，常规页是256字节，所以要特殊处理一下
uint8_t spi_flash_read_data_52832(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum)
{


   		 uint32_t len = ReadBytesNum, len_cut = 0;
         uint32_t addr = ReadAddr;
         uint8_t *pic_point = pBuffer;
		 uint32_t i;
		  CS_L;
		  spi_flash_write_one_byte(SPIFLASH_READDATA_CMD);
		  //spi_flash_write_one_byte((addr & 0xff000000) >> 24);
		  spi_flash_write_one_byte((addr & 0x00ff0000) >> 16);
		  spi_flash_write_one_byte((addr & 0x0000ff00) >> 8);
		  spi_flash_write_one_byte(addr & 0xff);
		// gspi_tx_buf[0] = SPIFLASH_READDATA_CMD;
		// gspi_tx_buf[1] = (uint8_t)((addr & 0xff000000) >> 24);
        // gspi_tx_buf[2] = (uint8_t)((addr & 0x00ff0000) >> 16);
        // gspi_tx_buf[3] = (uint8_t)((addr & 0x0000ff00) >> 8);
        // gspi_tx_buf[4] = (uint8_t)addr;
		//  flash_spi_xfer_done = false;
		// APP_ERROR_CHECK(nrf_drv_spi_transfer(&flash_spi, gspi_tx_buf, 5, gspi_rx_buf, 5));
		  //while(!flash_spi_xfer_done);
        	for(i=0;i<ReadBytesNum;i++)
        	{
				*pBuffer++ = spi_flash_read_one_byte();
			}
                //len_cut = (len >= (0xFF - 4)) ? (0xFF) : (len + 4);
                //flash_spi_xfer_done = false;
                //APP_ERROR_CHECK(nrf_drv_spi_transfer(&flash_spi, gspi_tx_buf, 4, gspi_rx_buf, len_cut));
                
               // *pBuffer++ = spi_flash_read_one_byte();
               // while(!flash_spi_xfer_done);
               // len--;

                //memcpy(pic_point, &gspi_rx_buf[4], (len_cut - 4));
                //addr += (len_cut - 4);
                //len -= (len_cut - 4);
                //pic_point += (len_cut - 4);
        
		 CS_H;
    return 1;
    
}

uint8_t spi_flash_erase_addr(uint32_t SectorAddr)
{
   SectorAddr *= 4096;
   spi_flash_write_enable();  //发送FLASH写使能命令
   spi_flash_wait_busy();  //等待写完成
   CS_L;	//片选有效
   spi_flash_write_one_byte(SPIFLASH_SECTORERASE);	//发送扇区擦除指令
   //spi_flash_write_one_byte((SectorAddr & 0xff000000) >> 24);
   spi_flash_write_one_byte((SectorAddr & 0XFF0000) >> 16); //发送扇区擦除地址的高位
   spi_flash_write_one_byte((SectorAddr & 0XFF00) >> 8);
   spi_flash_write_one_byte(SectorAddr & 0XFF);
   CS_H;  //片选无效
   spi_flash_wait_busy();  //等待擦除完成
}

uint8_t spi_flash_erase_chip()
{
	spi_flash_write_enable();  //发送FLASH写使能命令
	 spi_flash_wait_busy();  //等待写完成
    CS_L;  //片选有效
    spi_flash_write_one_byte(SPIFLASH_CHIPERASE);  //全片擦除
    CS_H;  //片选无效
    spi_flash_wait_busy();  //等待写完成
}


uint8_t spi_flash_write_page(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
	
   spi_flash_write_enable();  //发送FLASH写使能命令
   CS_L;	//片选有效
   spi_flash_write_one_byte(SPIFLASH_PAGEPROGRAM);	//发送写指令
   //spi_flash_write_one_byte((WriteAddr & 0xff000000) >> 24);
   spi_flash_write_one_byte((WriteAddr & 0XFF0000) >> 16); //发送写地址的高位
   spi_flash_write_one_byte((WriteAddr & 0XFF00) >> 8);
   spi_flash_write_one_byte(WriteAddr & 0XFF);
   while(NumByteToWrite--)
   {
	   spi_flash_write_one_byte(*pBuffer++);
	   //pBuffer++;
   }
   CS_H;  //片选无效
   spi_flash_wait_busy();  //等待写完成
}

//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void spi_flash_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite)   
{ 			 		 
	uint32_t pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		spi_flash_write_page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	}	    
}



uint32_t W25Q80_ReadID(void)
{
   volatile uint32_t temp = 0,temp0 = 0,temp1 = 0,temp2 = 0;
    CS_L;  //片选有效
    spi_flash_write_one_byte(SPIFLASH_JEDECID);
    temp0 = spi_flash_read_one_byte();
    temp1 = spi_flash_read_one_byte();
    temp2 = spi_flash_read_one_byte();
    CS_H;  //片选无效
    temp = (temp0 << 16)| (temp1 << 8) | temp2;
    return temp;
}


