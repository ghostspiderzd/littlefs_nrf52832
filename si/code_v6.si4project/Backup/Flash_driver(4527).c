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

		nrf_gpio_cfg_output(FLASH_SCK_PIN);																//选择复位接口
		nrf_gpio_cfg_output(FLASH_MOSI_PIN);																//转换使能端
		nrf_gpio_cfg_output(FLASH_CS_PIN);															//配置片选接口
	
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
** 鏂囦欢鍚嶇О锛歴tatic uint8_t spi_flash_read_status_reg(void)
** 鍔�    鑳斤細鈥滆鐘舵�佸瘎瀛樺櫒鈥濆懡浠わ紝retValue涓鸿鍒扮殑鏁板��
** 淇敼鏃ュ織锛�
** 闄�    褰曪細褰揅S鎷変綆涔嬪悗锛屾妸05H浠嶥I寮曡剼杈撳叆鍒癋lash鑺墖锛孋LK涓婂崌娌匡紝鏁版嵁鍐欏叆Flash锛屽綋Flash鏀跺埌05H鍚庯紝浼氭妸鈥滅姸鎬佸瘎瀛樺櫒鈥濈殑鍊硷紝浠嶥0寮曡剼杈撳嚭锛孋LK涓嬮檷娌胯緭鍑恒��  
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
** 鏂囦欢鍚嶇О锛歴tatic uint8_t spi_flash_wait_busy(void)
** 鍔�    鑳斤細鍒ゆ柇Flash鏄惁busy銆�
** 淇敼鏃ュ織锛�
锛歋PIFLASH_WRITE_BUSYBIT 鍐欑姸鎬佸瘎瀛樺櫒 
******************************************************************************/
static uint8_t spi_flash_wait_busy(void)
{
    uint8_t spi_wait_count = 0;
    while((spi_flash_read_status_reg() & SPIFLASH_WRITE_BUSYBIT) == 0x01)  //鐘舵�佸瘎瀛樺櫒0浣嶄负Busy浣�
    {  
       if(spi_wait_count++ >= 100)
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
** 鏂囦欢鍚嶇О锛歶int8_t spi_flash_read_data(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum)
** 鍔�    鑳斤細璇籉lash鐨勬煇鍦板潃ReadAddr锛岃澶氬ぇReadByteNum锛岀殑鏁板�笺��
** 淇敼鏃ュ織锛�
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

// 52832 spi flash 鐨勫簱姣忔鏈�澶氬彧鑳藉啓鎴栬�呰鍙� 251涓瓧鑺傦紝甯歌椤垫槸256瀛楄妭锛屾墍浠ヨ鐗规畩澶勭悊涓�涓�
uint8_t spi_flash_read_data_52832(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum)
{


   		 uint32_t len = ReadBytesNum, len_cut = 0;
        uint32_t addr = ReadAddr;
        uint8_t *pic_point = pBuffer;

        do {
                gspi_tx_buf[0] = SPIFLASH_READDATA_CMD;
                gspi_tx_buf[1] = (uint8_t)((addr & 0x00ff0000) >> 16);
                gspi_tx_buf[2] = (uint8_t)((addr & 0x0000ff00) >> 8);
                gspi_tx_buf[3] = (uint8_t)addr;
                len_cut = (len >= (0xFF - 4)) ? (0xFF) : (len + 4);

                CS_L;
                flash_spi_xfer_done = false;
                APP_ERROR_CHECK(nrf_drv_spi_transfer(&flash_spi, gspi_tx_buf, 4, gspi_rx_buf, len_cut));
                while(!flash_spi_xfer_done);
                CS_H;

                memcpy(pic_point, &gspi_rx_buf[4], (len_cut - 4));
                addr += (len_cut - 4);
                len -= (len_cut - 4);
                pic_point += (len_cut - 4);
        } while(len > 0);

    return 1;
    
}

uint8_t spi_flash_erase_addr(uint32_t SectorAddr)
{
	
   spi_flash_write_enable();  //鍙戦�丗LASH鍐欎娇鑳藉懡浠�
   spi_flash_wait_busy();  //绛夊緟鍐欏畬鎴�
   CS_L;	//鐗囬�夋湁鏁�
   spi_flash_write_one_byte(SPIFLASH_SECTORERASE);	//鍙戦�佹墖鍖烘摝闄ゆ寚浠�
   spi_flash_write_one_byte((SectorAddr & 0XFF0000) >> 16); //鍙戦�佹墖鍖烘摝闄ゅ湴鍧�鐨勯珮浣�
   spi_flash_write_one_byte((SectorAddr & 0XFF00) >> 8);
   spi_flash_write_one_byte(SectorAddr & 0XFF);
   CS_H;  //鐗囬�夋棤鏁�
   spi_flash_wait_busy();  //绛夊緟鎿﹂櫎瀹屾垚
}

uint8_t spi_flash_erase_chip()
{
	spi_flash_write_enable();  //鍙戦�丗LASH鍐欎娇鑳藉懡浠�
    CS_L;  //鐗囬�夋湁鏁�
    spi_flash_write_one_byte(SPIFLASH_CHIPERASE);  //鍏ㄧ墖鎿﹂櫎
    CS_H;  //鐗囬�夋棤鏁�
    spi_flash_wait_busy();  //绛夊緟鍐欏畬鎴�
}


uint8_t spi_flash_write_page_more(unsigned char* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	
   spi_flash_write_enable();  //鍙戦�丗LASH鍐欎娇鑳藉懡浠�
   CS_L;	//鐗囬�夋湁鏁�
   spi_flash_write_one_byte(SPIFLASH_PAGEPROGRAM);	//鍙戦�佸啓鎸囦护
   spi_flash_write_one_byte((WriteAddr & 0XFF0000) >> 16); //鍙戦�佸啓鍦板潃鐨勯珮浣�
   spi_flash_write_one_byte((WriteAddr & 0XFF00) >> 8);
   spi_flash_write_one_byte(WriteAddr & 0XFF);
   if(NumByteToWrite > 256)
   {
	   NRF_LOG_INFO("write too large!\r\n");
	   return 0;
   }
   while(NumByteToWrite--)
   {
	   spi_flash_write_one_byte(*pBuffer);
	   pBuffer++;
   }
   CS_H;  //鐗囬�夋棤鏁�
   spi_flash_wait_busy();  //绛夊緟鍐欏畬鎴�
}

uint32_t W25Q80_ReadID(void)
{
   volatile uint32_t temp = 0,temp0 = 0,temp1 = 0,temp2 = 0;
    CS_L;  //鐗囬�夋湁鏁�
    spi_flash_write_one_byte(SPIFLASH_JEDECID);
    temp0 = spi_flash_read_one_byte();
    temp1 = spi_flash_read_one_byte();
    temp2 = spi_flash_read_one_byte();
    CS_H;  //鐗囬�夋棤鏁�
    temp = (temp0 << 16)| (temp1 << 8) | temp2;
    return temp;
}


