#ifndef FALSH_DRIVER_H
#define FALSH_DRIVER_H

#include "nrf_drv_spi.h"
#include "BLE_SET.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nordic_common.h"
#include "nrf.h"

#define SPI_INSTANCE  2


#define FLASH_SCK_PIN 										14
#define FLASH_MISO_PIN 										13
#define FLASH_MOSI_PIN										15
#define FLASH_CS_PIN 										11



#define CS_L nrf_gpio_pin_clear(FLASH_CS_PIN)
#define CS_H nrf_gpio_pin_set(FLASH_CS_PIN)

#define SPIFLASH_WRITEEN_CMD 0x06
#define SPIFLASH_JEDECID 0X9F
#define SPIFLASH_READSR_CMD 0x05
#define SPIFLASH_BLOCKRERASE 0xD8
#define SPIFLASH_SECTORERASE 0x20

#define SPIFLASH_PAGEPROGRAM 0x02 
#define SPIFLASH_READDATA_CMD 0x03 
#define SPIFLASH_CHIPERASE 0xC7
#define SPIFLASH_ID 0XEF4020  //器件ID
#define SPIFLASH_DUMM_BYTE 0XFF
#define SPIFLASH_WRITE_BUSYBIT  0x01
























void flash_dev_spi_init(void);
void flash_dev_event_handler(nrf_drv_spi_evt_t const * p_event);

void spi_flash_write_enable(void);

uint8_t spi_flash_read_data(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t ReadBytesNum);

uint8_t spi_flash_read_data_52832(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum);

uint8_t spi_flash_erase_addr(uint32_t SectorAddr);

uint8_t spi_flash_write_page(unsigned char* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

void spi_flash_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);


uint32_t W25Q80_ReadID(void);


#endif
