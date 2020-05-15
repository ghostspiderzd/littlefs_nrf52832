#include "STM32SPI.h"
#include "nrf_delay.h"
static const nrf_drv_spi_t spi1 = NRF_DRV_SPI_INSTANCE(0);  /**< SPI instance. */
volatile bool spi1_xfer_done;
extern void spi1_event_handler(nrf_drv_spi_evt_t const * p_event);
void stm32_spi_init()
{
		nrf_drv_spi_config_t spi1_config = NRF_DRV_SPI_DEFAULT_CONFIG;
      	spi1_config.ss_pin   = SPI1_SS_PIN;
		nrf_gpio_cfg_output(SPI1_SS_PIN);	
		nrf_gpio_pin_set(SPI1_SS_PIN);	
    	spi1_config.miso_pin = SPI1_MISO_PIN;
    	spi1_config.mosi_pin = SPI1_MOSI_PIN;
    	spi1_config.sck_pin  = SPI1_SCK_PIN;
		//spi1_config.ss_pin	 = SPI1_SS_PIN;
		spi1_config.mode =	NRF_DRV_SPI_MODE_0;
		spi1_config.frequency = NRF_DRV_SPI_FREQ_8M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi1, &spi1_config, spi1_event_handler));
	//nrf_delay_ms(200);
	
}
void stm32_spi_conn(uint8_t *m_write,uint8_t *m_read,uint8_t m_length)
{
			unsigned int relax = 0;
			spi1_xfer_done = false;
			nrf_gpio_pin_clear(SPI1_SS_PIN);	
			nrf_delay_ms(5);
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi1, m_write, m_length, m_read, m_length));
			while (!spi1_xfer_done)
	        {
	            __WFE();
	           //if(relax++ > 200)	return;
	        }
			nrf_delay_ms(5);
			nrf_gpio_pin_set(SPI1_SS_PIN);	
}

void stm32_spi_once(uint8_t *m_write,uint8_t *m_read,uint8_t m_length)
{
	nrf_gpio_pin_clear(SPI1_SS_PIN);	
	nrf_delay_ms(2);
	nrf_drv_spi_transfer(&spi1, m_write, m_length, m_read, m_length);
	nrf_delay_ms(5);
	nrf_gpio_pin_set(SPI1_SS_PIN);	
}