#ifndef STM32SPI_H
#define STM32SPI_H

#include "stdint.h"
#include "nrf_drv_spi.h"
#include "pca10040.h"
//void spi1_event_handler(nrf_drv_spi_evt_t const * p_event);
void stm32_spi_init(void);
void stm32_spi_conn(uint8_t *m_write,uint8_t *m_read,uint8_t m_length);
void stm32_spi_once(uint8_t *m_write,uint8_t *m_read,uint8_t m_length);

#endif

