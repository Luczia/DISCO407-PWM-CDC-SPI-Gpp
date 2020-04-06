/*
 * spi_conf.h
 *
 *  Created on: Mar 28, 2020
 *      Author: Luczia
 */

#ifndef SPI_CONF_H_
#define SPI_CONF_H_

#define PORT_SPI1_SCK   GPIOA
#define PIN_SPI1_SCK    5
#define PORT_SPI1_MISO  GPIOA
#define PIN_SPI1_MISO   6
#define PORT_SPI1_MOSI  GPIOA
#define PIN_SPI1_MOSI   7

#define PORT_SPI2_CS    GPIOB
#define PIN_SPI2_CS     12
#define PORT_SPI2_SCK   GPIOB
#define PIN_SPI2_SCK    13
#define PORT_SPI2_MISO  GPIOB
#define PIN_SPI2_MISO   14
#define PORT_SPI2_MOSI  GPIOB
#define PIN_SPI2_MOSI   15



/*Low speed SPI configuration (164.125kHz, CPHA=0, CPOL=0, MSb first).
 */

static const SPIConfig ls_spicfg = {
  false,
  NULL,
  PORT_SPI2_CS,  //activate if SPI_SELECT_MODE == SPI_SELECT_MODE_PAD in chconf.h
  PIN_SPI2_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
  0
};

/*
 * Custom speed SPI configuration (1.3MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig cs_spicfg = {
  false,
  NULL,
  PORT_SPI2_CS,
  PIN_SPI2_CS,
  SPI_CR1_BR_2 , // | SPI_CR1_CPOL, //mask definition in C:\ChibiStudio\chibios_trunk\os\common\ext\ST\STM32F4xx\stm32f401xe.h
  0
};

/*
 * Custom speed SPI configuration (656kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig cs_spicfg1 = {
  false,
  NULL,
  PORT_SPI2_CS,
  PIN_SPI2_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_0, // | SPI_CR1_CPOL, //mask definition in C:\ChibiStudio\chibios_trunk\os\common\ext\ST\STM32F4xx\stm32f401xe.h
  0
};

/*
* Maximum speed SPI configuration (42MHz, CPHA=0, CPOL=0, MSb first).
*/ //Fonctionne avec SPI_SELECT_MODE_NONE
static const SPIConfig hs_spicfg = {
  false,
  NULL,
  PORT_SPI2_CS,
  PIN_SPI2_CS,
  0,
  0
};

#endif /* SPI_CONF_H_ */
