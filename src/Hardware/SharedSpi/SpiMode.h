/*
 * SpiMode.h
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDSPI_SPIMODE_H_
#define SRC_HARDWARE_SHAREDSPI_SPIMODE_H_

#include "spi_com.h"

enum SpiMode {
  mode0 = SPI_MODE_0,
  mode1 = SPI_MODE_1,
  mode2 = SPI_MODE_2,
  mode3 = SPI_MODE_3,
};

#endif /* SRC_HARDWARE_SHAREDSPI_SPIMODE_H_ */
