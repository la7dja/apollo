/* This file is part of the OS kernel included with the Apollo firmware 
	 Copyright (C) 2016  Espen S. Johnsen, LA7DJA

	 This program is free software: you can redistribute it and/or modify
	 it under the terms of the GNU General Public License as published by
	 the Free Software Foundation, either version 3 of the License, or
	 (at your option) any later version.
	 
	 This program is distributed in the hope that it will be useful,
	 but WITHOUT ANY WARRANTY; without even the implied warranty of
	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	 GNU General Public License for more details.
	 
	 You should have received a copy of the GNU General Public License
	 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef __SPI_H__
#define __SPI_H__

#include <stdbool.h>
#include <avr/io.h>
#include "ports.h"
#include "kernel.h"

#define SS B, 0
#define SCLK B, 1
#define MOSI B, 2
#define MISO B, 3


typedef enum {
  SPI_MODE_MASTER,
  SPI_MODE_SLAVE
} spi_mode_t;

typedef enum {
  SPI_CLOCK_IDLE_LOW,
  SPI_CLOCK_IDLE_HIGH
} spi_clock_polarity_t;

typedef enum {
  SPI_SAMPLE_LEADING_EDGE,
  SPI_SAMPLE_TRAILING_EDGE,
  SPI_SETUP_LEADING_EDGE = SPI_SAMPLE_TRAILING_EDGE,
  SPI_SETUP_TRAILING_EDGE = SPI_SAMPLE_LEADING_EDGE,  
} spi_clock_phase_t;

typedef enum {
  SPI_MSB_FIRST,
  SPI_LSB_FIRST
} spi_bit_order_t;


typedef struct _spi_device {
  spi_mode_t mode;
  bool output;
  spi_bit_order_t bit_order;
  spi_clock_polarity_t clock_polarity;
  spi_clock_phase_t clock_phase;
  uint8_t clock_rate;
  bool can_read; // data available in input buffer
  bool can_write; // output buffer empty
} spi_device_t;


/**
 *  Initialize the SPI subsystem
 *
 */
void spi_init ();

/**
 *  Acquires exclusive access to the SPI bus
 *
 */
void spi_acquire (spi_device_t *device);


/**
 *  Releases the SPI bus
 *
 */
void spi_release (spi_device_t *device);



#define WITH_SPI_DEVICE(device) \
  for (bool done = false, spi_acquire(device); !done; done = true, spi_release(device))

static inline bool spi_can_write (spi_device_t *device) {
  return device->can_write || bit_is_set(SPSR, SPIF);
	//  return device->can_write; // || bit_is_set(SPSR, SPIF);
}

static inline bool spi_can_read (spi_device_t *device) {
  return device->can_read || bit_is_set(SPSR, SPIF);
  /* return device->can_read; // || bit_is_set(SPSR, SPIF); */
}


static inline bool spi_slave_selected (spi_device_t *device) {
  return pin_is_low(SS);
}

uint16_t spi_read (spi_device_t *device, void *data, uint16_t length);

void spi_wait_write (spi_device_t *device);
void spi_wait_read (spi_device_t *device);

static inline void spi_write_byte (spi_device_t *device, uint8_t data) {
	spi_wait_write(device);
  device->can_write = false;
  SPDR = data;
}

static inline uint8_t spi_read_byte (spi_device_t *device) {
  spi_wait_read(device);
  device->can_read = false;	
	return SPDR;
}

static inline uint8_t spi_transfer_byte (spi_device_t *device, uint8_t data) {
	spi_write_byte(device, data);
	return spi_read_byte(device);
}

static inline bool spi_read_uint16_le (spi_device_t *device, uint16_t *data) {
	return spi_read(device, data, sizeof(uint16_t)) == sizeof(uint16_t);
}

static inline bool spi_read_uint32_le (spi_device_t *device, uint32_t *data) {
	return spi_read(device, data, sizeof(uint32_t)) == sizeof(uint32_t);
}

/* uint16_t spi_transfer (spi_device_t *device, uint8_t *data, uint16 length, spi_direction_t direction); */

static inline uint16_t spi_write (spi_device_t *device, uint8_t *data, uint16_t length) {
  /* spi_transfer(device, NULL, data, length); */
  for (int i = 0; i < length; i++)
    spi_write_byte(device, data[i]);
  return length;
}

#endif /* __SPI_H__ */
