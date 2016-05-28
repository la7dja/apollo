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

#include <avr/power.h>
#include <kernel/spi.h>

static lock_t spi_mutex;
task_t *spi_waiting;

void spi_init ()
{
#if 0
  for (int i = 0; i < SPI_NUM_PORTS; i++)
    lock_init(&spi_mutex[i]);
#endif
}


void spi_acquire (spi_device_t *device)
{
  lock_acquire(&spi_mutex);
  power_spi_enable();

  switch (device->clock_rate) {
  case   4: SPCR = 0; clear_bit(SPSR, SPI2X); break;
  case  16: SPCR = 1; clear_bit(SPSR, SPI2X); break;
  case  64: SPCR = 2; clear_bit(SPSR, SPI2X); break;
  case 128: SPCR = 3; clear_bit(SPSR, SPI2X); break;
  case   2: SPCR = 0; set_bit(SPSR, SPI2X); break;
  case   8: SPCR = 1; set_bit(SPSR, SPI2X); break;
  case  32: SPCR = 2; set_bit(SPSR, SPI2X); break;
  }
    
  if (device->mode == SPI_MODE_MASTER) {
    set_bit(SPCR, MSTR);
    make_output(SCLK);
    if (device->output)
      make_output(MOSI);
  } else {
    clear_bit(SPCR, MSTR);
    if (device->output)
      make_output(MISO);
  }

  if (!device->bit_order == SPI_MSB_FIRST)
    clear_bit(SPCR, DORD);
  else
    set_bit(SPCR, DORD);

  if (device->clock_polarity == SPI_CLOCK_IDLE_LOW)
    clear_bit(SPCR, CPOL);
  else
    set_bit(SPCR, CPOL);

  if (device->clock_phase == SPI_SAMPLE_LEADING_EDGE)
    clear_bit(SPCR, CPHA);
  else
    set_bit(SPCR, CPHA);

  device->can_write = true;
  device->can_read = false;

  set_bit(SPCR, SPE); 
}

void spi_release (spi_device_t *device)
{
  clear_bit(SPCR, SPE);
  power_spi_disable();
  lock_release(&spi_mutex);
}

static void spi_wait (spi_device_t *device)
{
#if 1
  spi_waiting = current_task;
  cli();
  set_bit(SPCR, SPIE);
  suspend();
#else
  while (!bit_is_set (SPSR, SPIF))
    ;
#endif
  device->can_read = device->can_write = true;
}

void spi_wait_write (spi_device_t *device)
{
  if (!spi_can_write(device))
    spi_wait(device);
}

void spi_wait_read (spi_device_t *device)
{
  if (!spi_can_read(device))
    spi_wait(device);
}

ISR_RESUME_TASK (SPI_STC)
{
  clear_bit(SPCR, SPIE);
	return spi_waiting;
}

uint16_t spi_read (spi_device_t *device, void *data, uint16_t length)
{
	int n;

	for (n = 0; n < length; n++) {
		/* TODO: all but the first read, need a timeout */
		((uint8_t*)data)[n] = spi_read_byte(device);
		if (!spi_slave_selected(device)) break;
	}

	return n;
}

/* uint16_t spi_transfer (spi_device_t *device, uint8_t *in, uint8_t *out, uint16 length) */
/* { */
/*   wait(spi_is_ready, spi_device); */
/*   for (int i = 0; i < length; i++) { */
/*     SPDR = out ? out[i] : 0; */
/*     if (in && i > 0) */
/*       in[i - 1] = SPDR; // Read buffered data from previous iteration */
/*     if (device->mode == SPI_MODE_MASTER) */
/*       wait(spi_is_ready, spi_device); */
/*     else { */
/*       bool slave_is_ready(spi_device_t *device) { */
/*       	return spi_is_ready(device) || !spi_slave_selected(device); */
/*       } */
/*       wait(slave_is_ready, device); */
/*       if (!spi_slave_selected(device)) */
/* 	return i; */
/*     } */
/*   } */
/*   if (in) { */
/*     wait(spi_is_ready, spi_device); */
/*     in[length - 1] = SPDR; */
/*   } */
  
/*   return length; */
/* } */
