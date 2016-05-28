/* Apollo ATU firmware
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <kernel/spi.h>

//#undef USB_DEBUG // unncomment to disable debugging output from this file
#include "apollo.h"


/* Message constants */

#define MSG_SET_FREQ 1
#define MSG_ENABLE_PTT 2
#define MSG_DISABLE_PTT 3
#define MSG_START_TUNING 4
#define MSG_ABORT_TUNING 5
#define MSG_GET_STATUS 6
#define MSG_GET_VERSION 7

spi_device_t spi = {
  .mode = SPI_MODE_SLAVE,
  .output = true,
  .clock_polarity = SPI_CLOCK_IDLE_LOW,
  .clock_phase = SPI_SAMPLE_LEADING_EDGE,
	.bit_order = SPI_LSB_FIRST
};

uint32_t frequency;

struct eeprom EEMEM settings;

TASK(heartbeat, TASK_PRIORITY_MEDIUM + 1, TASK_DEFAULT_STACK_SIZE)
{
  make_output (LED2);
  while (true) {
    toggle_port (LED2);
    usleep (100000);
  }
}


lock_t ptt_lock;
condition_t ptt_change;
time_t ptt_timeout = 0;
bool ptt_enabled = false;


TASK (ptt, TASK_PRIORITY_MEDIUM + 1, TASK_DEFAULT_STACK_SIZE)
{
  pull_down (PTT_LED);
  pull_down (PTT_OUT);
  lock_acquire (&ptt_lock);
  while (true) {
    tick_t time = get_system_ticks ();
    if (time_after(ptt_timeout, time)) {
      set_port (PTT_LED);
      set_port (PTT_OUT);
      ptt_enabled = true;
      condition_wait_with_wakeup (&ptt_change, &ptt_lock, ptt_timeout);
    } else {
      clear_port (PTT_OUT);
      clear_port (PTT_LED);
      ptt_enabled = false;
      condition_wait (&ptt_change, &ptt_lock);
    }
  }
}

void enable_ptt (uint16_t timeout)
{
  lock_acquire (&ptt_lock);
  INFO ("Enable PTT with %u ms timeout", timeout);
  ptt_timeout = get_system_ticks () + ms_to_ticks (timeout);
  condition_signal (&ptt_change);
  lock_release (&ptt_lock);
}

void disable_ptt ()
{
  lock_acquire (&ptt_lock);
  INFO ("Disable PTT");
  ptt_timeout = get_system_ticks();
  condition_signal (&ptt_change);
  lock_release (&ptt_lock);
}

static void read_msg_data (uint32_t *msg)
{
	uint8_t byte = (uint8_t)*msg;
	for (int n = 0; n < 3; n++) {
		byte = spi_transfer_byte(&spi, ~byte);
		*msg = *msg << 8 | byte;
	}
	spi_transfer_byte(&spi, ~byte);
}

static void write_msg_data (uint32_t msg)
{
	for (int n = 0; n < 4; n++) {
		uint8_t byte = (uint8_t)(msg >> 24);
		spi_transfer_byte(&spi, ~byte);
		msg = msg << 8;
	}
}


int main ()
{
	INFO ("Initializing");

	init_lpf ();
	init_atu ();

  spi_init ();
  spi_acquire (&spi);

	INFO ("Starting main loop");

	pull_up (STATUS);	
  while (true) {
		uint32_t msg = spi_transfer_byte(&spi, 0xff);
		switch (msg) {
      case MSG_SET_FREQ: {
				read_msg_data(&msg);
				
				frequency = (msg & 0x003fffff) << 4;
				bool lpf = msg & 0x00800000;
				/* bool atu = msg & 0x00400000; */
				
				INFO ("Setting frequency to %lu Hz", frequency);
				if (lpf) {
					enable_lpf ();
					update_lpf ();
				} else
					disable_lpf ();
				break;
			}
      case MSG_ENABLE_PTT: {
				read_msg_data(&msg);
				uint16_t timeout = msg & 0xffff;
 				if (!ptt_enabled)
					INFO ("PTT enabled with timeout %u ms\n\r", timeout);
				enable_ptt (timeout);
				break;
			}
      case MSG_DISABLE_PTT:
				read_msg_data(&msg);
				/* This message seems to create some instability when tuning,
					 so for now it is disabled. PTT will be disabled after a
					 short timeout when Hermes stops sendig PTT enable updates */
				/* disable_ptt (); */
				/* INFO ("PTT disabled\n\r"); */
				break;
				
      case MSG_START_TUNING:
				read_msg_data(&msg);
				atu_tune (frequency);
				break;

      case MSG_ABORT_TUNING:
				read_msg_data(&msg);
				// TODO
				break;

			case MSG_GET_STATUS:
				msg = msg << 24 | atu_get_status ();
				write_msg_data (msg);
				break;

			case MSG_GET_VERSION:
				msg = msg << 8 | VERSION_MAJOR;
				msg = msg << 8 | VERSION_MINOR;
				msg = msg << 8 | VERSION_MICRO;
				write_msg_data(msg);				
				break;
					
			default:
				/* Unknown message, read error or out of sync. */
				INFO ("Unknown message\n\r");
		}
  }
}
