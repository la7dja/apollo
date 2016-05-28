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

//#undef USB_DEBUG // unncomment to disable debugging output from this file
#include "apollo.h"

#define LPF_BYPASS 6
uint32_t lpf_cutoff[] = {2000000, 4000000, 8000000, 16000000, 24000000, 36000000, UINT32_MAX};            

lock_t lpf_lock;
condition_t lpf_cond;

uint8_t current_band, new_band;
bool lpf_enabled;

void lpf_write (uint16_t data)
{
  for (int i = 0; i < 16; i++) {
    clear_port (SRCK);
    if (data & 0x8000)
      set_port (SER_IN);
    else
      clear_port (SER_IN);
    
    set_port (SRCK);
    data <<= 1;
  }
  clear_port (SER_IN);
  
  set_port (RCK);
  usleep (50);
  clear_port (RCK);
}


void init_lpf ()
{
  pull_down (SRCK);
  pull_down (RCK);
  pull_down (SER_IN);

  /* Make sure the relays are in expected states. We're only
     activating/releasing one pair of relay coils at a time to prevent
     voltage drops and spikes which could lock up the MCU. */
  current_band = eeprom_read_byte(&settings.lpf_current_band);
  if (current_band > LPF_BYPASS) current_band = LPF_BYPASS;
  uint16_t relays = 0;
  for (int i = 0; i < 7; i++) {
    relays |= (i == current_band ? 1 : 2) << (i * 2);
    lpf_write (relays);
    usleep (RELAY_LATCH_DELAY);
  }
  usleep (RELAY_LATCH_TIME);
  for (int i = 0; i < 7; i++) {
    relays &= ~(3 << (i * 2));
    lpf_write (relays);
    usleep (RELAY_LATCH_DELAY);
  }

  lpf_enabled = eeprom_read_byte(&settings.lpf_enabled);
}

void update_lpf ()
{
  if (lpf_enabled) {
    uint8_t band = LPF_BYPASS;
    for (int i = 0; i < sizeof (lpf_cutoff) / sizeof (uint32_t); i++)
      if (frequency <= lpf_cutoff[i]) {
        band = i;
        break;
      }
    lock_acquire (&lpf_lock);
    if (band != current_band) {
      reset_atu ();
			//      eeprom_write_byte(&settings.lpf_current_band, band);
      new_band = band;
			condition_signal (&lpf_cond);
    }
    lock_release (&lpf_lock);
  }
}
        
void enable_lpf ()
{
  if (!lpf_enabled) {
    lpf_enabled = true;
		//    eeprom_write_byte(&settings.lpf_enabled, true);
    update_lpf ();
  }
}

void disable_lpf ()
{
  if (lpf_enabled) {
    lpf_enabled = false;
    eeprom_write_byte(&settings.lpf_enabled, false);
    lock_acquire (&lpf_lock);
    current_band = LPF_BYPASS;
    condition_signal (&lpf_cond);
    lock_release (&lpf_lock);
  }
}

uint8_t lpf_get_status ()
{
  uint8_t band = 0;
        
  if (lpf_enabled) {            
    lock_acquire (&lpf_lock);
    band = current_band + 1;
    lock_release (&lpf_lock);
  }
  return band;
}

TASK (lpf, TASK_PRIORITY_MEDIUM + 1, TASK_DEFAULT_STACK_SIZE)
{
  lock_acquire (&lpf_lock);
  condition_wait (&lpf_cond, &lpf_lock);

  while (true) {
    uint8_t band = new_band;
    lock_release (&lpf_lock);

    lpf_write ((uint16_t)1 << (band * 2));
    usleep (RELAY_SETTLE_TIME);
    lpf_write ((uint16_t)2 << (current_band * 2));
    usleep (RELAY_LATCH_TIME);
    lpf_write (0x0000);

    lock_acquire (&lpf_lock);
    current_band = band;
    if (current_band == new_band)
      condition_wait (&lpf_cond, &lpf_lock);
  }
}
