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

#include <math.h>
#include <avr/power.h>
#include <kernel/analog.h>

//#undef USB_DEBUG // unncomment to disable debugging output from this file
#include "apollo.h"


#define FWD_CHANNEL 0
#define REV_CHANNEL 1

lock_t atu_lock;
condition_t atu_tune_cond;
uint32_t tunig_frequency;
uint8_t atu_status = ATU_STATUS_RESET;

uint8_t active_inductor_relays;
int16_t active_capacitor_relays;


#define L1S E, 7
#define L1R E, 6
#define L2S D, 5
#define L2R D, 4
#define L3S E, 1
#define L3R E, 0
#define L4S E, 5
#define L4R E, 4
#define L5S B, 6
#define L5R B, 7
#define L6S B, 5
#define L6R B, 4
#define L7S E, 3
#define L7R F, 7
#define L8S D, 3
#define L8R D, 2

#define C1S A, 0
#define C1R A, 1
#define C2S A, 2
#define C2R A, 3
#define C3S A, 4
#define C3R A, 5
#define C4S A, 6
#define C4R A, 7
#define C5S C, 7
#define C5R C, 6
#define C6S C, 5
#define C6R C, 4
#define C7S C, 3
#define C7R C, 2
#define C8S C, 1
#define C8R C, 0

#define CBANKS D, 7
#define CBANKR D, 6


bool latch_inductors (int value)
{
  if (value != active_inductor_relays) {
    uint8_t set = (value ^ active_inductor_relays) & value;
    uint8_t reset = (value ^ active_inductor_relays) & ~value;
    active_inductor_relays = value;
                
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); set_port (L1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); set_port (L2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); set_port (L3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); set_port (L4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); set_port (L5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); set_port (L6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); set_port (L7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); set_port (L8S); }
        
    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); set_port (L1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); set_port (L2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); set_port (L3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); set_port (L4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); set_port (L5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); set_port (L6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); set_port (L7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); set_port (L8R); }
        
    usleep (RELAY_LATCH_TIME);
        
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (L1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (L2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (L3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (L4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (L5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (L6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (L7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (L8S); }
                
    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (L1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (L2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (L3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (L4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (L5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (L6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (L7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (L8R); }

    return true;
  }
        
  return false;
}


bool latch_capacitors (int value)
{
  if (value != active_capacitor_relays) {
    uint16_t set = (value ^ active_capacitor_relays) & value;
    uint16_t reset = (value ^ active_capacitor_relays) & ~value;
    active_capacitor_relays = value;
                
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); set_port (C1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); set_port (C2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); set_port (C3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); set_port (C4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); set_port (C5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); set_port (C6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); set_port (C7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); set_port (C8S); }
    if (set & 0x100) { usleep (RELAY_LATCH_DELAY); set_port (CBANKS); }

    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); set_port (C1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); set_port (C2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); set_port (C3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); set_port (C4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); set_port (C5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); set_port (C6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); set_port (C7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); set_port (C8R); }
    if (reset & 0x100) { usleep (RELAY_LATCH_DELAY); set_port (CBANKR); }
                
    usleep (RELAY_LATCH_TIME);
                
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (C1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (C2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (C3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (C4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (C5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (C6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (C7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (C8S); }
    if (set & 0x100) { usleep (RELAY_LATCH_DELAY); clear_port (CBANKS); }

    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (C1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (C2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (C3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (C4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (C5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (C6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (C7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (C8R); }
    if (reset & 0x100) { usleep (RELAY_LATCH_DELAY); clear_port (CBANKR); }

    return true;
  }

  return false;
}

bool latch_capacitors_signed (int value)
{
  uint16_t unsigned_value = value < 0 ? 0x100 - value : value;
  return latch_capacitors (unsigned_value);
}
        

void init_atu ()
{
  make_output (L1S);
  make_output (L2S);
  make_output (L3S);
  make_output (L4S);
  make_output (L5S);
  make_output (L6S);
  make_output (L7S);
  make_output (L8S);
  make_output (L1R);
  make_output (L2R);
  make_output (L3R);
  make_output (L4R);
  make_output (L5R);
  make_output (L6R);
  make_output (L7R);
  make_output (L8R);

  make_output (C1S);
  make_output (C2S);
  make_output (C3S);
  make_output (C4S);
  make_output (C5S);
  make_output (C6S);
  make_output (C7S);
  make_output (C8S);
  make_output (C1R);
  make_output (C2R);
  make_output (C3R);
  make_output (C4R);
  make_output (C5R);
  make_output (C6R);
  make_output (C7R);
  make_output (C8R);
  
  make_output (CBANKS);
  make_output (CBANKR);
        
  uint8_t inductors = eeprom_read_byte(&settings.atu_inductors);
  uint16_t capacitors = eeprom_read_word(&settings.atu_capacitors);

  /* Make sure the relays are in expected states. */
  active_inductor_relays = ~inductors;  
  active_capacitor_relays = ~capacitors;        
  latch_inductors (inductors);
  latch_capacitors (capacitors);
        
  power_adc_enable ();
  /* adc_set_reference (ADC_REFERENCE_VCC); */
  adc_set_reference (ADC_REFERENCE_INTERNAL);
  adc_enable ();
}

double get_phi ()
{
  uint32_t fwd = 0;
  uint32_t rev = 0;
  int num_samples = 100;
        
  for (int i = 0; i < num_samples; i++) {
    fwd += adc_read_channel (FWD_CHANNEL, NULL);
    rev += adc_read_channel (REV_CHANNEL, NULL);
  }

	//  INFO("Fwd=%f Rev=%f", (double)fwd / num_samples, (double)rev / num_samples);
        
  /* Assuming readout is linear, which may not be correct */
  double phi = (double)rev / (double)fwd;

  return phi;
}

double get_vswr ()
{
  double phi = get_phi ();
  return (1 + phi) / (1 - phi);
}

double compute_vswr (double phi)
{
  return (1 + phi) / (1 - phi);
}


static double latch_relays (bool (*latch)(int), int index)
{
  if (latch (index))
    usleep (RELAY_SETTLE_TIME); // TODO: adaptive waiting
  return get_phi ();
}


static int binary_search (bool (*latch)(int), int min, int max)
{
  if (!ptt_enabled) return 0;
        
  double phi_min;
  double phi_max;

  /* If the order is reversed when min < max, we will avoid an extra
     latching step (ie. the first call to latch_relays will not change
     the setting of any relays) when the capacitor range is extended
     in negative direction */
  if (min < max) {
    phi_min = latch_relays (latch, min);
    phi_max = latch_relays (latch, max);
  } else {
    phi_max = latch_relays (latch, max);
    phi_min = latch_relays (latch, min);
  }

  while (ptt_enabled && min != max) {
		//    INFO("min=%d max=%d", min, max);
    if (phi_min < phi_max) {
      /* Select lower half */
      max = min + (max - min) / 2;
      phi_max = latch_relays (latch, max);                      
    } else {
      /* Select upper half */
      min = max - (max - min) / 2;
      phi_min = latch_relays (latch, min);
    }           
  }
  INFO("val=%d", min);
  return min;
}

void atu_tune(uint32_t frequency)
{
  lock_acquire (&atu_lock);
	if (atu_status != ATU_STATUS_IN_PROGRESS) {
		tunig_frequency = frequency;
		condition_signal (&atu_tune_cond);
	}
  lock_release (&atu_lock);
}


void reset_atu()
{
  lock_acquire (&atu_lock);
  tunig_frequency = 0;
  condition_signal (&atu_tune_cond);
  lock_release (&atu_lock);
}

void disable_atu()
{
  reset_atu ();
}

void enable_atu()
{
  /* Currently a no-op, should restore settings from EEPROM */
}

uint32_t atu_get_status ()
{
  lock_acquire (&atu_lock);
  pull_down (STATUS);
  uint32_t status = 0;
  if (atu_status != ATU_STATUS_IN_PROGRESS) {
		status = active_capacitor_relays;
		status = (status & 0x1ff) << 8;			
    status |= active_inductor_relays;
	}
  status |=  (uint32_t)atu_status << 20;
  lock_release (&atu_lock);
  return status;
}

TASK (atu, TASK_PRIORITY_MEDIUM /* - 1 */, TASK_DEFAULT_STACK_SIZE)
{
	lock_acquire (&atu_lock);
  while (true) {
    condition_wait (&atu_tune_cond, &atu_lock);

    int inductors_max, capacitors_max;
    if (tunig_frequency == 0) {
			atu_status = ATU_STATUS_IN_PROGRESS;
      lock_release (&atu_lock);

      latch_inductors (0);
      latch_capacitors (0); 
			eeprom_write_byte(&settings.atu_inductors, 0);
			eeprom_write_word(&settings.atu_capacitors, 0);
				
			lock_acquire (&atu_lock);
      atu_status = ATU_STATUS_RESET;      
      pull_up (STATUS);
      continue;
    } 
		/* TODO: optimize cutoff limits */
		else if (tunig_frequency < 5000000)
      inductors_max = capacitors_max = 255;
    else if  (tunig_frequency < 10000000)
      inductors_max = capacitors_max = 128;
    else if  (tunig_frequency < 20000000)
      inductors_max = capacitors_max = 64;
    else if  (tunig_frequency < 20000000)
      inductors_max = capacitors_max = 32;
    else if  (tunig_frequency < 40000000)
      inductors_max = capacitors_max = 16;
    else
      inductors_max = capacitors_max = 8;
                
    atu_status = ATU_STATUS_IN_PROGRESS;
    pull_up (STATUS);
    lock_release (&atu_lock);

    if (ptt_enabled) {
      usleep (PTT_DELAY);
      latch_capacitors (0);

      /* Initial tune with limited range */
      int i = binary_search (latch_inductors, 0, inductors_max);
      int c = binary_search (latch_capacitors_signed, -capacitors_max, capacitors_max);

      /* Extend range if we have reached maximum value with current cutoff */
      while (i == inductors_max && inductors_max < 255) {
        int inductors_min = i;
        inductors_max = min (255, inductors_max * 2);
        i = binary_search (latch_inductors, inductors_min, inductors_max);
      }

      while (abs (c) == capacitors_max && capacitors_max < 255) {
        int capacitors_min = abs (c);
        capacitors_max = min (255, capacitors_max * 2);
        if (c > 0)
          c = binary_search (latch_capacitors_signed, capacitors_min, capacitors_max);
        else
          c = binary_search (latch_capacitors_signed, -capacitors_max, -capacitors_min);
      }                 

			double vswr = get_vswr();
			INFO ("VSWR=%f", vswr);
      lock_acquire (&atu_lock);
      if (!ptt_enabled || vswr > ATU_MAX_VSWR)
        atu_status = ATU_STATUS_FAILED;
      else 
        atu_status = ATU_STATUS_SUCCEEDED;                      

			eeprom_write_byte(&settings.atu_inductors, active_inductor_relays);
			eeprom_write_word(&settings.atu_capacitors, active_capacitor_relays);
				
			pull_up (STATUS);
    } else {
      lock_acquire (&atu_lock);
      atu_status = ATU_STATUS_FAILED;
      pull_up (STATUS);
    }
  }
}
