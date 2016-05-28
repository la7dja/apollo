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


#include <avr/io.h>
#include <kernel/analog.h>

double adc_reference_voltage;

uint16_t adc_get_value (double *voltage) {
  uint16_t adc;

  if (bit_is_set (ADMUX, ADLAR))
    adc = ADCH;
  else {
    uint8_t low = ADCL;
    adc = ((uint16_t)ADCH << 8) | low;
  }
  if (voltage) {
    *voltage = adc_to_voltage (adc);
  }
  return adc;
}

uint16_t adc_do_conversion (double *voltage)
{
  adc_start_conversion ();
  while (adc_conversion_in_progress ())
    ;
  return adc_get_value (voltage);
}

uint16_t adc_read_channel (uint8_t channel, double *voltage)
{
  adc_set_mux (channel);
  return adc_do_conversion (voltage);
}

void comparator_set_negative_input (int8_t input)
{
  if (input == AIN1) {
    clear_bit (ADCSRB, ACME);
    set_bit (DIDR1, AIN1D);
  } else {
    set_bit (ADCSRB, ACME);
    adc_set_mux (input);
  }
}

void comparator_set_bandgap_reference (bool intref)
{
  if (intref)
    set_bit (ACSR, ACBG);
  else {
    clear_bit (ACSR, ACBG);
    set_bit (DIDR1, AIN1D);
  }
}
