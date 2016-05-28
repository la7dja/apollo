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

#ifndef __ANALOG_H__
#define __ANALOG_H__

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include "ports.h"

/* Analog to Digital Converter */

enum {
  ADC0,
  ADC1,
  ADC2,
  ADC3,
  ADC4,
  ADC5,
  ADC6,
  ADC7,
  ADC_MUX_GND = 0x1f
};

#define ADC_INTERNAL_REF_VOLTAGE 2.56

typedef enum {
  ADC_REFERENCE_AREF,
  ADC_REFERENCE_VCC,
  ADC_REFERENCE_INTERNAL = 3
} adc_ref_t;

extern double adc_reference_voltage;

static inline void adc_enable () { set_bit(ADCSRA, ADEN); }
static inline void adc_disable () { clear_bit(ADCSRA, ADEN); }

static inline uint8_t adc_get_mux () { return ADMUX & 0x1f; }
static inline void  adc_set_mux (uint8_t channel) {
  ADMUX = (ADMUX & 0xe0) | channel;
}

static inline void adc_set_reference (adc_ref_t ref) {
  ADMUX = (ADMUX & 0x3f) | (ref << 6);
}
static inline adc_ref_t adc_get_reference (adc_ref_t ref) {
  return (ADMUX & 0xc0) >> 6;
}

static inline void adc_set_internal_reference () {
  adc_reference_voltage = ADC_INTERNAL_REF_VOLTAGE;
  adc_set_reference(ADC_REFERENCE_INTERNAL);
}

#ifdef VCC
static inline void adc_set_vcc_reference () {
  adc_reference_voltage = VCC;
  adc_set_reference(ADC_REFERENCE_VCC);
}
#else
#define adc_set_vcc_reference() { \
  adc_reference_voltage = VCC; \
  adc_set_reference(ADC_REFERENCE_VCC); \
}
#endif

static inline void adc_set_external_reference (double voltage) {
  adc_reference_voltage = voltage;
  adc_set_reference(ADC_REFERENCE_AREF);
}

static inline void adc_set_8bit () { set_bit(ADMUX, ADLAR); }
static inline void adc_set_10bit () { clear_bit(ADMUX, ADLAR); }
static inline uint8_t adc_get_bits () {
  return bit_is_set(ADMUX, ADLAR) ? 8 : 10;
}

static inline void adc_start_conversion () { set_bit(ADCSRA, ADSC); }
static inline bool adc_conversion_in_progress () {
  return bit_is_set(ADCSRA, ADSC);
}
uint16_t adc_get_value (double *voltage);
uint16_t adc_do_conversion (double *voltage);
uint16_t adc_read_channel (uint8_t channel, double *voltage);

static inline double adc_to_voltage (uint16_t value) {
  return value * adc_reference_voltage / (1 << adc_get_bits());
}


/* TODO: add auto trigger and differential input API */

/* Analog Comparator */

#define AIN1 -1

enum {
  COMP_INT_MODE_OUTPUT_TOGGLE,
  COMP_INT_MODE_FALLING_EDGE = 2,
  COMP_INT_MODE_RAISING_EDGE
};

static inline void comparator_enable () { clear_bit(ACSR, ACD); }
static inline void comparator_disable () { set_bit(ACSR, ACD); }

void comparator_set_negative_input (int8_t input);
void comparator_bandgap_reference (bool internal);

static inline void comparator_set_interrup_mode (uint8_t mode) {
  ACSR &= 0xfc; ACSR |= mode;
}
static inline void comparator_enable_interrup () { set_bit(ACSR, ACIE); }
static inline void comparator_disable_interrup () { clear_bit(ACSR, ACIE); }

static inline void comparator_enable_input_capture () { set_bit(ACSR, ACIC); }
static inline void comparator_disable_input_capture () { clear_bit(ACSR, ACIC); }

static inline bool comparator_flag_is_set () { return bit_is_set(ACSR, ACI); }
static inline void comparator_clear_flag () { set_bit(ACSR, ACI); }


#endif /* __ANALOG_H__ */
