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

#ifndef __PORTS_H__
#define __PORTS_H__

#include <ctype.h>
#include <stdint.h>

/*
 * Bit fiddling macros
 */
#define __port_template(prefix,op,port,pin) prefix##port op _BV(P##port##pin)
#define __set_ddr(port, pin) __port_template(DDR, |=, port, pin)
#define __clear_ddr(port, pin) __port_template(DDR, &=~, port, pin)
#define set_port(x, ...) __port_template(PORT, |=, x, ##__VA_ARGS__)
#define clear_port(x, ...) __port_template(PORT, &=~, x, ##__VA_ARGS__)
#define toggle_port(x, ...) __port_template(PIN, |=, x, ##__VA_ARGS__)
#define port_is_set(x, ...) (__port_template(PORT, &, x, ##__VA_ARGS__))
#define pin_is_high(x, ...) (__port_template(PIN, &, x, ##__VA_ARGS__))
#define pin_is_low(x, ...) (!pin_is_high(x, ##__VA_ARGS__))

/* Use float (or tri-state as Atmel calls it) if pin has external
   pull-up. Be carefull when switching direction. */
#define make_output(x, ...) __set_ddr(x, ##__VA_ARGS__);
#define make_input(x, ...) \
  ({ __clear_ddr (x, ##__VA_ARGS__); set_port(x, ##__VA_ARGS__); })
#define make_float(x, ...) \
  ({ __clear_ddr (x, ##__VA_ARGS__); clear_port(x, ##__VA_ARGS__); })
#define pull_down(x, ...) \
  ({ clear_port (x, ##__VA_ARGS__); __set_ddr(x, ##__VA_ARGS__); })
#define pull_up(x, ...) \
  ({ set_port (x, ##__VA_ARGS__); __set_ddr(x, ##__VA_ARGS__); })

#define pull_down_port pull_down
#define pull_up_port pull_up

#define set_bit(sfr,bit) (sfr)|=_BV(bit)
#define clear_bit(sfr,bit) (sfr)&=~_BV(bit)
#define toggle_bit(sfr,bit) (sfr)^=_BV(bit)

#endif
