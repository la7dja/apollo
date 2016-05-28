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

#ifndef __KERNEL_INTERNAL__
#define __KERNEL_INTERNAL__

#include <stddef.h>

#define ret() asm("ret");
#define reti() asm("reti");

#define SAVE_CALLER_PRESERVED_REGISTERS()				\
  asm ("push __tmp_reg__"            "\n\t"			\
       "in   __tmp_reg__, __SREG__"  "\n\t"			\
       "push __tmp_reg__"            "\n\t"			\
       "push __zero_reg__"           "\n\t"			\
       "clr  __zero_reg__"           "\n\t"			\
       "push r18"                    "\n\t"			\
       "push r19"                    "\n\t"			\
			 "push r20"                    "\n\t"			\
       "push r21"                    "\n\t"			\
       "push r22"                    "\n\t"			\
       "push r23"                    "\n\t"			\
       "push r24"                    "\n\t"			\
       "push r25"                    "\n\t"			\
       "push r26"                    "\n\t"			\
       "push r27"                    "\n\t"			\
       "push r30"                    "\n\t"			\
       "push r31"                    "\n\t")

#define SAVE_CALLEE_PRESERVED_REGISTERS() \
  asm ("push r2"                     "\n\t" \
       "push r3"                     "\n\t" \
       "push r4"                     "\n\t" \
       "push r5"                     "\n\t" \
       "push r6"                     "\n\t" \
       "push r7"                     "\n\t" \
       "push r8"                     "\n\t" \
       "push r9"                     "\n\t" \
       "push r10"                    "\n\t" \
       "push r11"                    "\n\t" \
       "push r12"                    "\n\t" \
       "push r13"                    "\n\t" \
       "push r14"                    "\n\t" \
       "push r15"                    "\n\t" \
       "push r16"                    "\n\t" \
       "push r17"                    "\n\t" \
       "push r28"                    "\n\t" \
       "push r29"                    "\n\t" \
       ::)


#define SAVE_DISPATCH_ADDRESS(dispatch)			\
	asm ("push r26" "\n\t"										\
			 "push r27" "\n\t"										\
			 :: "x" (dispatch))

       
#define SAVE_STACK(task) \
  asm ("in  r24, __SP_L__"          "\n\t" \
       "in  r25, __SP_H__"          "\n\t" \
       "st  z, r24"                 "\n\t" \
       "std z+1, r25"               "\n\t" \
       :: "z" (&(task->sp)) : "r24", "r25")


#define SWITCH_STACK(sp) \
  asm ("movw r24, r30"              "\n\t" \
       "out __SP_L__, r24"          "\n\t" \
       "out __SP_H__, r25"          "\n\t" \
       :: "z" (sp) : "r24", "r25")

#define RESTORE_STACK() \
  SWITCH_STACK(&current_task->sp)

#define RESTORE_CALLEE_PRESERVED_REGISTERS() \
  asm ("pop  r29"                   "\n\t" \
       "pop  r28"                   "\n\t" \
       "pop  r17"                   "\n\t" \
       "pop  r16"                   "\n\t" \
       "pop  r15"                   "\n\t" \
       "pop  r14"                   "\n\t" \
       "pop  r13"                   "\n\t" \
       "pop  r12"                   "\n\t" \
       "pop  r11"                   "\n\t" \
       "pop  r10"                   "\n\t" \
       "pop  r9"                    "\n\t" \
       "pop  r8"                    "\n\t" \
       "pop  r7"                    "\n\t" \
       "pop  r6"                    "\n\t" \
       "pop  r5"                    "\n\t" \
       "pop  r4"                    "\n\t" \
       "pop  r3"                    "\n\t" \
       "pop  r2"                    "\n\t");
       
#define RESTORE_ALL_REGISTERS() \
  RESTORE_CALLEE_PRESERVED_REGISTERS();	   \
  asm ("pop  r31"                   "\n\t" \
       "pop  r30"                   "\n\t" \
       "pop  r27"                   "\n\t" \
       "pop  r26"                   "\n\t" \
       "pop  r25"                   "\n\t" \
       "pop  r24"                   "\n\t" \
       "pop  r23"                   "\n\t" \
       "pop  r22"                   "\n\t" \
       "pop  r21"                   "\n\t" \
       "pop  r20"                   "\n\t" \
       "pop  r19"                    "\n\t" \
       "pop  r18"                    "\n\t" \
       "pop  __zero_reg__"          "\n\t" \
       "pop  __tmp_reg__"           "\n\t" \
       "out  __SREG__,__tmp_reg__"  "\n\t" \
       "pop  __tmp_reg__")


#define SAVE_FULL_CONTEXT()																			\
	SAVE_CALLER_PRESERVED_REGISTERS();														\
  asm ("lds r30, current_task" "\n\t"														\
			 "lds r31, current_task+1" "\n\t"													\
			 "ldd __tmp_reg__, Z + %[save_context]" "\n\t"						\
			 "ldd r31, Z + %[save_context] + 1" "\n\t"								\
			 "mov r30, __tmp_reg__" "\n\t"														\
			 "icall" "\n\t"																						\
			 :: [save_context] "M" (offsetof(task_t, save_context)))


#define SWITCH_TO_SYSTEM_STACK() SWITCH_STACK (SYSTEM_STACK + SYSTEM_STACK_SIZE - 1)

#define DISPATCH_NEXT_TASK()										\
  pick_next_task();															\
  SWITCH_STACK(current_task->sp);								\
  ret()


#endif 
