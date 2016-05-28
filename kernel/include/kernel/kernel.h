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

#ifndef __KERNEL_H__
#define __KERNEL_H__

#include <stdint.h>
#include <stdbool.h> 
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#include "kernel_internal.h"


/* Definitions needed by kernel_private.h */
   
typedef uint32_t tick_t;
typedef uint32_t time_t;
typedef int32_t timeout_t;

#include "kernel_private.h"

#ifndef CLK_DIV
#define CLK_DIV 1
#endif

/**
 *  Converts milliseconds to system ticks
 *
 *  \return Number of system ticks
 */
#if defined(__DOXYGEN__)
tick_t ms_to_ticks (time_t ms);
#else
/* Inlining this function does not work for unknown reasons */
/* static inline tick_t ms_to_ticks (time_t ms) */
/* { */
/*   return HZ * ms / 1000; */
/* } */
#define ms_to_ticks(ms) ((F_CPU / 1000 * (ms) / SYS_CLOCK_PRESCALE))
#endif

#define us_to_ticks(us)  ((F_CPU / 1000000 * (us) / SYS_CLOCK_PRESCALE))

extern bool in_critical_section;

/**
 *  \return Number of system clock ticks since startup.
 */
tick_t get_system_ticks ();




/**
 *  Default stack size. User configurable.
 */
#if !defined(TASK_DEFAULT_STACK_SIZE)
#define TASK_DEFAULT_STACK_SIZE 128
#endif


/**
 *  Predefined priority levels.
 */ 
#if defined(__DOXYGEN__)
enum {
  TASK_PRIORITY_LOW,    /*!< Lowest level allowed for any user task */
  TASK_PRIORITY_MEDIUM, /*!< Normal level for user tasks */
  TASK_PRIORITY_HIGH,   /*!< Highest level allowed for any user task */
};
#else
enum {
  TASK_PRIORITY_IDLE, // Priority level only used for idle the task
  TASK_PRIORITY_LOW,
  TASK_PRIORITY_MEDIUM = 5,
  TASK_PRIORITY_USB = 7,
  TASK_PRIORITY_HIGH = 10,
  TASK_PRIORITY_WAKEUP,
  TASK_NUM_PRIORITY_LEVELS
};
#endif




/**
 *  \defgroup task_management Task Managament
 */
/*\@{*/

__attribute__((OS_task)) int main ();

#if defined(__DOXYGEN__)
/**
 *   \c task_t is an opaque structure used to hold information about
 *   a task.
 */
typedef struct _task task_t;
#endif


#ifdef DEBUG_KERNEL
#define _CREATE_TASK(entry, exit, stack_size, priority, suspended) ({	\
	static uint8_t entry##_stack[stack_size];													\
	static task_t entry##_task_info;																	\
	init_task(&entry##_task_info, priority, entry##_stack, stack_size);	\
	entry##_task_info.name = #entry;																	\
	task_stack_push(&entry##_task_info, exit);													\
	task_stack_push(&entry##_task_info, entry);													\
	task_stack_push(&entry##_task_info, dispatch_task);									\
	insert_task(&entry##_task_info);																		\
	if (!suspended) resume_task(&entry##_task_info);										\
	&entry##_task_info; })
#else
#define _CREATE_TASK(entry, exit, stack_size, priority, suspended) ({	\
	static uint8_t stack[stack_size];													\
	static task_t task;																						\
	init_task(&task, priority, stack, stack_size);								\
	task_stack_push(&task, exit);																	\
	task_stack_push(&task, entry);																	\
	task_stack_push(&task, dispatch_task);													\
	insert_task(&task);																						\
  if (!(suspended)) resume_task(&task);													\
  &task; })
#endif


/**
 *  Macro which will create a task with statically allocate task
 *  structure and stack.
 *
 *  \param entry Entry point
 *  \param stack_size Size of stack area
 *  \param priority Priority of the task
 *  \param suspend True if task should be created in suspended state
 *
 *  \return A pointer to a \c task_t structure.
 */
#if defined(__DOXYGEN__)
#define CREATE_TASK(entry, stack_size, priority)
#else
#define CREATE_TASK(entry, stack_size, priority) \
	_CREATE_TASK(entry, exit,  stack_size, priority, false)
#endif



/**
 *  Macro which will create a statically allocated task.
 *
 *  \param name Task name
 *  \param stack_size Size of stack area
 *  \param priority Priority of the task
 *
 */
#if defined(__DOXYGEN__)
#define TASK(name, priority, stack_size) { body }
#else
#define TASK(name, priority, stack_size)															\
  task_t* name##_task;																								\
  __attribute__((OS_task)) int name ();															\
  __attribute__((naked)) __attribute__((section(".init7")))						\
  void __##name##_init() {																						\
    name##_task = CREATE_TASK(name, stack_size, priority);						\
  }																																		\
  int name ()
#endif


/**
 *  Macro which will create an ISR used to resume a task. Body should return
 *  pointer the task info for the process to be resumed or NULL. 
 *
 *  \param name Interrupt name
 *
 */
#define ISR_RESUME_TASK(name)																	\
	static inline task_t* name##_body();												\
  ISR (name##_vect, ISR_NAKED) {															\
		SAVE_FULL_CONTEXT();																			\
    SWITCH_TO_SYSTEM_STACK();																	\
		task_t *task = name##_body();															\
		if (task != NULL) { resume_task(task); }									\
		DISPATCH_NEXT_TASK();																			\
  }																														\
	task_t* name##_body()
	

/**
 *  Macro which will create a task to handle interrupts
 *
 *  \param name Interrupt name
 *  \param stack_size Size of stack area
 *  \param priority Priority of the task
 *
 */
#if defined(__DOXYGEN__)
#define ISR_TASK(name, priority, stack_size) { body }
#else
#define ISR_TASK(name, priority, stack_size)														\
	volatile uint8_t name##_counter = 0;																	\
	volatile bool name##_running = false;																	\
  task_t* name##_task;																									\
  __attribute__((OS_task)) void name ();																\
	ISR_RESUME_TASK(name) {																								\
		name##_counter++;																										\
		if (!name##_running) {																							\
			name##_running = true;																						\
			return name##_task;																								\
		} else																															\
			return NULL;																											\
	}																																			\
	void __attribute__((naked)) name##_exit(int status) {									\
		cli();																															\
		SAVE_DISPATCH_ADDRESS(name##_exit);																	\
		SAVE_DISPATCH_ADDRESS(name);																				\
		if (name##_counter != 0)																						\
			reti();																														\
		name##_running = false;																							\
		current_task->state = TASK_STATE_SUSPENDED;													\
		SAVE_STACK(name##_task);																						\
		SWITCH_TO_SYSTEM_STACK();																						\
		DISPATCH_NEXT_TASK();																								\
	}																																			\
  __attribute__((naked)) __attribute__((section(".init7")))							\
  void __##name##_init() {																							\
    name##_task =																												\
			_CREATE_TASK(name, name##_exit, stack_size, priority, true);			\
  }																																			\
	void name ()
#endif

	


/**
 *  Creates a new task. A \c task_t structure and stack space is
 *  dynamically allocated.
 *
 *  \param entry_routine Task entry routine
 *  \param stack_size Size of stack area
 *  \param priority Initial priority of the task
 *
 *  \return A pointer to a \c task_t structure.
 */
task_t* create_task (int (*entry_routine)(void), uint16_t stack_size, uint8_t priority);


/**
 *  Waits for a task to exit.
 *
 *  \param task Task to wait for
 *
 *  Returns the exit code of \c task.
 */
int join_task (task_t* task);


/**
 *  Frees the \c task_t structure and stack allocated to a task.
 *
 *  \param task Pointer to task that should be freed
 */
void free_task (task_t *task);


/**
 *  \return The PID of current task.
 */
#if defined(__DOXYGEN__)
pid_t getpid ();
#else
static inline pid_t getpid () {
	return current_task != NULL ? current_task->pid : 0;
}
#endif


/**
 *  \return The priority of current task.
 */
#if defined(__DOXYGEN__)
uint8_t getpriority ();
#else
static inline uint8_t getpriority () { return current_task->priority; }
#endif


/**
 *  Sets the priority of the calling task.
 *
 *  \param priority New priority level
 */
void setpriority (uint8_t priority);

/**
 *  Lets current task voluntarily relinquish the processor.
 */
#if defined(__DOXYGEN__)
void yield ();
#else
static inline void yield () {
  switch_task(TASK_STATE_READY, NULL, NULL);
}
#endif


/**
 *  Terminates current task.
 *
 *  \param status Exit status of the task
 */
void exit (int status);


/**
 *  Returns true if time a is after or equal to time b.
 *
 *  \param a Time a
 *  \param b Time b
 */
#if defined(__DOXYGEN__)
bool time_after (tick_t a, tick_t b);
#else
static inline bool time_after (tick_t a, tick_t b)
{
	return (int32_t)a - (int32_t)b >= 0;
}
#endif


/**
 *  Suspends current task for a given number of system ticks.
 *
 *  \param n Number of ticks to sleep
 */
void delay (tick_t n);


/**
 *  Suspends current task for a given number of seconds.
 *
 *  \param delay Time to suspend in second.
 */
void sleep (timeout_t delay);


/**
 *  Suspends current task for a given time.
 *
 *  \param delay Time to suspend in microseconds.
 */
void usleep (timeout_t delay);


/**
 *  Suspends current task until given time
 *
 *  \param n Tick number when task should be resumed
 */
void suspend_until (tick_t wakeup);



/**
 *  Suspends current task.
 *
 */
#if defined(__DOXYGEN__)
void suspend ()
#else
static inline void suspend () {
  switch_task(TASK_STATE_SUSPENDED, NULL, NULL);
}
#endif


/**
 *  Wakes up a task which have been suspended by \c usleep or \c suspend.
 *
 *  \param task Task to wake up
 */
#if defined(__DOXYGEN__)
void wakeup (task_t *task);
#else
static inline void wakeup(task_t *task) { resume_task(task); }
#endif

/*\@}*/


/**
 *  \defgroup atmoic Atomic operations
 */
/*\@{*/

#if !defined(__DOXYGEN__)
static inline bool enter_critical_section () {
	//	ASSERT(!in_critical_section);
	TIMSK1 &= ~_BV(OCIE1B); // Disable preemption interrupt
	in_critical_section = true;
	return true;
}
static inline bool leave_critical_section () {
	in_critical_section = false;
	yield();
	return false;
}
#endif
#define WITHOUT_PREEMPTION				   \
  for (bool _in_critical_section = enter_critical_section(); \
       _in_critical_section == true; \
       _in_critical_section = leave_critical_section())

static inline uint16_t atomic_get_word (uint16_t *addr) {
  uint16_t value;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
    value = *addr;
  }
  return value;
}

static inline void* atomic_get_pointer (void *addr) {
  uint16_t value;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
    value = *(uint16_t*)addr;
  }
  return (void*)value;
}

static inline void atomic_set_word (uint16_t *addr, uint16_t value) {
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
   *addr = value;
  }
}

static inline void atomic_set_pointer (void *addr, void *value) {
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
    *(uint16_t*)addr = (uint16_t)value;
  }
}

/*\@}*/


/**
 *  \defgroup mbox Mailboxes for task inter communication
 */
/*\@{*/

/**
 *  Mailbox type
 */
typedef struct _mbox mbox_t;

/**
 *  Initializes a mailbox structure.
 *
 *  \param mbox Pointer to a mailbox structure
 *  \param buffer Pointer to buffer or NULL to dynamically allocate
 *  \param length Length of buffer in bytes
 */
void* mbox_init (mbox_t *mbox, void *buffer, uint8_t length);

/**
 *  Resets a mailbox back to initial conditions
 *
 *  \param mbox Pointer to a mailbox structure
 */
void mbox_reset (mbox_t *mbox);

void mbox_put (mbox_t *mbox, void *data, uint8_t len);
bool mbox_put_nb (mbox_t *mbox, void *data, uint8_t len);
void mbox_get (mbox_t *mbox, void *data, uint8_t len);
bool mbox_get_nb (mbox_t *mbox, void *data, uint8_t len);


/*\@}*/



/* Mutexes */

/**
 *   \c lock_t is an opaque structure used to hold information about
 *   mutexes.
 */
typedef struct _lock lock_t;



/**
 *  Initializes a mutex. This function is normally not needed as
 *  globally or statically defeinded mutex variables does not need
 *  explicit initialization.
 *
 *  \param lock Pointer to the mutex which should be initialized.
 */
#if defined(__DOXYGEN__)
void lock_init (lock_t *lock);
#else
static inline void lock_init (lock_t *lock) {
  memset(lock, 0, sizeof(lock_t));
}
#endif


uint8_t lock_acquire (lock_t *lock);
void lock_release (lock_t *lock);
static inline bool lock_acquired (lock_t *lock) {
  return lock->owner == getpid();
}
static inline bool lock_acquired_recursivly (lock_t *lock) {
  return lock_acquired(lock) && lock->recursions > 0;
}

#define WITH_LOCK(lock) \
  for (bool done = false, lock_acquire(lock); !done; done = true, lock_release(lock))


/* Condition variables */

/**
 *   \c task_t is an opaque structure that is used to hold information
 *   about condition variables.
 */

typedef struct _condition condition_t;

void condition_init (condition_t *condition);
void condition_wait (condition_t *condition, lock_t *lock);
bool condition_wait_with_wakeup (condition_t *condition, lock_t *lock, tick_t wakeup);
bool condition_wait_with_timeout (condition_t *condition, lock_t *lock, timeout_t timeout);
void condition_signal (condition_t *condition);
void condition_broadcast (condition_t *condition);



/* Power Management */

#define SLEEP_MODE_NONE -1

extern volatile int8_t idle_sleep_mode;

static inline void set_cpu_sleep_mode (int8_t mode) {
  idle_sleep_mode = mode;
  if (mode != SLEEP_MODE_NONE) {
    set_sleep_mode(mode);
  }
}

static inline int8_t get_cpu_sleep_mode () {
  return idle_sleep_mode;
}

#ifdef DYNAMIC_CLOCK_SPEED
void set_cpu_speed (uint8_t speed);

static inline void enable_low_power_mode () {
  set_cpu_speed(8);
  set_cpu_sleep_mode(SLEEP_MODE_STANDBY);
}

static inline void disable_low_power_mode () {
  set_cpu_speed(0);
  set_cpu_sleep_mode(SLEEP_MODE_IDLE);
}
#endif

void power_down ();
void halt ();


/* Debugging support */

#ifdef DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>

void debug_printf_P(const char* fmt, ...);

#define INFO(msg, ...)							\
  {									\
    static const char fmt[] PROGMEM = "%010lu %s@%s: " msg "\r\n";			\
		ATOMIC_BLOCK (ATOMIC_RESTORESTATE) debug_printf_P(fmt, get_system_ticks(), current_task != NULL ? current_task->name : ".init", __func__, ##__VA_ARGS__);	\
  }
#define WARNING(msg, ...) INFO("Warning: " msg, ##__VA_ARGS__)
#define ERROR(msg, ...)						       \
  {								       \
    INFO("Error: " msg, ##__VA_ARGS__);			       \
    halt();							       \
  }
#define ASSERT(expr)						       \
  if (!(expr)) {						       \
    INFO("Assertation failed: %s", #expr);                            \
    halt();							       \
  }
#else
#define INFO(msg, ...) { }
#define WARNING(msg, ...) { }
#define ERROR(msg, ...) { halt(); }
#define ASSERT(expr) { }
#endif

#endif /* __KERNEL_H__ */
