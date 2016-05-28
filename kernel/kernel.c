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

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>


#ifdef DEBUG_KERNEL
#define DEBUG
#endif

#ifdef DEBUG
#include <util/atomic.h>
#include <stdio.h>

#define DEBUG_INIT
__attribute__((weak)) void debug_init () { }

#ifdef SIMULAVR
int debug_putchar (char ch, FILE *stream)
{
  *((volatile char *)DEBUG_OUTPUT) = ch;
  return 0;
}
#else
int debug_putchar (char ch, FILE *stream);
#endif

FILE debug_output = FDEV_SETUP_STREAM(debug_putchar, NULL, _FDEV_SETUP_WRITE);
void debug_printf_P (const char* fmt, ...) {
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf_P(&debug_output, fmt, ap);
    va_end(ap);
    fflush(&debug_output);
  }
}
#endif

#ifndef DEBUG_KERNEL
#undef DEBUG
#endif

#include <kernel/kernel.h>


/* Scheduler state */
task_t *task_queues[TASK_NUM_PRIORITY_LEVELS];
task_t *current_task;
int next_priority;
spinlock_t scheduler_queue_spinlock;
bool in_critical_section;

task_t* pids[TASK_MAX_PIDS];
spinlock_t pid_spinlock;

volatile uint16_t sys_ticks_high = 0;

volatile int8_t idle_sleep_mode;
#ifdef DYNAMIC_CLOCK_SPEED
uint8_t clkps;
#endif

void save_full_context ();

/* It would be tempting to just use the xch instruction to set the
	 spinlock value to 1, but that may lead to priority inversion when a
	 high priority task tries to acquire a spinlock held by low priority
	 task which is being prevented from running by a medium priority
	 task. To avoid this, the value of the spinlock is set to the pid of
	 the task holding it. This would enable a high priority task to lend
	 its priority to a lower priority task for the purpose of releasing
	 the spinlock as soon as possible.
*/
static inline pid_t test_and_set (spinlock_t *spinlock, pid_t pid)
{
	pid_t owner;	

	asm volatile("in   __tmp_reg__, __SREG__\n\t"									\
							 "cli\n\t"																				\
							 "ld   %0, z\n\t"																	\
							 "cpi  %0, 0\n\t"																	\
							 "brne done_%=\n\t"																\
							 "st   z, %2\n"																		\
							 "done_%=:\n\t"																		\
							 "out  __SREG__, __tmp_reg__\n\t"									\
							 : "=r" (owner) : "z" (spinlock), "r" (pid) : );
	
	return owner; // current owner of spinlock or zero if spinlock was free
}


task_t *sleeping_queue;
spinlock_t sleeping_queue_spinlock;



tick_t get_system_ticks ()
{
  uint16_t high;
  uint16_t low;

	ATOMIC_BLOCK (ATOMIC_RESTORESTATE) { 
    high = sys_ticks_high;
    low = TCNT1;
		if (TIFR1 & _BV(TOV1)) {
			TIFR1 |= _BV(TOV1);
			high = ++sys_ticks_high;
			low = TCNT1;
		}
	}
  return ((tick_t)high << 16) | low;
}

/* Spinlocks */

void spinlock_acquire (spinlock_t *spinlock)
{
	/* Multitasking is not started if getpid returns zero and thus the
		 lock should not be acquired */
  pid_t owner, pid = getpid();
	if (pid) {
		while ((owner = test_and_set(spinlock, pid))) {
			ASSERT(owner != pid); // a task shouldn't acquire spinlock if it
														// already has it
			task_t *task = find_task(owner);
			INFO("spinlock=0x%04x held by %s", spinlock, task->name);
			switch_task(TASK_STATE_READY, task, NULL);
		}
		INFO("acquired spinlock=0x%04x", spinlock);
	}
}

static inline void _spinlock_release (spinlock_t *spinlock) {
	INFO("spinlock=0x%04x", spinlock);
	*spinlock = 0;
}

void spinlock_release (spinlock_t *spinlock) {
	if (*spinlock) {
		INFO("spinlock=0x%04x", spinlock);
		*spinlock = 0;
		yield();
	}
}



/* Idle task */

static __attribute__((OS_task)) __attribute__((noreturn)) int idle () 
{
	sei(); // the idle task doesn't start through a dispatch function which
				 // would normally enable interrupts
  while (true) {
#ifndef SIMULAVR
    if (idle_sleep_mode != SLEEP_MODE_NONE)
      sleep_mode();
#endif
  }
}

static task_t idle_task;

static __attribute__((naked)) void save_idle_context ()
{
	IDLE_TASK_STACK[IDLE_TASK_STACK_SIZE - 1] = (uint16_t)idle;
  IDLE_TASK_STACK[IDLE_TASK_STACK_SIZE - 2] = (uint16_t)idle >> 8;
	
	ret();
}

static void create_idle_task()
{
  init_task(&idle_task, TASK_PRIORITY_IDLE, IDLE_TASK_STACK, IDLE_TASK_STACK_SIZE);
#ifdef DEBUG
	idle_task.name = "idle";
#endif
	idle_task.save_context = save_idle_context;
  *(idle_task.sp--) = (uint16_t)idle;
  *(idle_task.sp--) = (uint16_t)idle >> 8;
	INFO("stack=0x%04x entry=0x%04x", idle_task.sp, (uint16_t)(*(uint8_t*)(idle_task.sp + 1)) << 8 | *(uint8_t*)(idle_task.sp + 2));
  insert_task(&idle_task);
  resume_task(&idle_task);
}


/* Initialization */

#ifndef SIMULAVR
__attribute__((naked)) __attribute__((section (".init3")))
void init_hw ()
{
  MCUSR = 0;
  wdt_disable();

#ifdef CLKPR
  /* Set CPU speed to maximum */
  CLKPR = 0x80;
  CLKPR = 0;
#endif

  /* Disable all peripherals to save power */
  power_all_disable();

  set_cpu_sleep_mode(SLEEP_MODE_IDLE);
}
#endif

__attribute__((naked)) __attribute__((section(".init5")))
void init_system ()
{
#ifdef DEBUG_INIT
  debug_init();
#endif

  INFO("Initializing system");

	SWITCH_TO_SYSTEM_STACK(); // Must do this before creating the idle
														// task to avoid the idle task stack being
														// overwritten
	
  /* Create the idle task. Must be the first task created to be assign
     PID zero. It should not acquire any spinlocks as zero value in a
     spinlock indicates that it is free. */
	create_idle_task();
  
  /* Creating the main task. Nothing special about this except that a
     main symbol is required to link with libc. */
	CREATE_TASK(main, TASK_MAIN_STACK_SIZE, TASK_PRIORITY_MEDIUM);

  /* Setting up timer 1 in normal mode as system timer. */
#ifndef SIMULAVR
  power_timer1_enable();
#endif
  TCCR1A = 0;
  TIMSK1 = _BV(TOV1); /* Disable Compare Match and enable Overflow interrupt */
#if SYS_CLOCK_PRESCALE == 1024
  TCCR1B = 5;
#elif SYS_CLOCK_PRESCALE == 256
  TCCR1B = 4;
#elif SYS_CLOCK_PRESCALE == 64 
  TCCR1B = 3;
#elif SYS_CLOCK_PRESCALE == 8
  TCCR1B = 2;
#elif SYS_CLOCK_PRESCALE == 1
  TCCR1B = 1;
#endif	
}

__attribute__((naked)) __attribute__((section(".init8")))
void start_multitasking ()
{
  INFO("Starting multi-tasking");
	
	pick_next_task();
  SWITCH_STACK(current_task->sp);
  ret();
}


/* Task management */

void init_task (task_t *task, uint8_t priority,
								uint8_t *stack, uint16_t stack_size)
{
  if (current_task)
    spinlock_acquire(&pid_spinlock);

  uint8_t pid;
  for (pid = 0; pid < TASK_MAX_PIDS; pid++)
    if (pids[pid] == NULL) {
      pids[pid] = task;
      task->pid = pid;
      break;
    }
  ASSERT(pid < TASK_MAX_PIDS);
  
  if (current_task)
    spinlock_release(&pid_spinlock);

  task->sp = stack + stack_size - 1;
  task->stack = stack;
  task->stack_size = stack_size;
  task->priority = priority;
  task->state = TASK_STATE_SUSPENDED;
	task->save_context = save_full_context;
}

/* Dispatch function used when a task is run for the first time */
__attribute__((naked)) void dispatch_task ()
{
  reti();
}


void task_stack_push (task_t *task, void *addr)
{
  *(task->sp--) = (uint16_t)addr;
  *(task->sp--) = (uint16_t)addr >> 8;
}


task_t* create_task (int (*entry)(void), uint16_t stack_size, uint8_t priority)
{
  uint8_t* stack = malloc(stack_size);
  task_t *task = malloc(sizeof(task_t));
  init_task(task, priority, stack, stack_size);
	task_stack_push(task, exit);
	task_stack_push(task, entry);
	task_stack_push(task, dispatch_task);
  insert_task(task);
  resume_task(task);

  return task;
}

void free_task (task_t* task)
{
  ASSERT(task->state == TASK_STATE_EXITED);

  spinlock_acquire(&pid_spinlock);
  pids[task->pid] = NULL;
  spinlock_release(&pid_spinlock);
  
  free(task->stack);
  free(task);
}

void task_setpriority (task_t *task, uint8_t priority) {
	/* Changing task priority dynamically is not yet supported */
  ERROR("Not implemented");
}


/* Task queue management */

void resume_task (task_t *task)
{
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
		task->state = TASK_STATE_READY;
		if (task->priority > next_priority)
			next_priority = task->priority;
		INFO("%s", task->name);
	}
}

void insert_task (task_t *task)
{
	spinlock_acquire(&scheduler_queue_spinlock);
  INFO("%s", task->name);
  task_t *head = atomic_get_pointer(&task_queues[task->priority]);
  if (head == NULL) {
    task->next = task;
    atomic_set_pointer(&task_queues[task->priority], task);
  } else {
    task->next = head->next;
    atomic_set_pointer(&head->next, task);
  }
	spinlock_release(&scheduler_queue_spinlock);
}

void _remove_task (task_t *task)
{
  if (task->next == task)
    /* Last task at this priority level */
    atomic_set_pointer(task_queues[task->priority], NULL);
	else {
    task_t *tmp = task;
    while (task->next != task)
      tmp = tmp->next;

    ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
			tmp->next = task->next;
      if (task_queues[task->priority] == task)
				task_queues[task->priority] = task->next;
    }
  }
}

void remove_task (task_t *task)
{
	ASSERT(task != current_task);
  spinlock_acquire(&scheduler_queue_spinlock);
  INFO("%s", task->name);
	_remove_task(task);
  spinlock_release(&scheduler_queue_spinlock);
}


/* Task switching */

void pick_next_task ()
{
	if (in_critical_section) return;

	do {
		task_t *tmp = task_queues[next_priority];
		if (tmp != NULL)
			do {
				if (tmp->state == TASK_STATE_READY) {
					if (tmp != current_task) {
						current_task = tmp;
						INFO("New current task", current_task->name);
					}

					do
						tmp = tmp->next;
					while (tmp->state != TASK_STATE_READY || tmp != current_task);
					task_queues[next_priority] = tmp;
					
					if (tmp != current_task) {						
						TIFR1 |= _BV(OCF1B); // enable preemption interrupt 
						OCR1B = TCNT1 + PREEMPT_TIME_SLOT;
						TIMSK1 |= _BV(OCIE1B);
					} else
						TIMSK1 &= ~_BV(OCIE1B); // disable preemption interrupt

					return;
				}
				tmp = tmp->next;
			} while (tmp != task_queues[next_priority]);			
	} while (--next_priority >= 0);
	ERROR("No ready task found (where is the idle task?)");
}


__attribute__((naked)) __attribute__((noreturn)) void exit (int status)
{
  EXIT_CODE(current_task) = status;

	/* We need the spinlock to update queue and to enter critical section to
		 prevent a task switch after the task has been marked as exited */
  spinlock_acquire(&scheduler_queue_spinlock);
	enter_critical_section();

  INFO("status=%d", status);
	current_task->state = TASK_STATE_EXITED;
	_remove_task(current_task);
	cli();
	_spinlock_release(&scheduler_queue_spinlock);
	in_critical_section = false;
	pick_next_task();
  SWITCH_STACK(current_task->sp);
  ret();
}


static inline void _sleep (tick_t delay) {
	suspend_until(get_system_ticks () + delay);
}

void suspend_until (tick_t wakeup)
{
  spinlock_acquire(&sleeping_queue_spinlock);	
  insert_sleeping(current_task, wakeup);
  switch_task(TASK_STATE_SUSPENDED, NULL, &sleeping_queue_spinlock);
}

void usleep (timeout_t delay) {
  _sleep(us_to_ticks(delay));
}

void sleep (timeout_t delay) 
{
  _sleep(TICKS_PER_SECOND * delay);
}


__attribute__((naked)) void restore_context ()
{
	RESTORE_CALLEE_PRESERVED_REGISTERS();
  reti();
}

void switch_to_task(task_t *task)
{
	INFO("Switching to %s", task->name);
	current_task = task;
}

__attribute__((naked))
void switch_task (task_state_t state, task_t *task, spinlock_t *spinlock)
{
  cli();
	SAVE_CALLEE_PRESERVED_REGISTERS();
	/* This is a workaround to create a custom prologue without any
		 preceding compiler generated instructions */
	asm("jmp switch_task_part_2" "\n\t");
}

__attribute__((used)) __attribute__((naked)) __attribute__((noreturn)) static void switch_task_part_2 (task_state_t state, task_t *task, spinlock_t *spinlock)
{	
  SAVE_DISPATCH_ADDRESS(restore_context);
  SAVE_STACK(current_task);
  SWITCH_TO_SYSTEM_STACK();

	//	ASSERT(!in_critical_section);

  if (spinlock != NULL) _spinlock_release(spinlock);
  current_task->state = state;

	//	INFO("saved stack=0x%04x entry=0x%04x", current_task->sp, *(uint8_t*)(current_task->sp + 1) << 8 | *(uint8_t*)(current_task->sp + 2));
  
  if (task) {
		/* INFO("Switching to %s", task->name); */
    /* current_task = task; */
		switch_to_task(task);
	} else
    pick_next_task();

	//	INFO("restored stack=0x%04x entry=0x%04x", current_task->sp, *(uint8_t*)(current_task->sp + 1) << 8 | *(uint8_t*)(current_task->sp + 2));

  SWITCH_STACK(current_task->sp);
  ret();
}

__attribute__((naked)) void restore_full_context ()
{
  RESTORE_ALL_REGISTERS();
  reti();
}

__attribute__((naked)) void save_full_context ()
{
	/* Pop off return address */
  asm ("pop  r18" "\n\t" \
       "pop  r19" "\n\t");

	SAVE_CALLEE_PRESERVED_REGISTERS();
	SAVE_DISPATCH_ADDRESS(restore_full_context);
  SAVE_STACK(current_task);

  asm ("push r19" "\n\t" \
       "push r18" "\n\t" \
			 "ret" "\n\t");
}

/* Preemption timeout */
ISR (TIMER1_COMPB_vect, ISR_NAKED)
{
	SAVE_FULL_CONTEXT();
	SWITCH_TO_SYSTEM_STACK();
  DISPATCH_NEXT_TASK();
}



/* Queue management */

void queue_append (task_t **queue, task_t *task)
{
  if (*queue == NULL)
    *queue = current_task;
  else {
    task_t *tmp = *queue;
    while (tmp->next_waiting)
      tmp = tmp->next_waiting;
    tmp->next_waiting = task;
  }
}

void queue_remove (task_t **queue, task_t *task)
{
  if (*queue == task) {
    *queue = task->next_waiting;
    task->next_waiting = NULL;
    return;
  } else {
    task_t *tmp = *queue;
    while (tmp->next_waiting) {
      if (tmp->next_waiting == task) {
	tmp->next_waiting = task->next_waiting;
	task->next_waiting = NULL;
	return;
      }
      tmp = tmp->next_waiting;
    }
  }
}


/* System timer and sleeping queue managament */

ISR (TIMER1_OVF_vect)
{
  sys_ticks_high++;
}

ISR_TASK(TIMER1_COMPA, TASK_PRIORITY_WAKEUP, TASK_DEFAULT_STACK_SIZE)
{
	spinlock_acquire(&sleeping_queue_spinlock);
	TIMSK1 &= ~_BV(OCIE1A); // Disable timer interrupt
	TIMER1_COMPA_counter = 0;

	/* If remove_sleeping() was called between the time the interrupt
		 was triggered and spinlock was acquired, the queue may be empty
		 now */
  if (sleeping_queue) {
		do {
			/* Keep timer registers up to date */
			TIFR1 |= _BV(OCF1A);
			OCR1A = sleeping_queue->wakeup;

			INFO("wakeup %s at %lu", sleeping_queue->name, sleeping_queue->wakeup);
			
			while (time_after(get_system_ticks(), sleeping_queue->wakeup)) {
				resume_task(sleeping_queue);
				if ((sleeping_queue = sleeping_queue->next_sleeping) == NULL)
					goto exit;

				TIFR1 |= _BV(OCF1A);
				OCR1A = sleeping_queue->wakeup;				
			}
		} while (TIFR1 & _BV(OCF1A)); /* a new timeout may already have
																	   occurred, in which case we handle
																	   it right away instead of
																	   triggering a new interrupt */
		TIMSK1 |= _BV(OCIE1A); // Enable timer interrupt
	}

exit:
  _spinlock_release(&sleeping_queue_spinlock);
}


bool remove_sleeping (task_t *task)
{
  bool found = false;
  spinlock_acquire(&sleeping_queue_spinlock);
  if (sleeping_queue == task) {
    sleeping_queue = task->next_sleeping;
    found = true;
  } else {
    task_t *sleeping = sleeping_queue;
    while (sleeping && time_after(task->wakeup, sleeping->wakeup)) {
      if (sleeping->next_sleeping == task) {
				sleeping->next_sleeping = task->next_sleeping;
				found = true;
				break;
      }
      sleeping = sleeping->next_sleeping;
    }
  }
  spinlock_release(&sleeping_queue_spinlock);
  return found;
}

void insert_sleeping (task_t *task, tick_t wakeup)
{
  task->wakeup = wakeup;
  INFO("%s sleeping to %lu", task->name, wakeup);
  if (sleeping_queue == NULL || time_after(sleeping_queue->wakeup, task->wakeup)) {
    task->next_sleeping = sleeping_queue;
    sleeping_queue = task;   
    OCR1A = task->wakeup;
		TIFR1 |= _BV(OCF1A);
		TIMSK1 |= _BV(OCIE1A); // Enable interrupt to wakeup timer task
    if (time_after(get_system_ticks(), task->wakeup)) {
			/* Resume task directly just in case we missed the interrupt */
      resume_task(TIMER1_COMPA_task);
			INFO("TIMER1_COMPA_task resumed directly");
    }
  } else {
    task_t *sleeping = sleeping_queue;
    while (sleeping->next_sleeping != NULL &&
					 time_after(task->wakeup, sleeping->next_sleeping->wakeup))
      sleeping = sleeping->next_sleeping;

    task->next_sleeping = sleeping->next_sleeping;
    sleeping->next_sleeping = task;
  }
}

/* Block current task on queue, which should be protected by the spinlock */
void block (task_t **queue, spinlock_t *spinlock)
{
  ASSERT(queue != NULL && spinlock != NULL);
  INFO("Blocking on queue 0x%x", queue);

  queue_append(queue, current_task);
  switch_task(TASK_STATE_SUSPENDED, NULL, spinlock);
}

bool block_with_wakeup (task_t **queue, spinlock_t *spinlock, time_t wakeup)
{
  ASSERT(queue != NULL && spinlock != NULL);
  queue_append(queue, current_task);

  INFO("Blocking on queue 0x%x with wakeup %lu", queue, wakeup);

  spinlock_acquire(&sleeping_queue_spinlock);
  insert_sleeping(current_task, wakeup);

  cli();
  _spinlock_release(&sleeping_queue_spinlock);
  switch_task(TASK_STATE_SUSPENDED, NULL, spinlock);
  
  remove_sleeping(current_task);
  spinlock_acquire(spinlock);
	queue_remove(queue, current_task);
	spinlock_release(spinlock);

  return false;
}


/* The queue must be protected by the same spinlock as used for blocking */
void unblock (task_t **queue)
{
  if (*queue != NULL) {
    task_t *task = *queue;
    *queue = (*queue)->next_waiting;
    resume_task(task);
  }
}

inline void unblock_all (task_t **queue)
{
  while (*queue)
    unblock(queue);
}


/* Mailboxes for low level inter task communication (eg. message passing) */

void* mbox_init (mbox_t *mbox, void *buffer, uint8_t length)
{
  memset(mbox, 0, sizeof(mbox_t));

  if (buffer)
    mbox->buffer = buffer;
  else
    mbox->buffer = malloc(length);
	
  mbox->length = length;
  
  return mbox->buffer;
}

void mbox_reset (mbox_t *mbox)
{
  spinlock_acquire(&mbox->spinlock);
  mbox->count = mbox->in = mbox->out = 0;
  spinlock_release(&mbox->spinlock);
}

void _mbox_put (mbox_t *mbox, void *data, uint8_t len)
{
  for (int n = 0; n < len; n++) {
    mbox->buffer[mbox->in] = *(uint8_t*)(data + n);
    if (++mbox->in == mbox->length)
      mbox->in = 0;
  }

  spinlock_acquire(&mbox->spinlock);
  mbox->count += len;
  if (mbox->waiting && mbox->length - mbox->count >= mbox->request) {
    resume_task(mbox->waiting);
    mbox->waiting = NULL;
  }
  spinlock_release(&mbox->spinlock);
}


void mbox_put (mbox_t *mbox, void *data, uint8_t len)
{
  spinlock_acquire(&mbox->spinlock);
  if (mbox->length - mbox->count < len) {
    mbox->waiting = current_task;
    mbox->request = len;
    switch_task(TASK_STATE_SUSPENDED, NULL, &mbox->spinlock);
  } else  
    spinlock_release(&mbox->spinlock);

  _mbox_put(mbox, data, len);
}

bool mbox_put_nb (mbox_t *mbox, void *data, uint8_t len)
{
  if (mbox->length - mbox->count < len)
    return false;

  _mbox_put(mbox, data, len);
  return true;
}


void _mbox_get (mbox_t *mbox, void *data, uint8_t len)
{
  for (int n = 0; n < len; n++) {
    *(uint8_t*)(data + n) = mbox->buffer[mbox->out];
    if (++mbox->out == mbox->length)
      mbox->out = 0;
  }

  spinlock_acquire(&mbox->spinlock);
  mbox->count -= len;
  if (mbox->waiting && mbox->count >= mbox->request) {
    resume_task(mbox->waiting);
    mbox->waiting = NULL;
  }
  spinlock_release(&mbox->spinlock);
}

void mbox_get (mbox_t *mbox, void *data, uint8_t len)
{
  spinlock_acquire(&mbox->spinlock);
  if (mbox->count < len) {
    mbox->waiting = current_task;
    mbox->request = len;
    switch_task(TASK_STATE_SUSPENDED, NULL, &mbox->spinlock);
  } else  
    spinlock_release(&mbox->spinlock);
  
  _mbox_get(mbox, data, len);
}

bool mbox_get_nb (mbox_t *mbox, void *data, uint8_t len)
{
  if (mbox->count < len)
    return false;
  
  _mbox_get(mbox, data, len);
  return true;
}


/* Mutexes */

#ifdef LOCK_INIT_NOT_INLINED
void lock_init (lock_t *lock)
{
  spinlock_init(&lock->spinlock);
  lock->owner = 0;
  lock->recursion = 0;
  lock->waiting = NULL;
}
#endif

uint8_t lock_acquire (lock_t *lock)
{
  spinlock_acquire(&lock->spinlock);
  if (lock->owner == 0) {
    lock->owner = getpid();
    spinlock_release(&lock->spinlock);
  } else if (lock_acquired(lock)) {
    lock->recursions++;
    spinlock_release(&lock->spinlock);    
  } else
    block(&lock->waiting, &lock->spinlock);

  return lock->recursions;
}

void lock_release (lock_t *lock)
{
  spinlock_acquire(&lock->spinlock);
  if (lock->recursions)
    lock->recursions--;
  else if (lock->waiting == NULL)
    lock->owner = 0;
  else {
    lock->owner = lock->waiting->pid;
    unblock(&lock->waiting);
  }
  spinlock_release(&lock->spinlock);
}


/* Condition variables */

void condition_init (condition_t *condition)
{
  spinlock_init(&condition->spinlock);
  condition->waiting = NULL;
}

void condition_wait (condition_t *condition, lock_t *lock)
{
	spinlock_acquire(&condition->spinlock);
  lock_release(lock);
  block(&condition->waiting, &condition->spinlock);
  lock_acquire(lock);
}

bool condition_wait_with_wakeup (condition_t *condition, lock_t *lock, tick_t wakeup)
{
  spinlock_acquire(&condition->spinlock);
  lock_release(lock);
  bool expired = block_with_wakeup(&condition->waiting, &condition->spinlock, wakeup);
  lock_acquire(lock);
  return expired;
}


bool condition_wait_with_timeout (condition_t *condition, lock_t *lock, timeout_t timeout)
{
	tick_t wakeup = get_system_ticks() + timeout;
	return condition_wait_with_wakeup(condition, lock, wakeup);
}

void condition_signal (condition_t *condition)
{
  spinlock_acquire(&condition->spinlock);
  if (condition->waiting)
    unblock(&condition->waiting);
  spinlock_release(&condition->spinlock);
}

void condition_broadcast (condition_t *condition)
{
  spinlock_acquire(&condition->spinlock);
  unblock_all(&condition->waiting);
  spinlock_release(&condition->spinlock);
}



/* Power Management */

void power_down ()
{
  WITHOUT_PREEMPTION {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    set_sleep_mode(idle_sleep_mode);
  }
}

__attribute__((noreturn)) void halt ()
{
	cli();
#ifdef SIMULAVR
#ifdef DEBUG_EXIT
	*((volatile char *)DEBUG_EXIT) = 0;
#endif
	while (true) {}
#else
  wdt_disable();
	while (true) power_down();
#endif
}


/* Thread safe malloc. A patched avr-libc is required */

#ifdef MALLOC		

extern void *__malloc(size_t __size) __ATTR_MALLOC__;
extern void __free(void *__ptr);
extern void *__realloc(void *ptr, size_t len) __ATTR_MALLOC__;

lock_t malloc_lock;

void *malloc (size_t len)
{
  lock_acquire(&malloc_lock);
  void* p = __malloc(len);
  lock_release(&malloc_lock);
  return p;
}

void free (void *p)
{
  lock_acquire(&malloc_lock);
  __free(p);
  lock_release(&malloc_lock);
}

void *realloc (void *ptr, size_t size)
{
  lock_acquire(&malloc_lock);
  void* p = __realloc(ptr, size);
  lock_release(&malloc_lock);
  return p;
}
#endif

__attribute__((naked)) __attribute__((section(".init5")))
void init_malloc ()
{
  __malloc_heap_end = (char*)SYSTEM_STACK;
}

