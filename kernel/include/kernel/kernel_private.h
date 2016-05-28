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

#ifndef __KERNEL_PRIVATE__
#define __KERNEL_PRIVATE__

#include <stdint.h>

/** \file kernel_private.h
 *
 *  \brief Definitions private to the kernel code.
 *
 *  This file contains definitios which are considered private to the
 *  kernel, but could be needed when writing extension to it. Should
 *  normaly not be included in user code.
 */

/**
 *  Default system stack size. User configurable.
 */
#if !defined(SYSTEM_STACK_SIZE)
#ifdef DEBUG
#define SYSTEM_STACK_SIZE 128
#else
#define SYSTEM_STACK_SIZE 128 //64
#endif
#endif

/* We keep a two byte stack for the idle task at top of memory and
	 allow it to grow into the system stack just below, to hold caller
	 preserved registers and return address when doing context
	 switch. */
#ifdef DEBUG
#define IDLE_TASK_STACK_SIZE 32
#else
#define IDLE_TASK_STACK_SIZE 6
#endif
#define IDLE_TASK_STACK  ((uint8_t*)RAMEND - IDLE_TASK_STACK_SIZE + 1)

/* A dedicated stack used when doing context switch. Must be at least
	 15 bytes to allow caller perserved registers to be saved when
	 switcing from the idle task. */
#define SYSTEM_STACK (IDLE_TASK_STACK - SYSTEM_STACK_SIZE + 1)



/**
 *  System clock prescaler. User configurable.
 */
#ifndef SYS_CLOCK_PRESCALE
#define SYS_CLOCK_PRESCALE 1024 /* gives 128 µs resolutin and a counter
																	 periode of ~8 seconds at 8MHz */
#endif

/**
 *  Scheduling time slot in ticks. User configurable.
 */
#ifndef PREEMPT_TIME_SLOT
#define PREEMPT_TIME_SLOT 78 /* Approximately 10 ms with 128 µs resolution */
#endif

#define TICKS_PER_SECOND (F_CPU / SYS_CLOCK_PRESCALE)




/**
 *  Maximum number of PIDs which can be assigned. User configurable.
 */
#if !defined(TASK_MAX_PIDS)
#define TASK_MAX_PIDS 10 
#endif

#if !defined(TASK_MAIN_STACK_SIZE)
#define TASK_MAIN_STACK_SIZE TASK_DEFAULT_STACK_SIZE
#endif

/**
 * States a task can be in.
 */ 
typedef enum {
  TASK_STATE_READY,       /*!< Task is ready to run */
  TASK_STATE_SUSPENDED,   /*!< Task is suspended */
  TASK_STATE_EXITED       /*!< Task has terminated */
} task_state_t;


#define EXIT_CODE(task) *(int*)(task)->stack


/**
 *  Structure used to hold information about tasks.
 */
typedef uint8_t pid_t;
typedef struct _task task_t;
struct _task {
  uint8_t *sp;              /*!< Saved stack pointer */
  uint8_t *stack;           /*!< Start of stack area */
  uint16_t stack_size;      /*!< Stack size in bytes */
  pid_t pid;                /*!< Numeric ID */
  int8_t priority;          /*!< Priority level task is running at */
  task_state_t state;       /*!< Current state of task */
	void (*save_context)(void);

  task_t *next;             /*!< Pointer to next task in scheduler queue */
  task_t *next_waiting;     /*!< Pointer to next task in waiting queue */
  task_t *next_sleeping;    /*!< Pointer to next task in sleeping queue */
  tick_t wakeup;            /*!< Time when sleeping task should be resumed */
	
#ifdef DEBUG
  char *name;
#endif
};



/**
 *  Initializes a \c task_t structure.
 *
 *  \param task Pointer to task structure which should be initialized
 *  \param entry Task entry point
 *  \param stack Pointer to begining of the stack area
 *  \param stack_size Size of stack area
 *  \param priority Initial priority of the task
 */
void init_task (task_t *task, uint8_t priority,
								uint8_t *stack, uint16_t stack_size);

void task_stack_push (task_t *task, void *addr);

void pick_next_task ();

extern task_t *pids[];
static inline task_t* find_task (uint8_t pid) { return pids[pid]; }


/**
 *  Spinlock type
 */
typedef uint8_t spinlock_t;


/**
 *  Initializes a spinlock
 *
 *  \param lock Pointer to the spinlock which should be initialized.
 */
#if defined(__DOXYGEN__)
void spinlock_init (spinlock_t *spinlock);
#else
static inline void spinlock_init (spinlock_t *spinlock) { *spinlock = 0; }
#endif

/**
 *  Acquires a spinlock. Calls \c yield in a loop until the lock
 *  becomes available, if taken.
 *
 *  \param lock Pointer to the spinlock which should be acquired.
 */
void spinlock_acquire (spinlock_t *lock);

/**
 *  Releases a spinlock.
 *
 *  \param lock Pointer to the spinlock which should be released.
 */
void spinlock_release (spinlock_t *spinlock);


/**
 *  Mutex structure.
 */
struct _lock {
  spinlock_t spinlock; /*!< Spinlock protecting the mutex */
  uint8_t owner;       /*!< Pid of task owning the mutex or 0 if not taken */
  uint8_t recursions;  /*!< Number of recursive acquisitions */
  task_t *waiting;     /*!< List of tasks waiting for the mutex */
};


/**
 *  Condition variable structure.
 */
struct _condition {
  spinlock_t spinlock; /*!< Spinlock protecting the condition variable */
  task_t *waiting;     /*!< List of tasks waiting for the condition variable */
};



/**
 *  Mailbox structure.
 */
struct _mbox {
  uint8_t *buffer;         /*!< Buffer to hold messages */
  uint8_t length;          /*!< Length of buffer in bytes */
  uint8_t in;              /*!< Index where elemetes should be written */
  uint8_t out;             /*!< Index where elemetes should be read */
  uint8_t request;         /*!< Data or space in bytes requested by task */
  uint8_t count;           /*!< Number of elements (bytes) in buffer */
  spinlock_t spinlock;
  task_t *waiting;
};


/**
 *  Pointer to the current running task.
 */
extern task_t *current_task;

/**
 *  Inserts a task into the running queue for its priority level.
 *
 *  \param task The task to insert
 */
void insert_task (task_t *task);


/**
 *  emoves a task from its running queue.
 *
 *  \param task The task to remove
 */
void remove_task (task_t *task);


/**
 *  Resumes a task which have been suspended.
 *
 *  \param task The task to resume
 */
void resume_task (task_t *task);



/**
 *  Invokes the scheduler to switch from the calling task to another
 *  task.
 *
 *  \param state New state of the calling task
 *  \param task Task to switch to or \c NULL
 *  \param spinlock A spinlock to release or \c NULL
 */
void switch_task (task_state_t state, task_t *task, spinlock_t *spinlock);

void _start_task ();
void _restart_task ();
    
/**
 *  Suspend the calling task for a limited time, and appends it to a
 *  waiting queue.
 *
 *  \param queue Pointer to waiting queue
 *  \param spinlock Spinlock to protect the queue
 *  \param wakeu Wakeup time in system ticks
 *
 *  \return \c true if unblocked by call to \c unblock or by call to
 *  \c wakeup if not blocked on a queue
 */
bool block_with_wakeup (task_t **queue, spinlock_t *spinlock, tick_t wakeup);


/**
 *  Suspends the calling task and appends it to a waiting queue.
 *
 *  \param queue Pointer to waiting queue
 *  \param spinlock Spinlock to protect the queue
 */
void block (task_t **queue, spinlock_t *spinlock);
 
 
/**
 *  Unblocks first task in queue. The queue should be protected by the
 *  spinlock used when blocking.
 *
 *  \param queue Pointer to queue
 */
void unblock (task_t **queue);


/**
 *  Unblocks all tasks in queue. The queue should be protected by the
 *  spinlock used when blocking.
 *
 *  \param queue Pointer to queue
 */
void unblock_all (task_t **queue);


/**
 *  Appends a task to a queue.
 *
 *  \param queue Pointer to waiting queue
 *  \param task Task to append
 */
void queue_append (task_t **queue, task_t *task);


/**
 *  Removes a task from a queue.
 *
 *  \param queue Pointer to waiting queue
 *  \param task Task to remove
 *
 */
void queue_remove (task_t **queue, task_t *task);


void insert_sleeping (task_t *task, tick_t wakeup);
bool remove_sleeping (task_t *task);


//extern uint8_t *system_stack;
extern uint8_t* const system_stack;

void restore_full_context ();
void restore_context ();
void dispatch_task ();

#endif 
