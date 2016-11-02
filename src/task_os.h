#ifndef _TASK_OS_
#define _TASK_OS_
#include <stdint.h>

#define TASK_INIT(name) void* name##_init(uint16_t id)
#define TASK_UPDATE(name) task_ret_t name##_update(void* user, task_mutex mutex)
#define TASK_ADD(name) {&name##_init, &name##_update}
#define TASK_ADD(init, update) {&init, &update}
#define TASK_LIST(tasklist) task_t tasklist[] =

typedef uint16_t task_mutex;

typedef enum
{
	TASK_DO_CONTINUE = 0,		//Continue to execute tasks
	TASK_DONT_CONTINUE,			//Do not continue
	TASKERR_TIMEOUT,
	TASKERR_UNKNOWN
} task_ret_t;

typedef struct
{
	void* user;
	void (*init)(void*);
	task_ret_t (*update)(void*,task_mutex);
} task_t;

/*
 * Initialize tasks.
 */
void task_init(task_t* tasks, uint16_t n);

/*
 * Execute tasks, returns 0 on success or corresponding error code.
 */
task_ret_t task_update(task_t* tasks, uint16_t n);

/*
 * Abort execution of any remaining tasks.
 */
void task_timeout();

/* ---IMPLEMENT THIS---
 * Called before tasks are executed to start a timer that after timeout period
 * shall call task_timeout() to abort executing any unfinished lower priority tasks.
 */
extern void task_start_timer();

/* ---IMPLEMENT THIS---
 * Called when a error has ocurred.
 * Errors:
 * 	 TASKERR_TIMEOUT 	A timeout was called before all tasks were finished
 *   TASKERR_UNKNOWN	Something else?
 */
extern void task_on_error(task_ret_t err);

#endif