#include "task_os.h"


volatile int timeout = 0;

/*
 * Initialize tasks.
 */
void task_init(task_t* tasks, uint16_t n){
	for (int i = 0; i < n; ++i){
		if (tasks[i]->init == NULL)
			continue;
		tasks[i]->init(tasks[i]->user);
	}
}

/*
 * Execute tasks, returns 0 on success or corresponding error code.
 */
task_ret_t task_update(task_t* tasks, uint16_t n){
	for (int i = 0; i < n; ++i)
	{
		if (tasks[i]->update == NULL)
			continue;

		task_ret_t r = tasks[i]->update(task[i]->user);

		if (r != TASK_DO_CONTINUE){
			return TASK_DONT_CONTINUE;
		}
		if (timeout){
			return TASKERR_TIMEOUT;
		}
	}
	return TASK_DO_CONTINUE;
}

/*
 * Abort execution of any remaining tasks.
 */
void task_timeout(){
	timeout = 1;
}





