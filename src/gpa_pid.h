#ifndef GPA_PID_H_
#define GPA_PID_H_
#include <stdint.h>
#include <stdlib.h>

#define PID_INTEGRAL_LIMIT 50

typedef struct
{
	float Kp,Ki,Kd;
	float last, integral;
} pid_t;

float pid_update(pid_t* pid, float r, float y, float dt);


#endif




