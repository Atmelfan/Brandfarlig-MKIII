#include "gpa_pid.h"

float pid_update(pid_t* pid, float r, float y, float dt){
	float e = r - y;
	float de = 0, ie = 0;
	float out;

	if(pid->Ki != 0){
		pid->integral += e*dt;
		ie = pid->integral;
		if (pid->integral >= PID_INTEGRAL_LIMIT){
			pid->integral = PID_INTEGRAL_LIMIT;
		}else if(pid->integral <= -PID_INTEGRAL_LIMIT){
			pid->integral = -PID_INTEGRAL_LIMIT;
		}
	}
	
	if(pid->Kd != 0){
		de = (e - pid->last)/dt;
	}

	out = pid->Kp*e + pid->Ki*ie + pid->Kd*de;//Kp*e(t) + Ki*integrate(0, t, e(x)) + d/dt*e(t)

	pid->last = e;
	return out;
}