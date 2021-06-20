#include <stdint.h>
#include <string.h>
#include "setup.h"

typedef struct
{

	double *input;
	double *output;
	double *setpoint;

	double kp;
	double ki;
	double kd;

	double iterm;
	double last_time;
	double last_angle;

} PID_Control;

void pid_init(PID_Control *PID, double *pidin, double *pidout, double *pidsetpt, double kp, double ki, double kd);
void pid_compute(PID_Control *PID);
void set_tuning_params(PID_Control *PID, double kp, double ki, double kd);