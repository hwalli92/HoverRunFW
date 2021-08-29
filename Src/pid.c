#include "pid.h"

void pid_init(PID_Control *PID, double *pidin, double *pidout, double *pidsetpt, double kp, double ki, double kd)
{
	PID->input = pidin;
	PID->output = pidout;
	PID->setpoint = pidsetpt;
	PID->kp = kp;
	PID->ki = ki;
	PID->kd = kd;
	PID->iterm = 0.0;
	PID->last_angle = 0.0;
	PID->last_time = NAN;
	PID->max_pid = 100;
}

void pid_compute(PID_Control *PID)
{
	double this_time = millis();
	double input = *PID->input;
	double output;
	double dt;

	if (isnan(PID->last_time))
	{
		dt = 1e-16;
	}
	else
	{
		dt = this_time - PID->last_time;
	}

	double error = *PID->setpoint - input;

	PID->iterm += dt * error;

	double dterm = (PID->last_angle - input) / dt;

	output = (PID->kp * error) + (PID->ki * PID->iterm) + (PID->kd * dterm);

	if (output > PID->max_pid)
		*PID->output = PID->max_pid;
	else if (output < -PID->max_pid)
		*PID->output = -PID->max_pid;
	else
		*PID->output = output;

	PID->last_time = this_time;
	PID->last_angle = input;
}

void set_tuning_params(PID_Control *PID, double kp, double ki, double kd)
{
	PID->kp = kp;
	PID->ki = ki;
	PID->kd = kd;
}