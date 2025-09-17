#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	float Kp;
	float Ki;
	float Kd;

	// constante de tempo do PB do derivador
	float tau;

	float limMin;
	float limMax;

	float T;

	// memória
	float integrator;
	float previousError;
	float differentiator;
	float previousMeasurement;

	float output;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif