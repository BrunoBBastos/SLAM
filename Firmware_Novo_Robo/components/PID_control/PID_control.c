#include <stdio.h>
#include "PID_control.h"

#include "esp_log.h"


void PIDController_Init(PIDController *pid) {
	pid->integrator = 0.0f;
	pid->previousError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->previousMeasurement = 0.0f;

	pid->output = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{

    float error = setpoint - measurement;

    float proportional = pid->Kp * error;

    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->previousError);

    // Rotina anti-windup do integrador
    float limMaxInt, limMinInt;
    if (pid->limMax > proportional) limMaxInt = pid->limMax - proportional;
    else limMaxInt = 0.0f;
    if (pid->limMin < proportional) limMinInt = pid->limMin - proportional;
    else limMinInt = 0.0f;

    if (pid->integrator > limMaxInt) pid->integrator = limMaxInt;
    else if (pid->integrator < limMinInt) pid->integrator = limMinInt;

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->previousMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    pid->output = proportional + pid->integrator + pid->differentiator;

    if (pid->output > pid->limMax) pid->output = pid->limMax;
    else if (pid->output < pid->limMin) pid->output = pid->limMin;

    pid->previousError = error;
    pid->previousMeasurement = measurement;

    return pid->output;
}