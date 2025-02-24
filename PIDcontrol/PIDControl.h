#ifndef PIDCONTROL_H
#define PIDCONTROL_H

extern float Kp;  // Proportional gain
extern float Ki;  // Integral gain
extern float Kd;  // Derivative gain
extern float integral;  // Integral sum
extern float previousError;  // Previous error for derivative

float pidControl(int setpoint, int currentSpeed, int maxMotorSpeed);

#endif
