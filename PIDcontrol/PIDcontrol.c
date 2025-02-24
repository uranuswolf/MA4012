#include "PIDControl.h"

float Kp = 1.0;  // Set appropriate value
float Ki = 0.1;  // Set appropriate value
float Kd = 0.05; // Set appropriate value

float integral = 0;
float previousError = 0;

float pidControl(int setpoint, int currentSpeed, int maxMotorSpeed) {
    // Calculate error between desired speed and current speed
    float error = setpoint - currentSpeed;

    integral += error;
    float derivative = error - previousError;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    // Limit motor power to 127 max
    if (output > 127) output = 127;
    if (output < -127) output = -127;

    return output;
}
 
