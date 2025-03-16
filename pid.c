#include "pid.h"

// PID variables
int error, prevError = 0;
int integral = 0;
int derivative = 0;

// PID Calculation
int computePID(int desiredSpeed, int actualSpeed) {
    error = desiredSpeed - actualSpeed;
    integral += error;
    derivative = error - prevError;
    prevError = error;

    // Compute PID output
    int output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Limit output to motor range (-127 to 127)
    if (output > 127) output = 127;
    if (output < -127) output = -127;

    return output;
}

void resetPID() {
    error = 0;
    prevError = 0;
    integral = 0;
    derivative = 0;
}