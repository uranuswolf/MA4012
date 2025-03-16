#include "pid.h"

int computePID(int setpoint, int current) {
    static int lastError = 0;
    static int integral = 0;
    
    int error = setpoint - current;
    integral += error;
    int derivative = error - lastError;
    
    float Kp = 1.2, Ki = 0.01, Kd = 0.5;
    int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    lastError = error;
    
    return output;
}
