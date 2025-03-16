#ifndef PID_H
#define PID_H

// PID Constants (Tune these)
#define Kp 0.8
#define Ki 0.01
#define Kd 0.5

// PID Function
int computePID(int desiredSpeed, int actualSpeed);

void resetPID();

#endif
