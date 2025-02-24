#include "motorControl.h"  // Include the header file for motor control

// motorControl.c - Function to control the motors

void controlMotors(int leftSpeed, int rightSpeed)
{
    motor[leftmotor] = leftSpeed;
    motor[rightmotor] = rightSpeed;
}
