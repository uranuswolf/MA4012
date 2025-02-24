#include "MotorControl.h"
#include "PIDControl.h"

// Global variable for maximum motor speed (ticks per second)
int maxMotorSpeed = 3000; // Example value, adjust based on your motor specifications

void controlMotors(int leftPower, int rightPower) {
    motor[motorL] = leftPower;
    motor[motorR] = rightPower;
}

int getEncoderSpeed(int previousEncoder, int currentEncoder, int deltaTime) {
    int deltaTicks = currentEncoder - previousEncoder;
    return deltaTicks * 1000 / deltaTime;  // Calculate speed in ticks per second (scaled to milliseconds)
}
