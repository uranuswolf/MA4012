#include "MotorControl.h"
#include "PIDControl.h"

// Global variable for maximum motor speed (ticks per second)
int maxMotorSpeed = 3000; // Example value, adjust based on your motor specifications

void controlMotors(int leftPower, int rightPower) {
    motor[motorL] = leftPower;
    motor[motorR] = rightPower;
}

int getEncoderSpeed(int previousEncoder, int currentEncoder, int deltaTime) {
    // Calculate the difference in encoder values
    int deltaEncoder = currentEncoder - previousEncoder;

    // Calculate speed in ticks per second
    int speed = deltaEncoder * 1000 / deltaTime;

    // Convert speed to the range of -127 to 127
    int scaledSpeed = (speed * 127) / 2000;  // assuming 2000 is maxMotorSpeed

    return scaledSpeed;
}
