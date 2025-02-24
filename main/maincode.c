#include "MotorControl.h"
#include "PIDControl.h"

// Example motor encoder sensor definitions
#define encoderL1 SensorValue[encoderL1]
#define encoderR1 SensorValue[encoderR1]

task main() {
    int setpoint = 127;  // Desired motor speed (range -127 to 127)
    int previousEncoderL = 0, previousEncoderR = 0;
    clearTimer(T1);

    while (true) {
        int deltaTime = time1[T1];  // Get elapsed time
        clearTimer(T1);  // Reset timer

        // Get current encoder values
        int currentEncoderL = encoderL1;
        int currentEncoderR = encoderR1;

        // Calculate speed (in encoder ticks per second)
        int speedL = getEncoderSpeed(previousEncoderL, currentEncoderL, deltaTime);
        int speedR = getEncoderSpeed(previousEncoderR, currentEncoderR, deltaTime);

        // Compute PID output for speed control
        int leftPower = pidControl(setpoint, speedL, maxMotorSpeed);
        int rightPower = pidControl(setpoint, speedR, maxMotorSpeed);

        // Apply motor power
        controlMotors(leftPower, rightPower);

        // Update previous encoder values
        previousEncoderL = currentEncoderL;
        previousEncoderR = currentEncoderR;

        wait1Msec(20);  // Run loop every 20ms
    }
}

