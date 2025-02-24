#include "motorControl.h"  // Include the header file for motor control

// PID coefficients and variables
float Kp = 0.5;
float Ki = 0.1;
float Kd = 0.05;
float previousError = 0;
float integral = 0;

task main()
{
    // Desired position (setpoint) for the encoders
    int setpoint = 1000;  // For example, set the target encoder value to 1000

    // Reset encoders
    SensorValue[encoderR1] = 0;
    SensorValue[encoderL1] = 0;

    // Run motors based on PID control
    while(true)
    {
        // Get the current encoder values
        int currentLeft = SensorValue[encoderL1];
        int currentRight = SensorValue[encoderR1];

        // Calculate the PID control output for both motors
        float leftOutput = pidControl(setpoint, currentLeft);
        float rightOutput = pidControl(setpoint, currentRight);

        // Use the PID output to set motor speeds
        controlMotors(leftOutput, rightOutput);

        // Display encoder values and PID outputs on the debug stream
        writeDebugStreamLine("Left Encoder: %d, Left Output: %f", currentLeft, leftOutput);
        writeDebugStreamLine("Right Encoder: %d, Right Output: %f", currentRight, rightOutput);

        wait1Msec(20);  // Small delay to prevent overloading the CPU
    }
}

