#include "motorControl.h"  // Include the header file for motor control

task main()
{
    // Reset encoders
    SensorValue[encoderR1] = 0;
    SensorValue[encoderL1] = 0;
    
    // Start motors by calling the function from motors folder
    controlMotors(50, -50);  // Left motor speed 50, Right motor speed -50
    
    // Run for 3 seconds
    clearTimer(T1);
    while(time1[T1] < 3000) // Run for 3000ms (3 seconds)
    {
        // Display encoder values on the debug stream
        writeDebugStreamLine("Left Encoder: %d", SensorValue[encoderL1]);
        writeDebugStreamLine("Right Encoder: %d", SensorValue[encoderR1]);
        wait1Msec(100); // Small delay to reduce spam in debug stream
    }
    
    // Stop motors by calling the function again
    controlMotors(0, 0);  // Stop both motors
}
��
 
 
