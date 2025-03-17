#include "encoder.h"
#include <math.h>  

#define max_Ticks_per_sec 1450 
//run encoderTest.c to find the max ticks per second for max speed power 127 experimentally,
//Best to do when the the whole system is ready 

// Reset encoders
void resetEncoders() {
    SensorValue[dgtl1] = 0; // **change port number accordingly**
    SensorValue[dgtl3] = 0; // **change port number accordingly**
}

// Calculate left wheel speed 
int getLeftEncoderSpeed() {
    int initialTicks = LEFT_ENCODER;  // Read initial encoder value
    wait1Msec(10);                    // Wait for 10ms
    int finalTicks = LEFT_ENCODER;    // Read new encoder value
    int currentTicks_per_sec = (finalTicks - initialTicks) * 100;  // Convert to ticks/sec
    return (int) round(((double)currentTicks_per_sec * 127) / max_Ticks_per_sec);
}

// Calculate right wheel speed 
int getRightEncoderSpeed() {
    int initialTicks = RIGHT_ENCODER;  // Read initial encoder value
    wait1Msec(10);                    // Wait for 10ms
    int finalTicks = RIGHT_ENCODER;    // Read new encoder value
    int currentTicks_per_sec = (finalTicks - initialTicks) * 100;  // Convert to ticks/sec
    return (int) round(((double)currentTicks_per_sec * 127) / max_Ticks_per_sec);
}



