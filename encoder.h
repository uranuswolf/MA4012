#ifndef ENCODER_H
#define ENCODER_H

// Define encoder ports (Update port numbers as needed)
#define LEFT_ENCODER SensorValue[dgtl1] 
#define RIGHT_ENCODER SensorValue[dgtl3] 

// Function Prototypes
void resetEncoders();
int getLeftEncoderSpeed();
int getRightEncoderSpeed();

#endif
