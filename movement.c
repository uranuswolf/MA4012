#include "movement.h"
#include "encoder.h"  
#include "pid.h"      


// Define motor ports for movement
// Update port numbers as needed
#define LEFT_MOTOR motor[port4]                  
#define RIGHT_MOTOR motor[port5]                 

//**check which motor should have the negative sign** 
void moveForward(int speed) {                   
    int desiredSpeed = speed;     
    resetPID();             
    resetEncoders();
    while(true){
    LEFT_MOTOR = computePID(desiredSpeed, getLeftEncoderSpeed()); //CW from the LSV
    RIGHT_MOTOR = -computePID(desiredSpeed, getRightEncoderSpeed()); //CW from the LSV
    }
}

void moveBackward(int speed) {
    int desiredSpeed = speed;      
    resetPID();               
    resetEncoders();
    while(true){
        LEFT_MOTOR = -computePID(desiredSpeed, getLeftEncoderSpeed()); //ACW from the LSV
        RIGHT_MOTOR = computePID(desiredSpeed, getRightEncoderSpeed()); //ACW from the LSV
        }
}

//rotate left in place
void turnLeft(int speed) { 
    int desiredSpeed = speed;      
    resetPID();               
    resetEncoders();
    while(true){
        LEFT_MOTOR = -computePID(desiredSpeed, getLeftEncoderSpeed()); //ACW from the LSV
        RIGHT_MOTOR = -computePID(desiredSpeed, getRightEncoderSpeed()); //CW from the LSV
        }
}

 //rotate right in place
void turnRight(int speed) { 
    int desiredSpeed = speed;      
    resetPID();               
    resetEncoders();
    while(true){
        LEFT_MOTOR = computePID(desiredSpeed, getLeftEncoderSpeed()); //CW from the LSV
        RIGHT_MOTOR = computePID(desiredSpeed, getRightEncoderSpeed()); //ACW from the LSV
        }
}

void stopMotors() {
    LEFT_MOTOR = 0;
    RIGHT_MOTOR = 0;
}
