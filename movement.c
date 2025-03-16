#include "movement.h"

// Define motor ports for movement
#define LEFT_MOTOR motor[port4]  //**Change port number accordingly**
#define RIGHT_MOTOR motor[port5] //**Change port number accordingly**

void moveForward(int speed) {
    LEFT_MOTOR = speed;
    RIGHT_MOTOR = speed;
}

void moveBackward(int speed) {
    LEFT_MOTOR = -speed;
    RIGHT_MOTOR = -speed;
}

void turnLeft(int speed) {
    LEFT_MOTOR = -speed;
    RIGHT_MOTOR = speed;
}

void turnRight(int speed) {
    LEFT_MOTOR = speed;
    RIGHT_MOTOR = -speed;
}

void stopMotors() {
    LEFT_MOTOR = 0;
    RIGHT_MOTOR = 0;
}
