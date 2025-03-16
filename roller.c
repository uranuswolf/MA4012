#include "roller.h"

// Define motor ports for rollers
#define FRONT_ROLLER motor[port2]   // Vex Continuous Rotation Motor (276-2163) , **change port number accordingly**
#define BACK_ROLLER motor[port3]    // Vex Continuous Rotation Motor (276-2163), **change port number accordingly**

void FrontRollerIntakeBall(int speed) { // **check speed sign for intake of ball**
    FRONT_ROLLER = -speed; //**from ACW from LSV
}

void FrontRollerOutputBall(int speed) { // **check speed sign for outtake of ball**
    FRONT_ROLLER = -speed; //**from CW from LSV
}

void stopFrontRoller() {
    FRONT_ROLLER = 0;
}

void activateBackRoller(int speed) {
    BACK_ROLLER = speed;
}

void stopBackRoller() {
    BACK_ROLLER = 0;
}

