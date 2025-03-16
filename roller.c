#include "roller.h"

// Define motor ports for rollers
#define FRONT_ROLLER motor[port2]   // Continuous Rotation Motor, **change port number accordingly**
#define BACK_ROLLER motor[port3]    // Ball release motor, **change port number accordingly**

void activateFrontRoller(int speed) {
    FRONT_ROLLER = speed;
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

