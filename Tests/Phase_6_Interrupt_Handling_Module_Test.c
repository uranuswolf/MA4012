#pragma config(Sensor, in3,    IR_A,           sensorAnalog)
#pragma config(Sensor, in4,    IR_B,           sensorAnalog)
#pragma config(Sensor, in5,    IR_C,           sensorAnalog)
#pragma config(Sensor, in6,    IR_D,           sensorAnalog)
#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)

#define BASE_POWER 50
#define OFFSET_POWER_FOR_LEFT_MOTOR 1.28

bool boundaryInterruptFlag = false;
bool boundaryInterruptActive = false;
bool obstacleInterruptFlag = false;
bool obstacleInterruptActive = false;
int lastBoundaryTurn = -1;

int IR_values[4];
float distFC, distBC;

void moveDistance(float distance, bool backward = false) {
    motor[motorLeft] = backward ? BASE_POWER * OFFSET_POWER_FOR_LEFT_MOTOR : -BASE_POWER * OFFSET_POWER_FOR_LEFT_MOTOR;
    motor[motorRight] = backward ? -BASE_POWER : BASE_POWER;
    wait1Msec(distance * 1000);
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void turnDegrees(float degrees, bool right) {
    motor[motorLeft] = right ? 40 : -40;
    motor[motorRight] = right ? -40 : 40;
    wait1Msec(degrees * 10);
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void boundaryInterrupt() {
    if (!boundaryInterruptActive) {
        boundaryInterruptFlag = true;
        boundaryInterruptActive = true;
        writeDebugStreamLine("BOUNDARY INTERRUPT TRIGGERED");
    }
}

void obstacleInterrupt() {
    if (!obstacleInterruptActive && !boundaryInterruptActive) {
        obstacleInterruptFlag = true;
        obstacleInterruptActive = true;
        writeDebugStreamLine("OBSTACLE INTERRUPT TRIGGERED");
    }
}

void handleBoundary() {
    writeDebugStreamLine("Handling boundary");
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    
    bool frontLeft = (IR_values[1] == 0);
    bool frontRight = (IR_values[0] == 0);
    bool backLeft = (IR_values[3] == 0);
    bool backRight = (IR_values[2] == 0);

    int thisTurn = -1;

    if (frontLeft || frontRight) {
        moveDistance(-0.25, true);

        if (frontLeft && !frontRight) {
            thisTurn = 1;
        } else if (frontRight && !frontLeft) {
            thisTurn = 0;
        } else {
            thisTurn = rand() % 2;
        }

        if (thisTurn == lastBoundaryTurn) {
            thisTurn = 1 - thisTurn;
        }

        lastBoundaryTurn = thisTurn;
        turnDegrees(120, thisTurn == 1);
        moveDistance(0.15);
    } 
    else {
        moveDistance(0.25);

        if (backLeft && !backRight) {
            thisTurn = 1;
        } else if (backRight && !backLeft) {
            thisTurn = 0;
        } else {
            thisTurn = rand() % 2;
        }

        if (thisTurn == lastBoundaryTurn) {
            thisTurn = 1 - thisTurn;
        }

        lastBoundaryTurn = thisTurn;
        turnDegrees(90, thisTurn == 1);
        moveDistance(0.15);
    }

    boundaryInterruptFlag = false;
    boundaryInterruptActive = false;
}

void handleObstacle() {
    writeDebugStreamLine("Handling obstacle");
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    
    float frontDist = distFC;
    float backDist = distBC;

    if (frontDist < 40.0) {
        moveDistance(-0.2, true);
        turnDegrees(45, rand() % 2);
    } else if (backDist < 40.0) {
        moveDistance(0.2);
        turnDegrees(45, rand() % 2);
    }

    obstacleInterruptFlag = false;
    obstacleInterruptActive = false;
}

void testInterruptHandling() {
    // Simulate normal operation
    writeDebugStreamLine("Starting normal movement");
    moveDistance(0.5);
    
    // Test boundary interrupt
    writeDebugStreamLine("\nTESTING BOUNDARY INTERRUPT");
    IR_values[0] = 0; // Trigger front right boundary
    boundaryInterrupt();
    
    if (boundaryInterruptFlag) {
        handleBoundary();
    }
    wait1Msec(2000);
    
    // Test obstacle interrupt
    writeDebugStreamLine("\nTESTING OBSTACLE INTERRUPT");
    distFC = 30.0; // Trigger front obstacle
    obstacleInterrupt();
    
    if (obstacleInterruptFlag) {
        handleObstacle();
    }
    wait1Msec(2000);
    
    // Test interrupt priority
    writeDebugStreamLine("\nTESTING INTERRUPT PRIORITY");
    IR_values[1] = 0; // Trigger front left boundary
    distBC = 30.0;    // Also trigger back obstacle
    boundaryInterrupt();
    obstacleInterrupt();
    
    if (boundaryInterruptFlag) {
        writeDebugStreamLine("Boundary should take priority");
        handleBoundary();
    } else if (obstacleInterruptFlag) {
        handleObstacle();
    }
    
    writeDebugStreamLine("\nInterrupt tests complete!");
}

task main() {
    testInterruptHandling();
}