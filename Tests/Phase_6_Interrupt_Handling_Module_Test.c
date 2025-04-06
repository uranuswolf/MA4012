#pragma config(Sensor, in3,    IR_A,           sensorAnalog)
#pragma config(Sensor, in4,    IR_B,           sensorAnalog)
#pragma config(Sensor, in5,    IR_C,           sensorAnalog)
#pragma config(Sensor, in6,    IR_D,           sensorAnalog)
#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)

#define BASE_POWER 50

bool boundaryInterruptFlag = false;
bool boundaryInterruptActive = false;
bool obstacleInterruptFlag = false;
bool obstacleInterruptActive = false;
int lastBoundaryTurn = -1;

int IR_values[4]; // 0:FR, 1:FL, 2:BR, 3:BL
float distFC, distBC;

void stopMotors() {
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void moveDistance(float distance) {
    motor[motorLeft] = -BASE_POWER;
    motor[motorRight] = BASE_POWER;
    wait1Msec(distance * 1000);
    stopMotors();
}

void turnDegrees(float degrees, bool right) {
    motor[motorLeft] = right ? -50 : 50;
    motor[motorRight] = right ? -50 : 50;
    wait1Msec(degrees * 10);
    stopMotors();
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
    stopMotors();
    
    bool frontLeft = (IR_values[1] == 0);
    bool frontRight = (IR_values[0] == 0);
    
    int thisTurn = -1;
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
    moveDistance(-0.25);
    turnDegrees(120, thisTurn == 1);
    moveDistance(0.15);
    
    boundaryInterruptFlag = false;
    boundaryInterruptActive = false;
}

void handleObstacle() {
    writeDebugStreamLine("Handling obstacle");
    stopMotors();
    
    if (distFC < 40.0) {
        moveDistance(-0.2);
        turnDegrees(45, rand() % 2);
    } else if (distBC < 40.0) {
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