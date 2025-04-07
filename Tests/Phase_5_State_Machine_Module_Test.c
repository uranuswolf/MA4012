#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)
#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, dgtl7,  limitswitchBall,sensorDigitalIn)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorServoContinuousRotation, openLoop)

#define ROLLER_SPEED 127
#define OFFSET_POWER_FOR_LEFT_MOTOR 1.28 // Adjust this factor based on your robot's calibration

typedef enum RobotState {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
} RobotState;

typedef enum FlapMode {
    PUSH, 
    OPEN,
    CLOSE 
} FlapMode;

typedef enum RollerMode {
    INTAKE, 
    OUTPUT,
    STOP 
} RollerMode;

typedef struct {
    bool isBall;
    bool isBallPicked;
    bool reachedBase;
    bool isfirstBallDelivered;
} StatusFlags;

RobotState currentState = SEARCH;
StatusFlags status;

void frontRollerControl(RollerMode mode) {
    switch(mode){
        case INTAKE:
            motor[FRONT_ROLLER] = -ROLLER_SPEED; // Intake speed
            break;
        case OUTPUT:
            motor[FRONT_ROLLER] = ROLLER_SPEED; // Output speed
            break;
        case STOP:
            motor[FRONT_ROLLER] = 0; // Stop roller
            break;
    }
}

void flapperControl(FlapMode mode) {
    switch (mode) {
        case PUSH:
            // Implement PUSH behavior
            motor[BACK_ROLLER] = -50; // Push the ball
            wait1Msec(800);
            break;

        case OPEN:
            // Implement OPEN behavior
            motor[BACK_ROLLER] = 50;
            wait1Msec(1500);
            motor[BACK_ROLLER] = 0;
            break;

        case CLOSE:
            // Implement CLOSE behavior
            motor[BACK_ROLLER] = -50;
            wait1Msec(300);
            motor[BACK_ROLLER] = 0;
            break;
    }
}

void moveDistance(float distance, bool backward = false) {
    // Simplified for testing
    motor[motorLeft] = backward ? 50 : -50 * OFFSET_POWER_FOR_LEFT_MOTOR;
    motor[motorRight] = backward ? -50 : 50;
    wait1Msec(distance * 1000);
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}


void searchPhase() {
    writeDebugStreamLine("SEARCH PHASE");
    status.isBall = true; // Simulate finding ball
    currentState = COLLECT;
}

void collectPhase() {
    writeDebugStreamLine("COLLECT PHASE");
    frontRollerControl(INTAKE);
    moveDistance(0.2);
    
    // Simulate ball pickup
    if(SensorValue[limitswitchBall] == 0) {
        status.isBallPicked = true;
        frontRollerControl(STOP);
        flapperControl(CLOSE);
        currentState = RETURN;
    }
}

void returnPhase() {
    writeDebugStreamLine("RETURN PHASE");
    moveDistance(0.5, true);
    status.reachedBase = true;
    currentState = DELIVER;
}

void deliverPhase() {
    writeDebugStreamLine("DELIVER PHASE");
    flapperControl(PUSH);
    wait1Msec(1000);
    flapperControl(OPEN);
    
    status.isBallPicked = false;
    currentState = SEARCH;
    if (!status.isfirstBallDelivered) {
        status.isfirstBallDelivered = true;
    }
}

void testStateMachine() {
    // Manual override for testing
    SensorValue[limitswitchBall] = 1; // Start with no ball
    
    while(true) {
        writeDebugStreamLine("\nCurrent State: %d", currentState);
        
        switch(currentState) {
            case SEARCH:
                searchPhase();
                break;
            case COLLECT:
                collectPhase();
                break;
            case RETURN:
                returnPhase();
                break;
            case DELIVER:
                deliverPhase();
                break;
        }
        
        // Simulate ball pickup in collect phase
        if(currentState == COLLECT) {
            wait1Msec(1500);
            SensorValue[limitswitchBall] = 0;
        }
        
        // Simulate ball drop in deliver phase
        if(currentState == DELIVER) {
            wait1Msec(1500);
            SensorValue[limitswitchBall] = 1;
        }
        
        wait1Msec(1000);
    }
}

task main() {
    testStateMachine();
}