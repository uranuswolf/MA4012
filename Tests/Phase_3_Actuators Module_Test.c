#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorServoContinuousRotation, openLoop)

#define ROLLER_SPEED 127

typedef enum RollerMode {
    INTAKE, 
    OUTPUT,
    STOP 
} RollerMode;

typedef enum FlapMode {
    PUSH, 
    OPEN,
    CLOSE 
} FlapMode;

void frontRollerControl(RollerMode mode) {
    switch(mode){
        case INTAKE:
            motor[FRONT_ROLLER] = -ROLLER_SPEED;
            break;
        case OUTPUT:
            motor[FRONT_ROLLER] = ROLLER_SPEED;
            break;
        case STOP:
            motor[FRONT_ROLLER] = 0;
            break;
    }
}

void flapperControl(FlapMode mode) {
    switch (mode) {
        case PUSH:
            motor[BACK_ROLLER] = -50;
            wait1Msec(800);
            motor[BACK_ROLLER] = 0;
            break;
        case OPEN:
            motor[BACK_ROLLER] = 50;
            wait1Msec(1500);
            motor[BACK_ROLLER] = 0;
            break;
        case CLOSE:
            motor[BACK_ROLLER] = -50;
            wait1Msec(300);
            motor[BACK_ROLLER] = 0;
            break;
    }
}

void testActuatorsModule() {
    writeDebugStreamLine("Testing front roller INTAKE");
    frontRollerControl(INTAKE);
    wait1Msec(2000);
    frontRollerControl(STOP);
    
    writeDebugStreamLine("Testing front roller OUTPUT");
    frontRollerControl(OUTPUT);
    wait1Msec(2000);
    frontRollerControl(STOP);
    
    writeDebugStreamLine("Testing flapper OPEN");
    flapperControl(OPEN);
    wait1Msec(2000);
    
    writeDebugStreamLine("Testing flapper CLOSE");
    flapperControl(CLOSE);
    wait1Msec(2000);
    
    writeDebugStreamLine("Testing flapper PUSH");
    flapperControl(PUSH);
    wait1Msec(2000);
    
    writeDebugStreamLine("Actuator tests complete!");
}

task main() {
    testActuatorsModule();
}