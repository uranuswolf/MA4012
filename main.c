#pragma config(Sensor, dgtl2,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  LEFT_ENCODER,   sensorQuadEncoder)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  motorRight,     tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define max_Ticks_per_sec 1450
#define LEFT_MOTOR motor[motorLeft]
#define RIGHT_MOTOR motor[motorRight]

// PID variables
int error, prevError = 0;
int integral = 0;
int derivative = 0;

void resetEncoders() {
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;
}

int getLeftEncoderSpeed() {
    int initialTicks = SensorValue[LEFT_ENCODER];
    wait1Msec(10);
    int finalTicks = SensorValue[LEFT_ENCODER];
    int currentTicks_per_sec = (finalTicks - initialTicks) * 100;
    return (currentTicks_per_sec * 127) / max_Ticks_per_sec;
}

int getRightEncoderSpeed() {
    int initialTicks = SensorValue[RIGHT_ENCODER];
    wait1Msec(10);
    int finalTicks = SensorValue[RIGHT_ENCODER];
    int currentTicks_per_sec = (finalTicks - initialTicks) * 100;
    return (currentTicks_per_sec * 127) / max_Ticks_per_sec;
}

int computePID(int desiredSpeed, int actualSpeed) {
    error = desiredSpeed - actualSpeed;
    integral += error;
    derivative = error - prevError;
    prevError = error;
    
    int output = (0.5 * error) + (0.01 * integral) + (0.1 * derivative);
    if (output > 127) output = 127;
    if (output < -127) output = -127;
    
    return output;
}

void resetPID() {
    error = 0;
    prevError = 0;
    integral = 0;
    derivative = 0;
}

void moveForward(int speed) {
    resetPID();
    resetEncoders();
    int startTime = nSysTime;
    while (nSysTime - startTime < duration) {
        LEFT_MOTOR = computePID(speed, getLeftEncoderSpeed());
        RIGHT_MOTOR = -computePID(speed, getRightEncoderSpeed());
    }
    stopMotors();
}

void moveBackward(int speed) {
    resetPID();
    resetEncoders();
    int startTime = nSysTime;
    while (nSysTime - startTime < duration) {
        LEFT_MOTOR = -computePID(speed, getLeftEncoderSpeed());
        RIGHT_MOTOR = computePID(speed, getRightEncoderSpeed());
    }
    stopMotors();
}

void turnLeft(int speed) {
    resetPID();
    resetEncoders();
    int startTime = nSysTime;
    while (nSysTime - startTime < duration) {
        LEFT_MOTOR = -computePID(speed, getLeftEncoderSpeed());
        RIGHT_MOTOR = -computePID(speed, getRightEncoderSpeed());
    }
    stopMotors();
}

void turnRight(int speed) {
    resetPID();
    resetEncoders();
    int startTime = nSysTime;
    while (nSysTime - startTime < duration) {
        LEFT_MOTOR = computePID(speed, getLeftEncoderSpeed());
        RIGHT_MOTOR = computePID(speed, getRightEncoderSpeed());
    }
    stopMotors();
}

void stopMotors() {
    LEFT_MOTOR = 0;
    RIGHT_MOTOR = 0;
}

task main() {
    moveForward(80, 1000); // Move forward at speed 80 for 2 seconds
    wait1Msec(500); // Small delay
    turnLeft(60, 1000); // Turn left for 1 second
    wait1Msec(500);
    moveBackward(80, 1000); // Move backward at speed 80 for 2 seconds
    stopMotors();
    //wait1Msec(1000);
    //moveBackward(80);
    //wait1Msec(2000);
    //turnLeft(50);
    //wait1Msec(2000);
    //turnRight(50);
    //wait1Msec(2000);
    //stopMotors();
}
