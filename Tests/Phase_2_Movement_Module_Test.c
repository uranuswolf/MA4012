#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder)

#define PI 3.14159265359
#define WHEEL_DIAMETER 0.06985 // meters
#define WHEEL_BASE 0.188 // meters
#define TICKS_PER_REV 90
#define BASE_POWER 50
#define CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define DISTANCE_PER_TICK (CIRCUMFERENCE / TICKS_PER_REV)

int distanceToTicks(float distance) {
    return (int)(distance / DISTANCE_PER_TICK);
}

void moveDistance(float distance, bool backward = false) {
    float realDistance = distance * 3.5;
    int targetTicks = distanceToTicks(realDistance);
    
    int leftPower = backward ? 1.28 * BASE_POWER : -1.28 * BASE_POWER;
    int rightPower = backward ? -BASE_POWER : BASE_POWER;

    int accelRate = 10;
    int maxPowerLeft = leftPower;
    int maxPowerRight = rightPower;
    int currentLeftPower = 0;
    int currentRightPower = 0;

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    while(abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
          abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        if (currentLeftPower < maxPowerLeft) {
            currentLeftPower += accelRate;
            if (currentLeftPower > maxPowerLeft) currentLeftPower = maxPowerLeft;
        }
        if (currentRightPower < maxPowerRight) {
            currentRightPower += accelRate;
            if (currentRightPower > maxPowerRight) currentRightPower = maxPowerRight;
        }

        motor[motorLeft] = currentLeftPower;
        motor[motorRight] = currentRightPower;
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void turnDegrees(float degrees, bool right = false) {
    float realDegrees = degrees * 2.5;
    float arcLength = (realDegrees / 180.0) * (PI * WHEEL_BASE);
    int targetTicks = distanceToTicks(arcLength);
    int reducedSpeed = 127 * 0.7;

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    motor[motorLeft] = right ? -reducedSpeed : reducedSpeed;
    motor[motorRight] = right ? -reducedSpeed : reducedSpeed;

    while(abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
          abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void stopMotors() {
    int decelRate = 10;
    int leftPower = motor[motorLeft];
    int rightPower = motor[motorRight];

    while (leftPower != 0 || rightPower != 0) {
        if (leftPower > 0) leftPower -= decelRate;
        else if (leftPower < 0) leftPower += decelRate;
        if (rightPower > 0) rightPower -= decelRate;
        else if (rightPower < 0) rightPower += decelRate;

        motor[motorLeft] = leftPower;
        motor[motorRight] = rightPower;
        wait1Msec(50);
    }
}

void testMovementModule() {
    // Forward movement test
    writeDebugStreamLine("Testing forward movement (0.5m)");
    moveDistance(0.5);
    wait1Msec(2000);
    
    // Backward movement test
    writeDebugStreamLine("Testing backward movement (0.3m)");
    moveDistance(0.3, true);
    wait1Msec(2000);
    
    // Left turn test
    writeDebugStreamLine("Testing left turn (90 degrees)");
    turnDegrees(90);
    wait1Msec(2000);
    
    // Right turn test
    writeDebugStreamLine("Testing right turn (90 degrees)");
    turnDegrees(90, true);
    wait1Msec(2000);
    
    // Stop test
    writeDebugStreamLine("Testing motor stop");
    motor[motorLeft] = 50;
    motor[motorRight] = -50;
    wait1Msec(500);
    stopMotors();
    wait1Msec(2000);
    
    writeDebugStreamLine("Movement tests complete!");
}

task main() {
    testMovementModule();
}