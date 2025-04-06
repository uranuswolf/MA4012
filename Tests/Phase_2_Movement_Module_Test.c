#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder)

#define PI 3.14159265359
#define WHEEL_DIAMETER 0.06985
#define WHEEL_BASE 0.188
#define TICKS_PER_REV 90
#define BASE_POWER 50
#define CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define DISTANCE_PER_TICK (CIRCUMFERENCE / TICKS_PER_REV)
#define DISTANCE_CORRECTION_FACTOR 3.5
#define OFFSET_POWER_FOR_LEFT_MOTOR 1.28

int distanceToTicks(float distance) {
    return (int)(distance / DISTANCE_PER_TICK);
}

void moveDistance(float distance, bool backward = false) {
    float realDistance = distance * DISTANCE_CORRECTION_FACTOR;
    int targetTicks = distanceToTicks(realDistance);

    int maxPowerRight = (backward ? -1 : 1) * BASE_POWER;
    int maxPowerLeft = (backward ? -1 : 1) * BASE_POWER * OFFSET_POWER_FOR_LEFT_MOTOR;

    int currentRightPower = 0;
    int currentLeftPower = 0;
    int accelRate = 10;

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    while (true) {
        int leftTicks = abs(SensorValue[LEFT_ENCODER]);
        int rightTicks = abs(SensorValue[RIGHT_ENCODER]);

        if (leftTicks >= targetTicks && rightTicks >= targetTicks) break;

        int remainingTicks = targetTicks - ((leftTicks > rightTicks) ? leftTicks : rightTicks);

        int powerRight = maxPowerRight;
        int powerLeft = maxPowerLeft;

        if (remainingTicks < targetTicks * 0.3) {
            powerRight = maxPowerRight * remainingTicks / (targetTicks * 0.3);
            powerLeft = maxPowerLeft * remainingTicks / (targetTicks * 0.3);
        }

        if (abs(currentRightPower) < abs(powerRight)) {
            currentRightPower += accelRate * (powerRight > 0 ? 1 : -1);
            if ((powerRight > 0 && currentRightPower > powerRight) ||
                (powerRight < 0 && currentRightPower < powerRight)) {
                currentRightPower = powerRight;
            }
        } else {
            if (abs(currentRightPower) > abs(powerRight)) {
                currentRightPower -= accelRate * (powerRight > 0 ? 1 : -1);
                if ((powerRight > 0 && currentRightPower < powerRight) ||
                    (powerRight < 0 && currentRightPower > powerRight)) {
                    currentRightPower = powerRight;
                }
            } else {
                currentRightPower = powerRight;
            }
        }

        if (abs(currentLeftPower) < abs(powerLeft)) {
            currentLeftPower += accelRate * (powerLeft > 0 ? 1 : -1);
            if ((powerLeft > 0 && currentLeftPower > powerLeft) ||
                (powerLeft < 0 && currentLeftPower < powerLeft)) {
                currentLeftPower = powerLeft;
            }
        } else {
            if (abs(currentLeftPower) > abs(powerLeft)) {
                currentLeftPower -= accelRate * (powerLeft > 0 ? 1 : -1);
                if ((powerLeft > 0 && currentLeftPower < powerLeft) ||
                    (powerLeft < 0 && currentLeftPower > powerLeft)) {
                    currentLeftPower = powerLeft;
                }
            } else {
                currentLeftPower = powerLeft;
            }
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

    motor[motorLeft] = right ? reducedSpeed : -reducedSpeed;
    motor[motorRight] = right ? -reducedSpeed : reducedSpeed;

    while(abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
          abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
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
    
    writeDebugStreamLine("Movement tests complete!");
}

task main() {
    testMovementModule();
}