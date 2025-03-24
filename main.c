#pragma config(Sensor, dgtl2,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  LEFT_ENCODER,   sensorQuadEncoder)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  motorRight,     tmotorVex393_MC29, openLoop)

// Base motor power for movement
int basePower = 50;

// Wheel and encoder specs
float wheelDiameter = 0.06985; // meters
float wheelCircumference = wheelDiameter * 3.14159; // meters
int ticksPerRevolution = 90; // encoder ticks per revolution
float distancePerTick = wheelCircumference / ticksPerRevolution; // meters per tick
float wheelBase = 0.188; // distance between wheels (meters)

// Convert desired distance in meters to encoder ticks
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

// Move forward or backward by distance (in meters)
void moveDistance(float distance, bool backward = false) {
    int targetTicks = distanceToTicks(distance);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = basePower;
        motor[motorRight] = -basePower;
    } else {
        motor[motorLeft] = -basePower;
        motor[motorRight] = basePower;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Turn left or right by angle (in degrees)
void turnDegrees(float degrees, bool right = false) {
    float turningCircumference = 3.14159 * wheelBase;  // Full turning circle
    float arcLength = (degrees / 360.0) * turningCircumference; // One wheel arc distance
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (right) {
        motor[motorLeft] = -basePower;
        motor[motorRight] = -basePower;
    } else {
        motor[motorLeft] = basePower;
        motor[motorRight] = basePower;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

task main() {
    moveDistance(10); // Move forward 1 meter
    wait1Msec(1000);
    moveDistance(10, true); // Move backward 1 meter
    wait1Msec(1000);
    turnDegrees(90);         // Turn left 90 degrees
    wait1Msec(1000);
    turnDegrees(180, true);   // Turn right 90 degrees
}
