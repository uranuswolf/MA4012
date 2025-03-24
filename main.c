#pragma config(Sensor, dgtl2,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  LEFT_ENCODER,   sensorQuadEncoder)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  motorRight,     tmotorVex393_MC29, openLoop)

// Base power to ensure movement
int basePower = 50;

// Wheel and encoder specifications
float wheelDiameter = 0.06985; // in meters
float wheelCircumference = wheelDiameter * 3.14159; // meters
int ticksPerRevolution = 90; // VEX encoder ticks per shaft revolution
float distancePerTick = wheelCircumference / ticksPerRevolution; // meters per tick
float wheelBase = 0.188; // Distance between the two wheels (meters)

// Convert desired distance (meters) to encoder ticks
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

// Move forward or backward using encoder counts
void moveDistance(float distance, bool backward = false) {
    int targetTicks = distanceToTicks(distance);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;
    
    int leftPower = basePower;
    int rightPower = basePower;
    if (backward) {
        leftPower = -basePower;
        rightPower = -basePower;
    }
    
    motor[motorLeft] = leftPower;
    motor[motorRight] = rightPower;
    
    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }
    
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Turn left or right using encoder counts
void turnDegrees(float degrees, bool right = false) {
    float arcLength = (degrees / 360) * 2 * 3.14159 * (wheelBase / 2);
    int targetTicks = distanceToTicks(arcLength);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;
    
    int leftPower = basePower;
    int rightPower = -basePower;
    if (right) {
        leftPower = -basePower;
        rightPower = basePower;
    }
    
    motor[motorLeft] = leftPower;
    motor[motorRight] = rightPower;
    
    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }
    
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

task main() {
    moveDistance(1.0); // Move forward 1.0 meters
    moveDistance(1.0, true); // Move backward 1.0 meters
    turnDegrees(90); // Turn left 90 degrees
    turnDegrees(90, true); // Turn right 90 degrees
}
