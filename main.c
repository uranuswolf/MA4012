#pragma config(Sensor, dgtl2,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  LEFT_ENCODER,   sensorQuadEncoder)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  motorRight,     tmotorVex393_MC29, openLoop)

// Wheel and turning specifications
float wheelBase = 0.3; // Distance between left and right wheels in meters
float wheelDiameter = 0.06985;
float wheelCircumference = wheelDiameter * 3.14159;
int ticksPerRevolution = 90;
float distancePerTick = wheelCircumference / ticksPerRevolution;

// Convert degrees of rotation to encoder ticks
int degreesToTicks(float degrees) {
    float turnCircumference = 3.14159 * wheelBase; // Turning circle circumference
    float turnDistance = (degrees / 360.0) * turnCircumference;
    return distanceToTicks(turnDistance);
}

// Function to turn left
void turnLeft(float degrees) {
    int targetTicks = degreesToTicks(degrees);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        motor[motorLeft] = 50;   // Left motor moves backward
        motor[motorRight] = 50;  // Right motor moves forward
    }
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Function to turn right
void turnRight(float degrees) {
    int targetTicks = degreesToTicks(degrees);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        motor[motorLeft] = -50;  // Left motor moves forward
        motor[motorRight] = -50; // Right motor moves backward
    }
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Function to turn 180 degrees (turn backwards)
void turnBackwards() {
    turnLeft(180);  // Rotate in place by 180 degrees
}

task main() {
    moveForwardPID(1.5); // Move forward 1.5 meters
    turnLeft(90);        // Turn left 90 degrees
    moveForwardPID(1.0); // Move forward 1 meter
    turnRight(90);       // Turn right 90 degrees
    moveForwardPID(0.5); // Move forward 0.5 meters
    turnBackwards();     // Turn backwards
}
