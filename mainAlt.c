#pragma config(Sensor, dgtl2,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  LEFT_ENCODER,   sensorQuadEncoder)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  motorRight,     tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard !!*//

// PID constants
float kP = 0.5;
float kI = 0.01;
float kD = 0.1;

// Base power to ensure movement
int basePower = 50;

// Wheel and encoder specifications
float wheelDiameter = 0.06985; // in meters
float wheelCircumference = wheelDiameter * 3.14159; // meters
int ticksPerRevolution = 90; // VEX encoder ticks per shaft revolution
float distancePerTick = wheelCircumference / ticksPerRevolution; // meters per tick = 0.0024387
float wheelBase = 0.3; // Distance between the two wheels (meters)

// Convert desired distance (meters) to encoder ticks
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

// PID control function
void pidControl(int targetTicks, int &leftPower, int &rightPower, bool Backward, bool turn = false) {
    int leftError, rightError;
    int leftIntegral = 0, rightIntegral = 0;
    int leftDerivative, rightDerivative;
    int lastLeftError = 0, lastRightError = 0;

    while (true) {
        int leftPos = abs(SensorValue[LEFT_ENCODER]);
        int rightPos = abs(SensorValue[RIGHT_ENCODER]);

        // Compute errors
        leftError = targetTicks - leftPos;
        rightError = targetTicks - rightPos;

        // Stop condition: If both wheels have reached the target
        if (abs(leftError) < 5 && abs(rightError) < 5) {
            break;
        }

        // Compute integral (with windup prevention)
        leftIntegral += leftError;
        rightIntegral += rightError;
        if (abs(leftIntegral) > 2000) {
            leftIntegral = 2000 * (leftIntegral / abs(leftIntegral));
        }
        
        if (abs(rightIntegral) > 2000) {
            rightIntegral = 2000 * (rightIntegral / abs(rightIntegral));
        }

        // Compute derivative
        leftDerivative = leftError - lastLeftError;
        rightDerivative = rightError - lastRightError;

        lastLeftError = leftError;
        lastRightError = rightError;

        // Compute PID outputs
        int leftPID = (kP * leftError) + (kI * leftIntegral) + (kD * leftDerivative);
        int rightPID = (kP * rightError) + (kI * rightIntegral) + (kD * rightDerivative);

        // Apply base power + PID correction
        leftPower = -(basePower + leftPID);  // Reverse left motor due to mirror image
        rightPower = basePower + rightPID;   // Right motor remains as is

        // Ensure motor power stays within limits (-127 to 127)
        leftPower = (leftPower > 127) ? 127 : (leftPower < -127) ? -127 : leftPower;
        rightPower = (rightPower > 127) ? 127 : (rightPower < -127) ? -127 : rightPower;

        // Reverse motor power if moving backwards
        if (Backward) {
            leftPower = -leftPower;
            rightPower = -rightPower;
        }

        // Adjust for turning: flip right power if turning right
        if (turn) {
            rightPower = -rightPower;  // For left turn, keep left as negative and right positive
        }

        // Set motor power
        motor[motorLeft] = leftPower;
        motor[motorRight] = rightPower;

        wait1Msec(20); // Short delay for stability
    }

    // Stop motors
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Move PID function
void movePID(float distance, bool Backward = false) {
    int targetTicks = distanceToTicks(distance);
    // Reset encoders
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int leftPower, rightPower;

    // Call the modular PID control function for movement
    pidControl(targetTicks, leftPower, rightPower, Backward);
}

// Turn PID function
void turnPID(float degrees, bool Right = false) {
    float arcLength = (degrees / 360) * 2 * 3.14159 * (wheelBase / 2);
    int targetTicks = distanceToTicks(arcLength);
    // Reset encoders
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int leftPower, rightPower;

    // Call the modular PID control function for turning
    pidControl(targetTicks, leftPower, rightPower, false, true);
}

task main() {
    movePID(1.0); // Move forward 1.0 meters
    movePID(1.0, true); // Move backward 1.0 meters
    turnPID(90); // Turn left 90 degrees
    turnPID(90, true); // Turn right 90 degrees
}
