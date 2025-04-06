#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)
#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11, compass_MSB,    sensorDigitalIn)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)

#define PI 3.14159265359
#define WHEEL_DIAMETER 0.06985
#define WHEEL_BASE 0.188
#define TICKS_PER_REV 90
#define BASE_POWER 50
#define CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define DISTANCE_PER_TICK (CIRCUMFERENCE / TICKS_PER_REV)

typedef struct {
    float distFC;
    float distFR;
    float distFL;
    float distBC;
} DistanceSensors;

DistanceSensors distances;
int heading;

int distanceToTicks(float distance) {
    return (int)(distance /DISTANCE_PER_TICK);
}

void moveDistance(float distance, bool backward) {
    float realDistance = distance * 3.5;
    int targetTicks = distanceToTicks(realDistance);
    
    // Set the base power and calculate the final target powers for left and right motors
    int leftPower = backward ? 1.28 * BASE_POWER : -1.28 * BASE_POWER;
    int rightPower = backward ? -BASE_POWER : BASE_POWER;

    // Calculate the acceleration rate (you can adjust this value for smoother or faster acceleration)
    int accelRate = 10; // Increase the power by this amount every cycle (can be adjusted)
    int maxPowerLeft = leftPower;
    int maxPowerRight = rightPower;

    // Initialize motor power values at 0 for acceleration
    int currentLeftPower = 0;
    int currentRightPower = 0;

    // Reset encoders
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    // Start moving with gradual acceleration
    while(abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
          abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {

        // Gradually increase the power of the motors
        if (currentLeftPower < maxPowerLeft) {
            currentLeftPower += accelRate; // Increase left motor power
            if (currentLeftPower > maxPowerLeft) {
                currentLeftPower = maxPowerLeft; // Cap it at max power
            }
        }

        if (currentRightPower < maxPowerRight) {
            currentRightPower += accelRate; // Increase right motor power
            if (currentRightPower > maxPowerRight) {
                currentRightPower = maxPowerRight; // Cap it at max power
            }
        }

        // Set the motor power to the current power
        motor[motorLeft] = currentLeftPower;
        motor[motorRight] = currentRightPower;

        wait1Msec(10); // Small delay for smoother control
    }

    // Stop the motors once the target distance is reached
    stopMotors();
}

// Smoothly decelerates motors to a stop
void stopMotors() {
    // Start by slowly reducing the motor power
    int decelRate = 10;  // The rate at which the motor power will decrease
    int leftPower = motor[motorLeft];  // Get the current power of the left motor
    int rightPower = motor[motorRight];  // Get the current power of the right motor

    // Gradually decrease the motor power until both are stopped
    while (leftPower != 0 || rightPower != 0) {
        if (leftPower > 0) {
            leftPower -= decelRate;  // Gradually reduce power for left motor
            if (leftPower < 0) leftPower = 0;  // Don't go below 0
        } else if (leftPower < 0) {
            leftPower += decelRate;  // Gradually reduce power for left motor (if negative)
            if (leftPower > 0) leftPower = 0;  // Don't go above 0
        }

        if (rightPower > 0) {
            rightPower -= decelRate;  // Gradually reduce power for right motor
            if (rightPower < 0) rightPower = 0;  // Don't go below 0
        } else if (rightPower < 0) {
            rightPower += decelRate;  // Gradually reduce power for right motor (if negative)
            if (rightPower > 0) rightPower = 0;  // Don't go above 0
        }

        // Set the new motor power values
        motor[motorLeft] = leftPower;
        motor[motorRight] = rightPower;

        wait1Msec(50);  // Allow time for deceleration, adjust the time for smoother/longer deceleration
    }

    // Ensure motors are completely stopped
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void turnDegrees(float degrees, bool right) {
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
    stopMotors();
}

void searchingAlgo() {
    static int searchIteration = 0;
    const float PAN_ANGLE = 60.0;
    
    // Simple search pattern for testing
    writeDebugStreamLine("Searching algorithm iteration %d", searchIteration);
    
    if(searchIteration % 2 == 0) {
        turnDegrees(PAN_ANGLE);
    } else {
        turnDegrees(PAN_ANGLE, true);
    }
    
    moveDistance(0.3);
    searchIteration++;
}

void moveTowardsBall() {
    writeDebugStreamLine("Moving towards ball");
    
    if (distances.distFL < distances.distFR) {
        moveDistance(distances.distFL/100.0 - 0.2);
        turnDegrees(30);
    } else {
        moveDistance(distances.distFR/100.0 - 0.2);
        turnDegrees(30, true);
    }
}

void returnToBase() {
    writeDebugStreamLine("Returning to base");
    int target = 135;
    int degree = heading - target;
    
    if(degree < 0){
        turnDegrees(abs(degree), true);
    } else {
        turnDegrees(degree);
    }
    
    moveDistance(1.0, true);
}

task searchingBallTask() {
    while(true) {
        searchingAlgo();
        wait1Msec(100);
    }
}

task moveTowardsBallTask() {
    while(true) {
        moveTowardsBall();
        wait1Msec(100);
    }
}

task returnToBaseTask() {
    while(true) {
        returnToBase();
        wait1Msec(100);
    }
}

void testNavigationModule() {
    // Simulate sensor readings for testing
    distances.distFL = 40.0;
    distances.distFR = 60.0;
    distances.distFC = 80.0;
    heading = 180;
    
    // Test searching algorithm
    writeDebugStreamLine("=== TESTING SEARCHING ALGORITHM ===");
    startTask(searchingBallTask);
    wait1Msec(5000);
    stopTask(searchingBallTask);
    
    // Test ball approach
    writeDebugStreamLine("\n=== TESTING BALL APPROACH ===");
    distances.distFL = 30.0;
    distances.distFR = 50.0;
    startTask(moveTowardsBallTask);
    wait1Msec(5000);
    stopTask(moveTowardsBallTask);
    
    // Test return to base
    writeDebugStreamLine("\n=== TESTING RETURN TO BASE ===");
    heading = 90;
    startTask(returnToBaseTask);
    wait1Msec(5000);
    stopTask(returnToBaseTask);
    
    writeDebugStreamLine("\nNavigation tests complete!");
}

task main() {
    testNavigationModule();
}