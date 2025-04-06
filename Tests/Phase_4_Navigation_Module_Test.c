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
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)

#define PI 3.14159265359
#define WHEEL_DIAMETER 0.06985
#define WHEEL_BASE 0.188
#define TICKS_PER_REV 90
#define BASE_POWER 50
#define CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define DISTANCE_PER_TICK (CIRCUMFERENCE / TICKS_PER_REV)
#define DISTANCE_CORRECTION_FACTOR 3.5
#define OFFSET_POWER_FOR_LEFT_MOTOR 1.28

typedef struct {
    float distFC;
    float distFR;
    float distFL;
    float distBC;
} DistanceSensors;

typedef struct {
    bool isfirstBallDelivered;
    bool panRight;
} StatusFlags;

DistanceSensors distances;
StatusFlags status;
int heading;

int distanceToTicks(float distance) {
    return (int)(distance /DISTANCE_PER_TICK);
}

void moveDistance(float distance, bool backward) {
    float realDistance = distance * DISTANCE_CORRECTION_FACTOR;
    int targetTicks = distanceToTicks(realDistance);

    int maxPowerRight = (backward ? -1 : 1) * BASE_POWER;
    int maxPowerLeft  = (backward ? -1 : 1) * BASE_POWER * OFFSET_POWER_FOR_LEFT_MOTOR;

    int currentRightPower = 0;
    int currentLeftPower = 0;
    int accelRate = 10;

    // Reset encoders
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    while (true) {
        int leftTicks  = abs(SensorValue[LEFT_ENCODER]);
        int rightTicks = abs(SensorValue[RIGHT_ENCODER]);

        if (leftTicks >= targetTicks && rightTicks >= targetTicks) break;

        int remainingTicks = targetTicks - ((leftTicks > rightTicks) ? leftTicks : rightTicks);

        // Choose whether to accelerate or decelerate
        int powerRight = maxPowerRight;
        int powerLeft  = maxPowerLeft;

        // If within 30% of target, start decelerating
        if (remainingTicks < targetTicks * 0.3) {
            powerRight = maxPowerRight * remainingTicks / (targetTicks * 0.3);
            powerLeft  = maxPowerLeft  * remainingTicks / (targetTicks * 0.3);
        }

        // Smooth acceleration
        if (abs(currentRightPower) < abs(powerRight)) {
            currentRightPower += accelRate * (powerRight > 0 ? 1 : -1);
            if ((powerRight > 0 && currentRightPower > powerRight) ||
                (powerRight < 0 && currentRightPower < powerRight)) {
                currentRightPower = powerRight;  // Cap to max power
            }
        } else {
            // Smooth deceleration path
            if (abs(currentRightPower) > abs(powerRight)) {
                currentRightPower -= accelRate * (powerRight > 0 ? 1 : -1);
                if ((powerRight > 0 && currentRightPower < powerRight) ||
                    (powerRight < 0 && currentRightPower > powerRight)) {
                    currentRightPower = powerRight;  // Smooth deceleration
                }
            } else {
                currentRightPower = powerRight;  // Final stop at target power
            }
        }

        if (abs(currentLeftPower) < abs(powerLeft)) {
            currentLeftPower += accelRate * (powerLeft > 0 ? 1 : -1);
            if ((powerLeft > 0 && currentLeftPower > powerLeft) ||
                (powerLeft < 0 && currentLeftPower < powerLeft)) {
                currentLeftPower = powerLeft;  // Cap to max power
            }
        } else {
            // Smooth deceleration path
            if (abs(currentLeftPower) > abs(powerLeft)) {
                currentLeftPower -= accelRate * (powerLeft > 0 ? 1 : -1);
                if ((powerLeft > 0 && currentLeftPower < powerLeft) ||
                    (powerLeft < 0 && currentLeftPower > powerLeft)) {
                    currentLeftPower = powerLeft;  // Smooth deceleration
                }
            } else {
                currentLeftPower = powerLeft;  // Final stop at target power
            }
        }

        motor[motorLeft] = currentLeftPower;
        motor[motorRight] = currentRightPower;

        wait1Msec(10);
    }

    // Full stop to cancel any lingering movement
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void turnDegrees(float degrees, bool right) {
    float realDegrees = degrees * 2.5;
    float arcLength = (realDegrees / 180.0) * (PI * WHEEL_BASE);
    int targetTicks = distanceToTicks(arcLength);
    int reducedSpeed = ROLLER_SPEED * 0.7;

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

void searchingAlgo() {
    static int searchIteration = 0;
    const float PAN_ANGLE = 20.0;
    const float INITIAL_DISTANCE = 1.0;
    const float ROW_DISTANCE = 0.2;

    if (!status.isfirstBallDelivered && searchIteration == 0) {
        // Initial search pattern
        moveDistance(INITIAL_DISTANCE);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, !status.panRight);
        wait1Msec(500);

        moveDistance(ROW_DISTANCE);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, !status.panRight);
        wait1Msec(500);
            
        moveDistance(ROW_DISTANCE);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, !status.panRight);
        wait1Msec(500);

        searchIteration++;
    }
    else {
        // Spiral search pattern
        const float SPIRAL_BASE = 0.4;
        const float SPIRAL_INC = 0.15;
        float spiralRadius = SPIRAL_BASE + (searchIteration * SPIRAL_INC);
        
        int patternVariant = rand() % 3;
        
        switch(patternVariant) {
            case 0: {
                // Spiral search
                const int MAX_SPIRAL_LOOPS = 3;
                const int CIRCLE_DURATION = 4000;
                const float BASE_LINEAR_SPEED = 50;
                const float BASE_TURN_RATIO = 0.6;
            
                for (int i = 0; i < MAX_SPIRAL_LOOPS; i++) {
                    int loopStartTime = nPgmTime;
                    float radiusFactor = BASE_TURN_RATIO + (i * 0.15);
                    float leftSpeed = (BASE_LINEAR_SPEED*OFFSET_POWER_FOR_LEFT_MOTOR) * radiusFactor;
                    float rightSpeed = BASE_LINEAR_SPEED;
            
                    while (nPgmTime - loopStartTime < CIRCLE_DURATION) {
                        motor[motorLeft] = leftSpeed;
                        motor[motorRight] = rightSpeed;
                        wait1Msec(50);
                    }
            
                    motor[motorLeft] = 0;
                    motor[motorRight] = 0;
                    wait1Msec(500);
                }
                break;
            }
                
            case 1:
                // Sector search
                for (int i = 0; i < 3; i++) {
                    moveDistance(spiralRadius * 0.6);
                    turnDegrees(45 + (rand() % 30), rand() % 2);
                    searchIteration++;
                }
                break;
                
            case 2:
                // Expanding square
                moveDistance(spiralRadius);
                turnDegrees(90 + (rand() % 15 - 7), rand() % 2);
                searchIteration++;
                break;
        }

        if (searchIteration > 5) {
            searchIteration = 0;
            if (rand() % 4 == 0) turnDegrees(360, rand() % 2);
        }
    }
}

void moveTowardsBall() {
    if (distances.distFL < distances.distFR) {
        moveDistance(distances.distFL/100.0 - 0.2);
        turnDegrees(30);
    } else {
        moveDistance(distances.distFR/100.0 - 0.2);
        turnDegrees(30, true);
    }
}

void returnToBase() {
    int target = 135;
    int degree = heading - target;
    if(degree < 0){
        turnDegrees(abs(degree), true);
    } else {
        turnDegrees(degree);
    }
    moveDistance(3.0, true);
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
    status.isfirstBallDelivered = false;
    status.panRight = false;
    
    // Test searching algorithm
    writeDebugStreamLine("=== TESTING SEARCHING ALGORITHM ===");
    startTask(searchingBallTask);
    wait1Msec(10000);
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