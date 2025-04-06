#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)
#pragma config(Sensor, in3,    IR_A,           sensorAnalog)
#pragma config(Sensor, in4,    IR_B,           sensorAnalog)
#pragma config(Sensor, in5,    IR_C,           sensorAnalog)
#pragma config(Sensor, in6,    IR_D,           sensorAnalog)
#pragma config(Sensor, dgtl6,  limitswitchLB,  sensorDigitalIn)
#pragma config(Sensor, dgtl5,  limitswitchRB,  sensorDigitalIn)
#pragma config(Sensor, dgtl7,  limitswitchBall,sensorDigitalIn)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder) 
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder) 
#pragma config(Sensor, dgtl8,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11, compass_MSB,    sensorDigitalIn)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorVex393_MC29, openLoop)

// ================================================================== Constants ==================================================================
#define PI 3.14159265359
#define WHEEL_DIAMETER 0.06985 // meters
#define WHEEL_BASE 0.188 // meters
#define TICKS_PER_REV 90
#define BASE_POWER 50
#define MAX_DISTANCE 300 // cm
#define ROLLER_SPEED 127 // speed of the roller motors
#define SAMPLE_SIZE 5
#define CIRCUMFERENCE (WHEEL_DIAMETER * PI) // meters
#define DISTANCE_PER_TICK (CIRCUMFERENCE / TICKS_PER_REV) // meters per tick

// ================================================================== Types & Enums ==================================================================
typedef enum RobotState {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
} RobotState;

typedef enum FlapMode {
    PUSH, 
    OPEN,
    CLOSE 
} FlapMode;

typedef enum RollerMode {
    INTAKE, 
    OUTPUT,
    STOP 
} RollerMode;

typedef struct {
    float distFC;
    float distFR;
    float distFL;
    float distBC;
} DistanceSensors;

typedef struct {
    bool isBoundary;
    bool isBall;
    bool isFrontObstacle;
    bool isBackObstacle;
    bool isBallPicked;
    bool reachedBase;
    bool isDelivered;
    bool isfirstBallDelivered; // Flag to check if the first ball is delivered
} StatusFlags;

// ================================================================== Global Variables ==================================================================
RobotState currentState = SEARCH;
StatusFlags status;
DistanceSensors distances;

// Interrupt system variables
bool boundaryInterruptFlag = false;
bool boundaryInterruptActive = false;
bool obstacleInterruptFlag = false;
bool obstacleInterruptActive = false;

int IR_values[4]; // 0:FR, 1:FL, 2:BR, 3:BL
int limitSwitches[3]; // 0:LB, 1:RB, 2:Ball
int heading;
int lastBoundaryTurn = -1; // -1 = no previous turn, 0 = left, 1 = right


// ================================================================== Function Prototypes ==================================================================
// Movement functions
void moveDistance(float distance, bool backward = false);
void turnDegrees(float degrees, bool right = false);
int distanceToTicks(float distance);

// State functions
void searchPhase(void);
void collectPhase(void);
void returnPhase(void);
void deliverPhase(void);

// Helper functions
void handleBoundary(void);
void handleObstacle(void);
void frontRollerControl(RollerMode mode);
void flapperControl(FlapMode mode);
void searchingAlgo(void);
void moveTowardsBall(void);
void decideTurn(float leftDist, float rightDist);
void stopMotors(void);
void returnToBase(void);


// Interrupt handlers
void boundaryInterrupt(void);
void obstacleInterrupt(void);

// ================================================================== Tasks definition ==================================================================

task moveTowardsBallTask() {
    moveTowardsBall();
}
task returnToBaseTask(){
    returnToBase();
}

task searchingBallTask(){
    searchingAlgo();
}


// ================================================================== Interrupt Handlers ==================================================================
void boundaryInterrupt() {
    if (!boundaryInterruptActive) {
        boundaryInterruptFlag = true;
        boundaryInterruptActive = true;
    }
}

void obstacleInterrupt() {
    if (!obstacleInterruptActive && !boundaryInterruptActive) {
        obstacleInterruptFlag = true;
        obstacleInterruptActive = true;
    }
}

// ================================================================== Sensor Task ==================================================================
task readSensorsTask() {
    while(true) {
        // Read IR sensors
        for(int i = 0; i < 4; i++) {
            IR_values[i] = SensorValue[IR_A + i] < 300 ? 0 : 1;
        }

        // Read limit switches
        limitSwitches[0] = SensorValue[limitswitchLB];
        limitSwitches[1] = SensorValue[limitswitchRB];
        limitSwitches[2] = SensorValue[limitswitchBall];

       // Ball pickup status updates automatically based on limit switch
        status.isBallPicked = (limitSwitches[2] == 0);


        // Read compass
        heading = SensorValue[compass_MSB] * 8 + 
                 SensorValue[compass_Bit2] * 4 + 
                 SensorValue[compass_Bit3] * 2 + 
                 SensorValue[compass_LSB];

        // Read and convert sharp sensors with filtering
        tSensors sharpSensors[] = {sharpFC, sharpFR, sharpFL, sharpBC};
        float* distPtr[] = {&distances.distFC, &distances.distFR, 
                           &distances.distFL, &distances.distBC};
        
        for(int i = 0; i < 4; i++) {
            int sum = 0;
            for(int j = 0; j < SAMPLE_SIZE; j++) {
                sum += SensorValue[sharpSensors[i]];
                wait1Msec(2);
            }
            
            float voltage = ((sum / SAMPLE_SIZE) * 5.0) / 4095.0;
            *distPtr[i] = (voltage == 0) ? 80.0 : 
                         (i == 0) ? 25.99 / pow(voltage, 1.22) :
                         (i == 1) ? 28.37 / pow(voltage, 1.08) :
                         (i == 2) ? 26.82 / pow(voltage, 1.28) :
                         10.02 / pow(voltage, 1.26);
        }

        // Update status flags and trigger interrupts
        status.isBoundary = (IR_values[0] == 0 || IR_values[1] == 0 || 
                           IR_values[2] == 0 || IR_values[3] == 0);
        
        status.isFrontObstacle = (distances.distFC >= 10.0 && distances.distFC <= 40.0);
        status.isBackObstacle = (distances.distBC >= 10.0 && distances.distBC <= 40.0);
        
        // Boundary has priority over obstacle
        if (status.isBoundary && !boundaryInterruptActive) {
            boundaryInterrupt();
        }
        else if (!status.isBoundary && boundaryInterruptActive) {
            boundaryInterruptActive = false;
        }
        
        // Only check for obstacles if no boundary is active
        if (!boundaryInterruptActive) {
            if ((status.isFrontObstacle || status.isBackObstacle) && !obstacleInterruptActive) {
                obstacleInterrupt();
            }
            else if (!status.isFrontObstacle && !status.isBackObstacle && obstacleInterruptActive) {
                obstacleInterruptActive = false;
            }
        }

        // Additional status flags
        status.isBall = ((distances.distFL >= 10.0 && distances.distFL <= 70.0) || 
                       (distances.distFR >= 10.0 && distances.distFR <= 70.0)) &&
                      !(distances.distFC >= 10.0 && distances.distFC <= 40.0);

        wait1Msec(50);
    }
}

// ================================================================== Movement Functions ==================================================================
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

// ================================================================== State Functions ==================================================================
void searchPhase() {
    startTask(searchingBallTask); // Start searching for the ball

    while(currentState == SEARCH) {
        if(status.isBall) {
            currentState = COLLECT;
            break;
        }    
        // Check for interrupts
        if (boundaryInterruptFlag) {
            stopTask(searchingBallTask);
            handleBoundary();
            boundaryInterruptFlag = false;
            startTask(searchingBallTask); // Restart searching after handling boundary
        }
        else if (obstacleInterruptFlag) {
            stopTask(searchingBallTask);
            handleObstacle();
            obstacleInterruptFlag = false;
            startTask(searchingBallTask); // Restart searching after handling obstacle

        }
        wait1Msec(100);
    }

}

void collectPhase() {
    frontRollerControl(INTAKE); // Start roller intake
    flapperControl(OPEN); // Open the flapper to pick up the ball
    startTask(moveTowardsBallTask);
    
    while(currentState == COLLECT) {
        // if the ball is no longer in sight 
        if(!status.isBall) {
            frontRollerControl(STOP);
            stopTask(moveTowardsBallTask);
            currentState = SEARCH;
            break;
        }
        
        if(limitSwitches[2] == 0) { // Ball picked up
            frontRollerControl(STOP);
            status.isBallPicked = true;
            stopTask(moveTowardsBallTask);
            flapperControl(CLOSE); // Close the flapper to hold the ball
            currentState = RETURN;
            break;
        }
        
        // Check for interrupts
        if (boundaryInterruptFlag) {
            frontRollerControl(STOP);
            stopTask(moveTowardsBallTask);
            handleBoundary();
            boundaryInterruptFlag = false;
            startTask(moveTowardsBallTask); // Restart moving towards ball after handling boundary
            frontRollerControl(INTAKE); // Resume roller intake
        }
        else if (obstacleInterruptFlag) {
            frontRollerControl(STOP);
            stopTask(moveTowardsBallTask);
            handleObstacle();
            obstacleInterruptFlag = false;
            startTask(moveTowardsBallTask); // Restart moving towards ball after handling obstacle
            frontRollerControl(INTAKE); // Resume roller intake
        }
        wait1Msec(100);
    }
}

void returnPhase() {
    startTask(returnToBaseTask);
    
    while(currentState == RETURN) {
       /// Check if the robot has reached the base
        if (status.reachedBase) {
            stopTask(returnToBaseTask);
            currentState = DELIVER;
            status.reachedBase = false;
            break;
        }
 
        // Check for interrupts
        if (boundaryInterruptFlag) {
            stopTask(returnToBaseTask);
            handleBoundary();
            boundaryInterruptFlag = false;
            startTask(returnToBaseTask); // Restart returning to base after handling boundary
        }
        else if (obstacleInterruptFlag) {
            stopTask(returnToBaseTask);
            handleObstacle();
            obstacleInterruptFlag = false;
            startTask(returnToBaseTask); // Restart returning to base after handling obstacle
        }
        wait1Msec(100);
    }

}

void deliverPhase() {
    // Activate flapper to deliver ball
    flapperControl(PUSH);
    wait1Msec(1000); // Give time to deliver

    // Open the flapper and check if the ball is delivered
    flapperControl(OPEN);

    if (!status.isBallPicked) {
        status.isDelivered = true; // Ball is delivered
        currentState = SEARCH;     // Move to search state
        if (!status.isfirstBallDelivered) {
            status.isfirstBallDelivered = true; // First ball delivered
        }
    }

    // if the ball is still not released, keep pushing it out
    while (status.isBallPicked) {
        // If the ball is still picked, push the ball out again
        flapperControl(PUSH);
        wait1Msec(1000); // Give time to hold

        // Open the flapper again to release the ball
        flapperControl(OPEN);

        // Give some time for the ball to settle after opening the flapper
        wait1Msec(500);  // Adjust this as needed for ball behavior
        
        // If the ball is no longer picked, mark it as delivered
        if (!status.isBallPicked) {
            status.isDelivered = true; // Ball is delivered
            currentState = SEARCH;     // Move to search state
            if (!status.isfirstBallDelivered) {
                status.isfirstBallDelivered = true; // First ball delivered
            }
            break; // Exit loop once ball is delivered
        }
    }
}

// ================================================================== Helper Functions ==================================================================
void frontRollerControl(RollerMode mode) {
    switch(mode){
        case INTAKE:
            motor[FRONT_ROLLER] = -ROLLER_SPEED; // Intake speed
            break;
        case OUTPUT:
            motor[FRONT_ROLLER] = ROLLER_SPEED; // Output speed
            break;
        case STOP:
            motor[FRONT_ROLLER] = 0; // Stop roller
            break;
    }
}

void flapperControl(FlapMode mode) {
    switch (mode) {
        case PUSH:
            // Implement PUSH behavior
            motor[BACK_ROLLER] = -50; // Push the ball
            wait1Msec(800);
            break;

        case OPEN:
            // Implement OPEN behavior
            motor[BACK_ROLLER] = 50;
            wait1Msec(1500);
            motor[BACK_ROLLER] = 0;
            break;

        case CLOSE:
            // Implement CLOSE behavior
            motor[BACK_ROLLER] = -50;
            wait1Msec(300);
            motor[BACK_ROLLER] = 0;
            break;
    }
}

void searchingAlgo() {
    static int searchIteration = 0;
    const float PAN_ANGLE = 60.0;  // Degrees to pan left/right
    const float INITIAL_DISTANCE = 1.2; // Meters to move forward initially

    // Phase 1: Initial ball acquisition (known position)
    if (!isfirstBallDelivered) {
        // Move forward to expected ball location
        moveDistance(INITIAL_DISTANCE);
        
        // If ball not found, perform targeted pan search
        if (!status.isBall) {
            // Alternate pan directions each time
            bool panRight = (searchIteration % 2 == 0);
            
            // First pan
            turnDegrees(PAN_ANGLE, panRight);
            wait1Msec(300);  // Sensor settling time
            
            // Second pan (wider angle)
            turnDegrees(PAN_ANGLE * 1.5, !panRight);
            wait1Msec(300);
            
            // Return to center
            turnDegrees(PAN_ANGLE / 2, panRight);
        }
        
        if (status.isBall) {
            firstBallFound = true;
            return;
        }
        
        searchIteration++;
        return;
    }

    // Phase 2: Competitive search pattern after first delivery
    const float SPIRAL_BASE = 0.4;
    const float SPIRAL_INC = 0.15;
    const int QUAD_TIME = 1200;  // ms per quadrant
    
    // Adaptive spiral search with random elements
    float spiralRadius = SPIRAL_BASE + (searchIteration * SPIRAL_INC);
    int startTime = nPgmTime;
    
    // Randomize the search pattern slightly to avoid predictability
    int patternVariant = rand() % 3;
    
    switch(patternVariant) {
        case 0: // Archimedean spiral
            while (nPgmTime - startTime < QUAD_TIME) {
                float progress = (nPgmTime - startTime) / (float)QUAD_TIME;
                float currentAngle = progress * 90.0;
                
                // Use sinDegrees directly (RobotC supports it)
                float angular = 15.0 * sinDegrees(currentAngle * 4); // Using sinDegrees

                motor[motorLeft] = BASE_POWER * 0.7 - angular;
                motor[motorRight] = BASE_POWER * 0.7 + angular;
                
                wait1Msec(50);
                
                if (status.isBall) {
                    stopMotors();
                    return;
                }
            }
            break;
            
        case 1: // Sector search with quick scans
            for (int i = 0; i < 3; i++) {
                // Quick forward
                moveDistance(spiralRadius * 0.6);
                
                // Sharp turn
                turnDegrees(45 + (rand() % 30), rand() % 2);
                
                if (status.isBall) return;
            }
            break;
            
        case 2: // Expanding square
            moveDistance(spiralRadius);
            turnDegrees(90 + (rand() % 15 - 7), rand() % 2);
            break;
    }
    
    searchIteration++;
    
    // Reset spiral after full cycle to prevent getting stuck
    if (searchIteration > 8) {
        searchIteration = 0;
        
        // Occasionally do a full 360° scan
        if (rand() % 4 == 0) {
            turnDegrees(360, rand() % 2);
        }
    }
}



void moveTowardsBall() {

        if (distances.distFL < distances.distFR) {
            // Ball is more to the left
            moveDistance(distances.distFL - 20); // Adjust movement
            wait1Msec(500);
            turnDegrees(30);
        }
        // If the ball is detected by the right sensor
        else if (distances.distFR < distances.distFL) {
            // Ball is more to the right
            moveDistance(distances.distFR - 20); // Adjust movement
            wait1Msec(500);
            turnDegrees(30, true);
}
}


// Boundary Handling Function
void handleBoundary() {
    stopMotors(); // Stop motors immediately with braking

    // Determine which boundary sensors are triggered
    bool frontLeft = (IR_values[1] == 0);
    bool frontRight = (IR_values[0] == 0);
    bool backLeft = (IR_values[3] == 0);
    bool backRight = (IR_values[2] == 0);

    int thisTurn = -1;  // Initializing to an invalid turn value

    if (frontLeft || frontRight) {  // Front boundary detected
        moveDistance(-0.25, true);  // Back up 25cm

        // Decide turn direction
        if (frontLeft && !frontRight) {
            thisTurn = 1;  // Turn right
        } else if (frontRight && !frontLeft) {
            thisTurn = 0;  // Turn left
        } else {
            thisTurn = rand() % 2;  // Random if both triggered
        }

        // Avoid repeating last turn direction
        if (thisTurn == lastBoundaryTurn) {
            thisTurn = 1 - thisTurn;  // Flip direction if same as last
        }

        lastBoundaryTurn = thisTurn;  // Remember the turn
        turnDegrees(120, thisTurn == 1);  // Perform the turn
        moveDistance(0.15);  // Cooldown: move forward slightly
    } 
    else {  // Rear boundary detected
        moveDistance(0.25);  // Move forward 25cm

        // Decide turn direction
        if (backLeft && !backRight) {
            thisTurn = 1;  // Turn right
        } else if (backRight && !backLeft) {
            thisTurn = 0;  // Turn left
        } else {
            thisTurn = rand() % 2;  // Random if both triggered
        }

        // Avoid repeating last turn direction
        if (thisTurn == lastBoundaryTurn) {
            thisTurn = 1 - thisTurn;  // Flip direction if same as last
        }

        lastBoundaryTurn = thisTurn;  // Remember the turn
        turnDegrees(90, thisTurn == 1);  // Perform the turn
        moveDistance(0.15);  // Cooldown: move forward slightly
    }

    // Reset flags for boundary
    boundaryInterruptFlag = false;
    boundaryInterruptActive = false;
}

// Helper function to decide turn direction based on side distances
void decideTurn(float leftDist, float rightDist) {

    if (leftDist > rightDist && leftDist > 40) {
        turnDegrees(90, false);  // More space on left, turn left
    } else if (rightDist > leftDist && rightDist > 40) {
        turnDegrees(90, true);   // More space on right, turn right
    } else {
        turnDegrees(90, rand() % 2);  // Random turn if no clear direction
    }
}

// Obstacle Handling Function
void handleObstacle() {
    stopMotors();  // Stop motors immediately with braking

    // Get obstacle distances for more intelligent maneuvering
    float frontDist = distances.distFC;
    float leftDist = distances.distFL;
    float rightDist = distances.distFR;
    float backDist = distances.distBC;

    // Handle front obstacles if moving forward
    if (status.isFrontObstacle) {
        moveDistance(-0.2, true);  // Back up 20cm
        
        // If the front distance is very small, prefer turning to the side with more space
        if (frontDist < 0.5) {  // Adjust threshold based on sensor range
            decideTurn(leftDist, rightDist);  // Decide turn direction based on side distances
        } else {
            // Otherwise, just move straight and handle turn later
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
        }
    } 
    // Handle back obstacles while reversing
    else if (status.isBackObstacle && motor[motorLeft] > 0 && motor[motorRight] < 0) {
        moveDistance(0.2);  // Move forward 20cm
        
        // If the back distance is very small, prefer turning to the side with more space
        if (backDist < 0.5) {  // Adjust threshold based on sensor range
            decideTurn(leftDist, rightDist);  // Decide turn direction based on side distances
        } else {
            // Otherwise, just stop or adjust movement
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
        }
    }

    // Reset flags for obstacle
    obstacleInterruptFlag = false;
    obstacleInterruptActive = false;
}


void returnToBase(){
    while (true) {
        int target = 135;
        int degree = heading - target;
        if(degree <0){
            turnDegrees(degree, true);
        }
        else{
            turnDegrees(degree);
        }
        moveDistance(MAX_DISTANCE / 100.0, true);

        if (heading == 135 && (limitSwitches[0] == 0 || limitSwitches[1] == 0) && 
            (IR_values[2] == 0 && IR_values[3] == 0)) {
            status.reachedBase = true;
            break;
        }
        wait1Msec(50);
    }
}

// ================================================================== Main Task ==================================================================
task main() {
    // Initialize all status flags to false
    status = (StatusFlags){0};

    // Initialize sensor reading task
    startTask(readSensorsTask);
    wait1Msec(500); // Allow sensors to stabilize

    while(true) {
        switch(currentState) {
            case SEARCH:
                searchPhase();
                break;
            case COLLECT:
                collectPhase();
                break;
            case RETURN:
                returnPhase();
                break;
            case DELIVER:
                deliverPhase();
                break;
        }
        wait1Msec(100);
    }
}
