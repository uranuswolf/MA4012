#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)
#pragma config(Sensor, in6,    IR_A,           sensorAnalog)
#pragma config(Sensor, in5,    IR_B,           sensorAnalog)
#pragma config(Sensor, in4,    IR_C,           sensorAnalog)
#pragma config(Sensor, in3,    IR_D,           sensorAnalog)
#pragma config(Sensor, dgtl6,  limitswitchLB,  sensorDigitalIn)
#pragma config(Sensor, dgtl5,  limitswitchRB,  sensorDigitalIn)
#pragma config(Sensor, dgtl7,  limitswitchBall,sensorDigitalIn)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder) 
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder) 
#pragma config(Sensor, dgtl8,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11, compass_MSB,    sensorDigitalIn)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorServoContinuousRotation, openLoop)

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
#define DISTANCE_CORRECTION_FACTOR 3.5 // Adjust this factor based on your robot's calibration
#define OFFSET_POWER_FOR_LEFT_MOTOR 1.28 // Adjust this factor based on your robot's calibration


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
    bool panRight; // Flag to check if should pan left or right at the start of the search
    bool startLeft; // Flag to check if should start left or right at the start of the search
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
float sharpDistances[4]; // Index 0 = FC, 1 = FR, 2 = FL, 3 = BC


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
void returnToBase(void);
void resetStatus(void);
float getSharpDistance(tSensors sensor, int analogValue);


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
    while(true){
        searchingAlgo();
    }
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
float getSharpDistance(tSensors sensor, int analogValue) {
    if (analogValue <= 0) return 80.0; // Max distance fallback

    float voltage = (analogValue * 5.0) / 4095.0; // Convert analog reading to voltage

    switch (sensor) {
        case sharpFC: return 25.99 / pow(voltage, 1.22);
        case sharpFR: return 28.37 / pow(voltage, 1.08);
        case sharpFL: return 26.82 / pow(voltage, 1.28);
        case sharpBC: return 10.02 / pow(voltage, 1.26);
        default: return 80.0;
    }
}
task readSensorsTask() {
    bool hasPanDirectionSet = false; // Only allow setting panRight once
    while(true) {
        // Read IR sensors
        for(int i = 0; i < 4; i++) {
            IR_values[i] = SensorValue[IR_A + i] < 1000 ? 0 : 1;
        }

        // Read limit switches
        limitSwitches[0] = SensorValue[limitswitchLB];
        limitSwitches[1] = SensorValue[limitswitchRB];
        limitSwitches[2] = SensorValue[limitswitchBall];

       // Ball pickup status updates automatically based on limit switch
        status.isBallPicked = (limitSwitches[2] == 0);
            
        // Set panRight only once when left switch is pressed
        if (!hasPanDirectionSet && limitSwitches[0] == 0) {
            status.panRight = false;  // or false depending on your logic
            hasPanDirectionSet = true;
        }

        // Read compass
        heading = SensorValue[compass_MSB] * 8 + 
                 SensorValue[compass_Bit2] * 4 + 
                 SensorValue[compass_Bit3] * 2 + 
                 SensorValue[compass_LSB];

        tSensors sharpSensors[] = {sharpFC, sharpFR, sharpFL, sharpBC};

        for (int i = 0; i < 4; i++) {
            int sum = 0;
            for (int j = 0; j < SAMPLE_SIZE; j++) {
                sum += SensorValue[sharpSensors[i]];
                wait1Msec(2);
            }
            int avgAnalog = sum / SAMPLE_SIZE;
            sharpDistances[i] = getSharpDistance(sharpSensors[i], avgAnalog);
        
            // Update distances struct
            switch(i) {
                case 0: distances.distFC = sharpDistances[i]; break;
                case 1: distances.distFR = sharpDistances[i]; break;
                case 2: distances.distFL = sharpDistances[i]; break;
                case 3: distances.distBC = sharpDistances[i]; break;
            }
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
    float realDistance = distance * DISTANCE_CORRECTION_FACTOR;
    int targetTicks = distanceToTicks(realDistance);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = -(OFFSET_POWER_FOR_LEFT_MOTOR * BASE_POWER);
        motor[motorRight] = -BASE_POWER;
          
        
          
    } else {
        motor[motorLeft] = (OFFSET_POWER_FOR_LEFT_MOTOR  * BASE_POWER);
        motor[motorRight] = BASE_POWER;
          
        
          
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
           abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
      
 
      
}

void turnDegrees(float degrees, bool right) {
    float realDegrees = degrees * 2.5;
    float turningCircumference = PI * WHEEL_BASE;
    float arcLength = (realDegrees / 180.0) * turningCircumference;
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int reducedSpeed = 127 * 0.7;

    if (right) {
        motor[motorLeft] = reducedSpeed;
        motor[motorRight] = -reducedSpeed;
    } else {
        motor[motorLeft] = -reducedSpeed;
        motor[motorRight] = reducedSpeed;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
           abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
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
// Function to reset the status flags
void resetStatus() {
    status.isBoundary = false;
    status.isBall = false;
    status.isFrontObstacle = false;
    status.isBackObstacle = false;
    status.isBallPicked = false;
    status.reachedBase = false;
    status.isDelivered = false;
    status.isfirstBallDelivered = false;
    status.panRight = false;
}

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
    const float PAN_ANGLE = 20.0;
    const float INITIAL_DISTANCE = 1.2;
    const float ROW_DISTANCE = 0.3;

    // Phase 1: Runs exactly once at startup (before first delivery)
    if (!status.isfirstBallDelivered && searchIteration == 0) {
        searchIteration++;  // Increment here to ensure Phase 1 runs ONLY ONCE
        // 1st pan in 1st row 
        moveDistance(INITIAL_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, !status.panRight);
        wait1Msec(1000);

        // 2nd pan in 2nd row 
        moveDistance(ROW_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, !status.panRight);
        wait1Msec(1000);
            
        // 3rd pan in 3rd row
        moveDistance(ROW_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, !status.panRight);
        wait1Msec(1000);

    }
    else {
        // Phase 2: Competitive/spiral search (after first delivery or if Phase 1 fails)
        const float SPIRAL_BASE = 0.4;
        const float SPIRAL_INC = 0.15;
        
        float spiralRadius = SPIRAL_BASE + (searchIteration * SPIRAL_INC);
        
        
        int patternVariant = rand() % 3;
        
        switch(patternVariant) {
        
            case 0: { // Moving spiral search — robot drives in circles, increasing radius each loop
                const int MAX_SPIRAL_LOOPS = 3;
                const int CIRCLE_DURATION = 4000;  // Duration to complete one 360° arc
                const float BASE_LINEAR_SPEED = 50;  // Base forward speed
                const float BASE_TURN_RATIO = 0.6;   // Start with a tighter circle
            
                for (int i = 0; i < MAX_SPIRAL_LOOPS; i++) {
                    int loopStartTime = nPgmTime;
            
                    // Calculate the wheel speed difference to create circular motion
                    float radiusFactor = BASE_TURN_RATIO + (i * 0.15);  // Widen circle over time
                    float leftSpeed = (BASE_LINEAR_SPEED*OFFSET_POWER_FOR_LEFT_MOTOR) * radiusFactor;
                    float rightSpeed = BASE_LINEAR_SPEED;
            
                    // Drive in a circle for the duration of one full 360° arc
                    while (nPgmTime - loopStartTime < CIRCLE_DURATION) {
                        motor[motorLeft] = leftSpeed;
                        motor[motorRight] = rightSpeed;
                        wait1Msec(50);
                    }
            
                    // Brief stop and pause between loops
                    motor[motorLeft] = 0;
                    motor[motorRight] = 0;
                    wait1Msec(500);
                }
                break;
            }
                
            case 1: // Sector search
                for (int i = 0; i < 3; i++) {
                    moveDistance(spiralRadius * 0.6);
                    turnDegrees(45 + (rand() % 30), rand() % 2);
                    searchIteration++;  // Increment here for Phase 2's spiral expansion
                }
                break;
                
            case 2: // Expanding square
                moveDistance(spiralRadius);
                turnDegrees(90 + (rand() % 15 - 7), rand() % 2);
                searchIteration++;  // Increment here for Phase 2's spiral expansion
                break;
        }

        // Reset after full cycle to avoid infinite spiral growth
        if (searchIteration > 5) {
            searchIteration = 0;
            if (rand() % 4 == 0) turnDegrees(360, rand() % 2);  // Optional 360° scan
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
    // Stop motors immediately 
    motor[motorLeft] = 0;
    motor[motorRight] = 0; 

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
    // Stop motors immediately 
    motor[motorLeft] = 0;
    motor[motorRight] = 0; 
    
    // Get obstacle distances for more intelligent maneuvering
    float frontDist = distances.distFC;
    float leftDist = distances.distFL;
    float rightDist = distances.distFR;
    float backDist = distances.distBC;

    // Handle front obstacles if moving forward
    if (status.isFrontObstacle) {
        moveDistance(0.2, true);  // Back up 20cm
        
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
    resetStatus();

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
