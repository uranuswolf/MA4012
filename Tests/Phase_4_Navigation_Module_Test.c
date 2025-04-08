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
// ================================================================== Global Constants ==================================================================
const float PI = 3.14159265359;
const float WHEEL_DIAMETER = 0.06985;
const float WHEEL_BASE = 0.188;
const int TICKS_PER_REV = 90;
const int BASE_POWER = 30;
const float DISTANCE_CORRECTION_FACTOR = 3.85;
const float OFFSET_POWER_FOR_LEFT_MOTOR = 1.28;
const int ROLLER_SPEED = 127;
const float PAN_ANGLE = 20.0;
const float INITIAL_DISTANCE = 0.2;
const float ROW_DISTANCE = 0.2;
const float SPIRAL_BASE = 0.4;
const float SPIRAL_INC = 0.15;
const float MAX_DISTANCE = 300;

const float CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float DISTANCE_PER_TICK = CIRCUMFERENCE / TICKS_PER_REV;


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
} StatusFlags;

typedef enum RollerMode {
    INTAKE, 
    OUTPUT,
    STOP 
} RollerMode;

typedef enum FlapMode {
    PUSH, 
    OPEN,
    CLOSE 
} FlapMode;

typedef enum RobotState {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
} RobotState;

// ============================================================ Global Variables ==================================================================
DistanceSensors distances;
StatusFlags status;
RobotState currentState = SEARCH;

int IR_values[4];
int limitSwitches[3];
int heading;
bool panRightInitialized = false;
int searchIteration = 0;


//============================================================ Function Prototypes ==================================================================
void frontRollerControl(RollerMode mode);
void flapperControl(FlapMode mode);
float getSharpDistance(tSensors sensor, int analogValue);
void readSensors(void);
int distanceToTicks(float distance);
void moveDistance(float distance, bool backward=false);
void turnDegrees(float degrees, bool right=false);
void searchingAlgo(void);
void resetStatus(void);
void moveTowardsBall(void);
void searchPhase(void);
void collectPhase(void);
void returnPhase(void);
void decideTurn(float leftDist, float rightDist);
void handleObstacle(void);
void handleBoundary(void);
void returnToBase(void);


//============================================================ Helper Functions ==================================================================

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
            motor[FRONT_ROLLER] = ROLLER_SPEED;
            break;
        case OUTPUT:
            motor[FRONT_ROLLER] = -ROLLER_SPEED;
            break;
        case STOP:
            motor[FRONT_ROLLER] = 0;
            break;
    }
}

void flapperControl(FlapMode mode) {
    switch (mode) {
        case PUSH:
            motor[BACK_ROLLER] = -50;
            wait1Msec(800);
            motor[BACK_ROLLER] = 0;
            break;
        case OPEN:
            motor[BACK_ROLLER] = 50;
            wait1Msec(1500);
            motor[BACK_ROLLER] = 0;
            break;
        case CLOSE:
            motor[BACK_ROLLER] = -50;
            wait1Msec(300);
            motor[BACK_ROLLER] = 0;
            break;
    }
}

float getSharpDistance(tSensors sensor, int analogValue) {
    if (analogValue == 0) return 100; // max range

    float voltage = (analogValue * 5.0) / 4095.0; // ADC to voltage

    // Calibration for each sensor
    if (sensor == sharpFC) return 25.99 / pow(voltage, 1.22);
    if (sensor == sharpFR) return 28.37 / pow(voltage, 1.08);
    if (sensor == sharpFL) return 26.82 / pow(voltage, 1.28);
    if (sensor == sharpBC) return 10.02 / pow(voltage, 1.26);

    return 100; // default if unknown sensor
}

void readSensors() {
    // Read IR sensors
    tSensors IR_ports[4] = {IR_A, IR_B, IR_C, IR_D};
    for (int i = 0; i < 4; i++) {
        IR_values[i] = SensorValue[IR_ports[i]] < 2700 ? 1 : 0;
    }

    // Read limit switches
    limitSwitches[0] = SensorValue[limitswitchLB];
    limitSwitches[1] = SensorValue[limitswitchRB];
    limitSwitches[2] = SensorValue[limitswitchBall];

   if (!panRightInitialized) {
    status.panRight = (limitSwitches[0] == 0) ? true : false; // default to true
    panRightInitialized = true;
}
   
    // Read compass
    heading = SensorValue[compass_MSB] * 8 + 
             SensorValue[compass_Bit2] * 4 + 
             SensorValue[compass_Bit3] * 2 + 
             SensorValue[compass_LSB];

   // Read sharp sensors with improved averaging
   int rawFC = SensorValue[sharpFC];
   int rawFR = SensorValue[sharpFR];
   int rawFL = SensorValue[sharpFL];
   int rawBC = SensorValue[sharpBC];

   distances.distFC = getSharpDistance(sharpFC, rawFC); // Update the distances struct
   distances.distFR = getSharpDistance(sharpFR, rawFR); // Update the distances struct
   distances.distFL = getSharpDistance(sharpFL, rawFL); // Update the distances struct
   distances.distBC = getSharpDistance(sharpBC, rawBC); // Update the distances struct

    // Update status flags
    status.isBoundary = (IR_values[0] || IR_values[1]  || 
                         IR_values[2] || IR_values[3] );

    status.isFrontObstacle = (distances.distFC >= 5.0 && distances.distFC <= 40.0);

    status.isBackObstacle = (distances.distBC >= 5.0 && distances.distBC <= 40.0);

    // Only allow resetting isBallPicked during RETURN state
    if (currentState == DELIVER) {
    status.isBallPicked = (limitSwitches[2] == 0); 
     } else {
    // Once it's true, don't let it go false until RETURN state
    status.isBallPicked = status.isBallPicked || (limitSwitches[2] == 0);
}



    status.isBall = ((distances.distFL >= 5.0 && distances.distFL <= 70.0) || 
    (distances.distFR >= 5.0 && distances.distFR <= 70.0)) &&
    (!status.isFrontObstacle);

}

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
        motor[motorLeft] = (OFFSET_POWER_FOR_LEFT_MOTOR * BASE_POWER);
        motor[motorRight] = BASE_POWER;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void turnDegrees(float degrees, bool right) {
    float realDegrees = degrees * 2.9;
    float turningCircumference = PI * WHEEL_BASE;
    float arcLength = (realDegrees / 180.0) * turningCircumference;
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int reducedSpeed = 127;  // You can tweak this to adjust turn speed

    if (right) {
        motor[motorLeft] = (OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed);
        motor[motorRight] = -reducedSpeed;
    } else {
        // Adjusted the power for the left motor to prevent moving backward
        motor[motorLeft] = -(OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed);  // Left motor should go backward
        motor[motorRight] = (reducedSpeed);  // Right motor should go forward
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}


void searchingAlgo() {

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
        // Phase 2: Sector search only
      
        float spiralRadius = SPIRAL_BASE + (searchIteration * SPIRAL_INC);

        // Sector search pattern (only)
        for (int i = 0; i < 3; i++) {
            moveDistance(spiralRadius * 0.3);
            turnDegrees(45 + (rand() % 30), rand() % 2);
            searchIteration++;  // Spiral expansion retained
        }

        // Reset after full cycle to avoid infinite spiral growth
        if (searchIteration > 5) {
            searchIteration = 1;
            if (rand() % 4 == 0) {
                turnDegrees(360, rand() % 2);  // Optional 360° scan
            }
        }
    }
}


void moveTowardsBall() {
    float leftDist = distances.distFL;
    float rightDist = distances.distFR;

    float targetDistance;
    bool turnRight;

    if (leftDist < rightDist) {
        targetDistance = leftDist;
        turnRight = false;
    } else {
        targetDistance = rightDist;
        turnRight = true;
    }

    float moveDist = (targetDistance * 0.01) - 0.20; // Convert cm to m and add safety buffer

    if (moveDist < 0.05) moveDist = 0.05; // Safety minimum distance

    moveDistance(moveDist, false);
    turnDegrees(25, turnRight);
    moveDistance(0.3, false); // Final forward approach
}

// Boundary Handling Function
void handleBoundary() {
    // Stop motors immediately 
    motor[motorLeft] = 0;
    motor[motorRight] = 0;

    // Read sensor states (0 = boundary detected)
    bool frontRight = (IR_values[0] == 0);
    bool frontLeft  = (IR_values[1] == 0);
    bool backRight  = (IR_values[2] == 0);
    bool backLeft   = (IR_values[3] == 0);

    // Combined special cases first
    if (frontRight && backRight && limitSwitches[0] && limitSwitches[1]) {
        turnDegrees(90, true);        // Turn right
        moveDistance(0.25);           // Move forward 25cm
    }
    else if (frontLeft && backLeft && limitSwitches[0] && limitSwitches[1]) {
        turnDegrees(90, false);       // Turn left
        moveDistance(0.25);           // Move forward 25cm
    }
    // Handle individual front triggers
    else if ((frontRight || frontLeft) && limitSwitches[0] && limitSwitches[1]) {
        moveDistance(0.25, true);     // Move back 25cm
    }
    // Handle individual rear triggers
    else if ((backRight || backLeft) && limitSwitches[0] && limitSwitches[1]) {
        moveDistance(0.25);           // Move forward 25cm
    }
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
    
    // Handle front obstacles if moving forward
    if (status.isFrontObstacle) {
        if (distances.distFC <= 0.2) {
            moveDistance(0.2, true);  // Back up 20cm
        }
        
        wait1Msec(2000);  // Let the background task update sensor readings

        // Recheck if the front obstacle is still present
        if (status.isFrontObstacle) {
            decideTurn(distances.distFL, distances.distFR);
        }
        return;
    } 
    
    // Handle back obstacles if reversing
    else if (status.isBackObstacle && motor[motorLeft] < 0 && motor[motorRight] < 0) {
        if (distances.distBC <= 0.2) {
            moveDistance(0.2);  // Move forward 20cm
        }

        wait1Msec(2000);  // Allow time for sensor update

        // Recheck if the back obstacle is still present
        if (status.isBackObstacle) {
            decideTurn(distances.distFL, distances.distFR);
            moveDistance(0.2);  // Move forward slightly after turn
        }
        return;
    } 
}

void returnToBase() {
    while (true) {
        int target = 135;
        int degree = heading - target;
        if (degree < 0) {
            turnDegrees(degree, true);
        } else {
            turnDegrees(degree);
        }

        moveDistance(MAX_DISTANCE / 100.0, true); //Move backwards

        // Check if the condition to stop is met
        if (heading == 135 &&
            (limitSwitches[0] == 0 || limitSwitches[1] == 0) && //0 means limit swtich is pressed
            (IR_values[2] == 0 && IR_values[3] == 0)) {
            status.reachedBase = true;
            break;
        }

        // Else keep repeating forward, reorient, backward until the condition is true
        while (true) {
            moveDistance(0.25); // Move forward 0.25m

            // Reorient to 135 degrees again
            degree = heading - target;
            if (degree < 0) {
                turnDegrees(degree, true);
            } else {
                turnDegrees(degree);
            }

            moveDistance(0.4, true); // Move backward 0.4m

            // Check condition again
            if (heading == 135 &&
                (limitSwitches[0] == 0 || limitSwitches[1] == 0) &&
                (IR_values[2] == 0 && IR_values[3] == 0)) {
                status.reachedBase = true;
                return;
            }

            wait1Msec(50);
        }
    }
}

//============================================================ Task Definitions ==================================================================

task readSensorsTask() {
    while(true) {
        readSensors();
        wait1Msec(50);
    }
}

task searchingBallTask() {
    while(true) {
        searchingAlgo(); //the second time this function is called, then it will be in the else statement
        wait1Msec(100);
    }
}

task moveTowardsBallTask() {
    while(true) {
        moveTowardsBall();
    }
}

task returnToBaseTask(){
        returnToBase();
    }

task testSensorModuleTask() {
     while(true) {
        // Display all sensor readings
        writeDebugStreamLine("\n--- SENSOR READINGS ---");
        writeDebugStreamLine("IR Sensors: FR=%d FL=%d BR=%d BL=%d", 
                               IR_values[0], IR_values[1], IR_values[2], IR_values[3]);
        writeDebugStreamLine("Limit Switches: LB=%d RB=%d Ball=%d",
                               limitSwitches[0], limitSwitches[1], limitSwitches[2]);
        writeDebugStreamLine("Compass Heading: %d", heading);
        writeDebugStreamLine("Distances: FC=%.1fcm FR=%.1fcm FL=%.1fcm BC=%.1fcm",
                               distances.distFC, distances.distFR, 
                               distances.distFL, distances.distBC);
            
        writeDebugStreamLine("\n--- STATUS FLAGS ---");
        writeDebugStreamLine("Boundary: %d | Ball: %d | F.Obst: %d | B.Obst: %d",
                               status.isBoundary, status.isBall,
                               status.isFrontObstacle, status.isBackObstacle);
        writeDebugStreamLine("Ball Picked: %d | Pan Right: %d | First Delivered: %d",
                               status.isBallPicked, status.panRight, status.isfirstBallDelivered);
        writeDebugStreamLine("Raw IR: A=%d B=%d C=%d D=%d", 
        SensorValue[IR_A], SensorValue[IR_B], SensorValue[IR_C], SensorValue[IR_D]);
    
            
            wait1Msec(50); // Reduced delay for more frequent updates
        }
    }
    
//============================================================ Phase Functions ==================================================================

void searchPhase() {
    startTask(searchingBallTask); // Start searching for the ball

    while(currentState == SEARCH) {

        // Step 1: Handle boundary first
        if (status.isBoundary) {
            stopTask(searchingBallTask); 
            frontRollerControl(STOP); // Stop the front roller
            handleBoundary();
            wait1Msec(500); // small pause to stabilize
            startTask(searchingBallTask); // Restart searching after handling obstacle
            frontRollerControl(INTAKE); // Resume roller intake
            continue;       // restart loop
        }

        // If an obstacle is detected, handle it
        if(status.isFrontObstacle || status.isBackObstacle) {
            stopTask(searchingBallTask); 
            frontRollerControl(STOP); // Stop the front roller
            handleObstacle(); // Stop search and handle the obstacle
            wait1Msec(500); // small pause to stabilize
            startTask(searchingBallTask); // Restart searching after handling obstacle
            frontRollerControl(INTAKE); // Resume roller intake
            continue;       // restart loop
        }

        if(status.isBall) {
            currentState = COLLECT;
            stopTask(searchingBallTask); // Stop searching when ball is found
            break;
        }
        
        if(status.isBallPicked) {  // Bypass collectPhase and jump straight to RETURN if ball is picked
            currentState = RETURN;
            stopTask(searchingBallTask); // Stop searching if ball is picked
            frontRollerControl(STOP); // Stop the front roller
            break;
        }

        wait1Msec(100); // Wait to avoid tight loop
    }
}


void collectPhase() {
    flapperControl(OPEN); // Open the flapper to pick up the ball
    startTask(moveTowardsBallTask);
    
    while(currentState == COLLECT) {
        // Step 1: Handle boundary first
        if (status.isBoundary) {
            stopTask(moveTowardsBallTask); 
            frontRollerControl(STOP); // Stop the front roller
            handleBoundary();
            wait1Msec(500); // small pause to stabilize
            startTask(moveTowardsBallTask); // Restart searching after handling obstacle
            frontRollerControl(INTAKE); // Resume roller intake
            continue;       // restart loop
        }

        // If an obstacle is detected, handle it
        if(status.isFrontObstacle || status.isBackObstacle) {
            stopTask(moveTowardsBallTask); 
            frontRollerControl(STOP); // Stop the front roller
            handleObstacle(); // Stop search and handle the obstacle
            wait1Msec(500); // small pause to stabilize
            startTask(moveTowardsBallTask); // Restart searching after handling obstacle
            frontRollerControl(INTAKE); // Resume roller intake
            continue;       // restart loop
        }

        // if the ball is no longer in sight 
        if(!status.isBall) {
            stopTask(moveTowardsBallTask);
            currentState = SEARCH;
            break;
        }
        if (status.isBallPicked){
            stopTask(moveTowardsBallTask);
            frontRollerControl(STOP); // Stop the front roller
            // Close the flapper
            flapperControl(CLOSE);
            currentState = RETURN;
            break;
        }
        wait1Msec(100);
    }
}

void returnPhase() {
    startTask(returnToBaseTask); // Start returning to base
    while(currentState == RETURN) {
        // Step 1: Handle boundary first
        if (status.isBoundary) {
            stopTask(returnToBaseTask); 
            handleBoundary();
            wait1Msec(500); // small pause to stabilize
            startTask(returnToBaseTask); // Restart searching after handling obstacle
            continue;       // restart loop
        }

        // If an obstacle is detected, handle it
        if(status.isFrontObstacle || status.isBackObstacle) {
            stopTask(returnToBaseTask); 
            handleObstacle();
            wait1Msec(500); // small pause to stabilize
            startTask(returnToBaseTask); // Restart searching after handling obstacle
            continue;       // restart loop
        }
      
        if (status.reachedBase) {
            stopTask(returnToBaseTask); 
            currentState = DELIVER;
            break;  
        }
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

//============================================================ Main Task ==================================================================
task main() {
    // Initialize all status flags to false
    resetStatus();
    // Start front roller
    frontRollerControl(INTAKE);
    // Open the flapper to pick up the ball
    flapperControl(OPEN);
    // Initialize sensor reading task
    startTask(readSensorsTask);
    startTask(testSensorModuleTask); // Start reading sensors
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