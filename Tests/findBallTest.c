//=================================================================Pragma Configuration=================================================================
#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)
#pragma config(Sensor, dgtl6,  limitswitchLB,  sensorDigitalIn)
#pragma config(Sensor, dgtl5,  limitswitchRB,  sensorDigitalIn)
#pragma config(Sensor, dgtl7,  limitswitchBall,sensorDigitalIn)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder) 
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder) 
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorServoContinuousRotation, openLoop)

//================================================================= Sensor MACROS ==================================================================
#define sharpFC        in8
#define sharpFL        in2
#define sharpBC        in7
#define sharpFR        in1


#define limitswitchLB  dgtl6
#define limitswitchRB  dgtl5
#define limitswitchBall dgtl7

#define RIGHT_ENCODER  dgtl1
#define LEFT_ENCODER   dgtl3


#define motorLeft      port3
#define motorRight     port2
#define FRONT_ROLLER   port8
#define BACK_ROLLER    port9

// ================================================================== Global Constants ==================================================================
const float PI = 3.14159265359;
const float WHEEL_DIAMETER = 0.06985;
const float WHEEL_BASE = 0.188;
const int TICKS_PER_REV = 90;
const int BASE_POWER = 30;
const float DISTANCE_CORRECTION_FACTOR = 3.85;
const float OFFSET_POWER_FOR_LEFT_MOTOR = 1.28;
const int ROLLER_SPEED = 127;
const float PAN_ANGLE = 20;
const float INITIAL_DISTANCE = 0.3;
const float ROW_DISTANCE = 0.3;

const float CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float DISTANCE_PER_TICK = CIRCUMFERENCE / TICKS_PER_REV;

//============================================================ Struct definition ==================================================================
typedef struct {
    float distFC;
    float distFR;
    float distFL;
    float distBC;
} DistanceSensors;

typedef struct {
    bool isBall;
    bool isBallPicked;
    bool panRight;
    bool isfirstBallDelivered;
    bool isBallDetectedFlag;
} StatusFlags;

//============================================================ Enum definition ==================================================================
typedef enum RollerMode {
    INTAKE, 
    OUTPUT,
    STOP 
} RollerMode;

typedef enum RobotState {
    SEARCH,
    COLLECT,
    DELIVER
} RobotState;

// ============================================================ Global Variables ==================================================================
DistanceSensors distances;
StatusFlags status;
RobotState currentState = SEARCH;

int limitSwitches[3];
bool panRightInitialized = false;
int searchIteration = 0;


//============================================================ Function Prototypes ==================================================================
void frontRollerControl(RollerMode mode);
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
void randomsearch(void);

//============================================================ Helper Functions ==================================================================

void resetStatus() {
    status.isBall = false;
    status.isBallPicked = false;
    status.panRight = false;
    status.isBallDetectedFlag = false;
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
   
    // Read limit switches
    limitSwitches[0] = SensorValue[limitswitchLB];
    limitSwitches[1] = SensorValue[limitswitchRB];
    limitSwitches[2] = SensorValue[limitswitchBall];

    if (!panRightInitialized) {
        status.panRight = (limitSwitches[0] == 0) ? true : false;
        panRightInitialized = true;
    }


    // Read sharp sensors
    distances.distFC = getSharpDistance(sharpFC, SensorValue[sharpFC]);
    distances.distFR = getSharpDistance(sharpFR, SensorValue[sharpFR]);
    distances.distFL = getSharpDistance(sharpFL, SensorValue[sharpFL]);
    distances.distBC = getSharpDistance(sharpBC, SensorValue[sharpBC]);

    // Update status flags
    status.isBall = ( distances.distFL <= 50) || 
                    (distances.distFR <= 50);

    if (status.isBall) {
        status.isBallDetectedFlag = true;
    }

    status.isBallPicked = (limitSwitches[2] == 0);

// Only allow resetting isBallPicked during DELIVER state
if (currentState == DELIVER) {
    status.isBallPicked = (limitSwitches[2] == 0); 
     } else {
    // Once it's true, don't let it go false until DELIVER state
    status.isBallPicked = status.isBallPicked || (limitSwitches[2] == 0);
    }

}

int distanceToTicks(float distance) {
    return (int)(distance / DISTANCE_PER_TICK);
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
    float realDegrees = degrees * 3.1;
    float turningCircumference = PI * WHEEL_BASE;
    float arcLength = (realDegrees / 180.0) * turningCircumference;
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int reducedSpeed = 70;

    if (right) {
        motor[motorLeft] = (OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed);
        motor[motorRight] = -reducedSpeed;
    } else {
        motor[motorLeft] = -(OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed);
        motor[motorRight] = (reducedSpeed);
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}


void randomsearch() {
    int angle = 50 + rand()%(360-50+1);
    int direction = rand() % 2;
    turnDegrees(angle, direction);
    wait1Msec(1000);
    motor[motorLeft] = 60;
    motor[motorRight] = 60;
}

void searchingAlgo() {
    // Phase 1: Runs exactly once at startup
    if (!status.isfirstBallDelivered && searchIteration == 0) {
        searchIteration++;

        // 1st pan in 1st row 
        moveDistance(INITIAL_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, true);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, !true);
        wait1Msec(1000);

        // 2nd pan in 2nd row 
        moveDistance(ROW_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, true);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, !true);
        wait1Msec(1000);

        // 3rd pan in 3rd row
        moveDistance(ROW_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, true);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, !true);
        wait1Msec(1000);
    }
    else {
        randomsearch();
    }
}

void moveTowardsBall() {
    if (distances.distFL < distances.distFR) {
        // Ball is more to the left
        turnDegrees(20, false);
        wait1Msec(1000);
        motor[motorLeft] = 30;
        motor[motorRight] = 30;
    }
    else {
        // Ball is more to the right
        turnDegrees(20, true);
        wait1Msec(1000);
        motor[motorLeft] = 30;
        motor[motorRight] = 30;
    }
}

//============================================================ Task Definitions ==================================================================

task readSensorsTask() {
    while(true) {
        readSensors();
        wait1Msec(1000);
    }
}

task searchingBallTask() {
    while(true) {
        searchingAlgo();
        wait1Msec(100);
    }
}

task moveTowardsBallTask() {
        moveTowardsBall();
}

task startBallSecuring() {
    motor[BACK_ROLLER] = -50;  // Close the flapper
    frontRollerControl(OUTPUT); // Reverse the front roller to prevent extra balls from being picked up
    wait1Msec(300);             // Wait for 300 milliseconds
    motor[BACK_ROLLER] = 0;    // Stop the back roller
}

task startBallDispensing() {
    frontRollerControl(STOP);   // Stop the front roller
    motor[BACK_ROLLER] = -50;   // Reverse the back roller to dispense the ball
    wait1Msec(800);             
    //Open the flapper again
    motor[BACK_ROLLER] = 50;    // Move the back roller forward to ensure ball is fully dispensed
    wait1Msec(2000);            // Wait for 1000 milliseconds for the ball to be fully dispensed
    motor[BACK_ROLLER] = 0;     // Stop the back roller
}


//============================================================ Test Task ==================================================================
task testSensorModuleTask() {
    while(true) {
        // Display all sensor readings
        writeDebugStreamLine("\n--- SENSOR READINGS ---");
        writeDebugStreamLine("Limit Switches: LB=%d RB=%d Ball=%d",
                           limitSwitches[0], limitSwitches[1], limitSwitches[2]);
        writeDebugStreamLine("Distances: FC=%.1fcm FR=%.1fcm FL=%.1fcm BC=%.1fcm",
                           distances.distFC, distances.distFR, 
                           distances.distFL, distances.distBC);

        writeDebugStreamLine("\n--- STATUS FLAGS ---");
        writeDebugStreamLine("Ball Detected: %d | Ball Picked: %d | Pan Right: %d | First Delivered: %d",
                           status.isBall, status.isBallPicked, status.panRight, status.isfirstBallDelivered);
        
        // Additional status checks
        if (!status.isBallDetectedFlag) {
            writeDebugStreamLine("Status: Ball detected flag is not set.");
        }

        // Additional status checks
        if (status.isBallDetectedFlag) {
            writeDebugStreamLine("Status: Ball detected flag is set.");
        }


        // Display current robot state
        writeDebugStreamLine("Current State: %d (0=SEARCH, 1=COLLECT, 2=DELIVER)", currentState);
        
        wait1Msec(1000);  // Delay for 1 second before the next update
    }
}


//============================================================ Phase Functions ==================================================================

void searchPhase() {
    frontRollerControl(INTAKE); // Start the front roller to intake the ball
    startTask(searchingBallTask);

    while(currentState == SEARCH) {
        if(status.isBall || status.isBallDetectedFlag) {
            stopTask(searchingBallTask);
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            wait1Msec(100);
            currentState = COLLECT;
            break;
        }
        
        if(status.isBallPicked) {
            stopTask(searchingBallTask);
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            currentState= DELIVER;
            break;
        }

        wait1Msec(50);
    }
}


void collectPhase() {
    startTask(moveTowardsBallTask);  // Start the task to move towards the ball
    clearTimer(T1);  // Clear the timer

    while(currentState == COLLECT) {
        if (status.isBallPicked) {
            clearTimer(T1);  // Stop the timer when the ball is picked
            stopTask(moveTowardsBallTask);  // Stop the task
            motor[motorLeft] = 0;  
            motor[motorRight] = 0;
            startTask(startBallSecuring);  // Start securing the ball
            currentState = DELIVER;
            break;
        }

        // Timeout check - if more than 7 seconds without picking ball
        if (time1[T1] > 7000) {  // 7000 ms = 7 seconds
            clearTimer(T1);  // Stop the timer
            stopTask(moveTowardsBallTask);  // Stop moving towards the ball
            motor[motorLeft] = 0;  // Stop the motors
            motor[motorRight] = 0;
            currentState = SEARCH;  // Restart the search phase if no ball is picked
            break;
        }

        wait1Msec(50);  // Small delay to prevent overloading the CPU
    }
}


//============================================================ Main Task ==================================================================
task main() {

    // Clear the debug stream (no need for NXT LCD display setting)
    clearDebugStream();

    // Initialize all status flags to false
    resetStatus();

    //Make the flapper open to pick up the ball
    motor[BACK_ROLLER] = 50;
    wait1Msec(1000);
    motor[BACK_ROLLER] = 0;

    // Start the front roller to intake the ball
    frontRollerControl(INTAKE);


    // Initialize sensor reading task
    startTask(readSensorsTask);
    startTask(testSensorModuleTask); // for debugging purposes
    wait1Msec(500); // Allow sensors to stabilize

    while(true) {
        switch(currentState) {
            case SEARCH:
                resetStatus();
                searchPhase();
                break;
            case COLLECT:
                collectPhase();
                break;
            case DELIVER:
                motor[motorLeft] = 0;
                motor[motorRight] = 0;
                while(true) {
                	  stopTask(testSensorModuleTask); // for debugging purposes
                    writeDebugStreamLine("DELIVER MODE...");
                    if (!limitSwitches[0] && !limitSwitches[1]) {
                        writeDebugStreamLine("LIMITSWITCH IS PRESSED...");
                        startTask(startBallDispensing); // Start dispensing the ball
                        wait1Msec(100);
                    }
                }
                break;
        }
        wait1Msec(100);
    }
}