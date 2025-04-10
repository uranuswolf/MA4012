//=================================================================Pragma Configuration=================================================================
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

//================================================================= Sensor MACROS ==================================================================
#define sharpFC        in8
#define sharpFL        in2
#define sharpBC        in7
#define sharpFR        in1
#define IR_A           in6
#define IR_B           in5
#define IR_C           in4
#define IR_D           in3

#define limitswitchLB  dgtl6
#define limitswitchRB  dgtl5
#define limitswitchBall dgtl7

#define RIGHT_ENCODER  dgtl1
#define LEFT_ENCODER   dgtl3

#define compass_LSB    dgtl8
#define compass_Bit3   dgtl9
#define compass_Bit2   dgtl10
#define compass_MSB    dgtl11

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
const float INITIAL_DISTANCE = 1.2;
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
    bool isBoundary;
    bool isBall;
    bool isFrontObstacle;
    bool isBackObstacle;
    bool isBallPicked;
    bool reachedBase;
    bool isDelivered;
    bool isfirstBallDelivered; // Flag to check if the first ball is delivered
    bool panRight; // Flag to check if should pan left or right at the start of the search
    bool ballWithinSight; // Flag to check if the ball is within sight of the robot
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
    RETURN,
    DELIVER
} RobotState;

// ============================================================ Global Variables ==================================================================
DistanceSensors distances;
StatusFlags status;
RobotState currentState = SEARCH;


int limitSwitches[3];
int heading;
bool panRightInitialized = false;
int searchIteration = 0;
bool IR_A_value;
bool IR_B_value;
bool IR_C_value;
bool IR_D_value;


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
void returnPhase(void);
void decideTurn(float leftDist, float rightDist);
void handleObstacle(void);
void handleBoundary(void);
void returnToBase(void);
float compass(int heading);
void randomsearch(void);



//============================================================ Helper Functions ==================================================================

void resetStatus() {
    status.isBoundary = false;
    status.isBall = false;
    status.isFrontObstacle = false;
    status.isBackObstacle = false;
    status.isBallPicked = false;
    status.reachedBase = false;
    status.isDelivered = false;
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
    
IR_A_value = SensorValue[IR_A] < 2700 ? true : false;
IR_B_value = SensorValue[IR_B] < 2700 ? true : false;
IR_C_value = SensorValue[IR_C] < 2700 ? true : false;
IR_D_value = SensorValue[IR_D] < 2700 ? true : false;

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

// Read sharp sensors
distances.distFC = getSharpDistance(sharpFC, SensorValue[sharpFC]);
distances.distFR = getSharpDistance(sharpFR, SensorValue[sharpFR]);
distances.distFL = getSharpDistance(sharpFL, SensorValue[sharpFL]);
distances.distBC = getSharpDistance(sharpBC, SensorValue[sharpBC]);

// Update status flags
status.isBoundary = (IR_A_value || IR_B_value ||IR_C_value || IR_D_value);
status.isFrontObstacle = (distances.distFC < 30.0);
status.isBackObstacle = (distances.distBC < 30.0);

status.isBall = ((10 <= distances.distFL && distances.distFL <= 50) || 
                 (10 <= distances.distFR && distances.distFR <= 50)) &&
                (!status.isFrontObstacle);

// Only allow resetting isBallPicked during DELIVER state
if (currentState == DELIVER) {
status.isBallPicked = (limitSwitches[2] == 0); 
 } else {
// Once it's true, don't let it go false until DELIVER state
status.isBallPicked = status.isBallPicked || (limitSwitches[2] == 0);
}

if (currentState == RETURN && !status.reachedBase) {
    if ((limitSwitches[0] == 0 || limitSwitches[1] == 0) && // 0 means limit switch is pressed
        (IR_C_value == true && IR_D_value == true)) {
        status.reachedBase = true;
    }
}
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
    float realDegrees = degrees * 3.1;
    float turningCircumference = PI * WHEEL_BASE;
    float arcLength = (realDegrees / 180.0) * turningCircumference;
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int reducedSpeed = 127;

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

float compass(int heading){
	
	switch(heading){
	case 7: return 0; 		//W
		break;
	case 3: return 45; 		//SW
		break;
	case 11: return 90; 	//S
		break;
	case 9: return 135; 	//SE
		break;
	case 13: return 180;	//E
		break;
	case 12: return 225; 	//NE
		break;
	case 14: return 270; 	//W
		break;
	case 6: return 315; 	//NW
		break;
	}
	return -1;
}

void randomsearch(){
    int angle = 50 + rand()%(360-50+1);
    int direction= rand() %2;
    while(!status.isBoundary){
      turnDegrees(angle,direction);
      motor[motorLeft]=60;
      motor[motorRight]=60;
    }
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
        while(true){
            randomsearch();
        }
    }
}



void moveTowardsBall() {

    if (distances.distFL < distances.distFR) {
        // Ball is more to the left
        turnDegrees(20,false);
        wait1Msec(500);
        motor[motorLeft] =30;
        motor[motorRight] = 30;
    }
    else {
        // Ball is more to the right
        turnDegrees(20,true);
        wait1Msec(500);
        motor[motorLeft] =30;
        motor[motorRight] = 30;
    }
   
}

// Boundary Handling Function
void handleBoundary() {
    // Stop motors immediately 
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    

    // Combined special cases first
    if (IR_A_value && IR_C_value  && limitSwitches[0] && limitSwitches[1]) { 
    	  writeDebugStreamLine("boundary on right! turn left");
        turnDegrees(90,false);       // Turn left
        moveDistance(0.3,false);           // Move forward 30cm
    } else if (IR_B_value&& IR_D_value && limitSwitches[0] && limitSwitches[1]) {
        writeDebugStreamLine("boundary on left! turn right");
        turnDegrees(90,true);       // Turn right
        moveDistance(0.3,false);           // Move forward 30cm
    } else if ((IR_A_value || IR_B_value) && limitSwitches[0] && limitSwitches[1]){
        writeDebugStreamLine("boundary in front! turn back");
        moveDistance(0.2,true);    // Move back 10cm
        turnDegrees(180,true);     //Reverse
    } else if ((IR_C_value || IR_D_value) && limitSwitches[0] && limitSwitches[1]){
        writeDebugStreamLine("boundary in back! move front");
        moveDistance(0.3,false); // Move forward 30cm
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
        if (distances.distFC <= 15) {
            moveDistance(0.2, true);  // Reverse 20cm away from the obstacle
        }
        
        wait1Msec(1500);  // Let the background task update sensor readings

        // Recheck if the front obstacle is still present
        if (status.isFrontObstacle) {
            decideTurn(distances.distFL, distances.distFR);
            moveDistance(0.2,false);  // Move forward slightly after turn
        }
        return;
    } 
    
    // Handle back obstacles if reversing
    else if (status.isBackObstacle && motor[motorLeft] < 0 && motor[motorRight] < 0) {
        if (distances.distBC <= 15) {
            moveDistance(0.2,false);  // Move forward 20cm
        }

        wait1Msec(1500);  // Allow time for sensor update

        // Recheck if the back obstacle is still present
        if (status.isBackObstacle) {
            decideTurn(distances.distFL, distances.distFR);
            moveDistance(0.2,false);  // Move forward slightly after turn
        }
        return;
    } 
}


void returnToBase() {
    //HOME_BASE_HEADING = 180;
    while (!compass(heading) == 180) {
        int reducedSpeed = 60; // Adjust speed as needed
        motor[motorLeft] = (OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed);
        motor[motorRight] = -reducedSpeed;
        wait1Msec(500);
            break;
        }

        // Move backward continuously
        motor[motorLeft] = -127;
        motor[motorRight] = -127;
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

task roller_indexer() {
        //run this conce
        motor[BACK_ROLLER] = 50;
        wait1Msec(1000);
        motor[BACK_ROLLER] = 0;

        while (true) {
         //secure ball
         waitUntil(status.isBallPicked);
         motor[BACK_ROLLER] = -50; //close the flapper 
         motor[FRONT_ROLLER] = -127;
         wait1Msec(300);
         motor[BACK_ROLLER] = 0;
       
         //dispense ball
         waitUntil(status.reachedBase);
         motor[FRONT_ROLLER] = 0;
         motor[BACK_ROLLER] = -50;
         wait1Msec(800);
         motor[BACK_ROLLER] = 50;
         wait1Msec(1000);
         motor[BACK_ROLLER] = 0;
        
        wait1Msec(100);
        }
    }

//============================================================ Test Task ==================================================================
// This task is for debugging purposes only

task testSensorModuleTask() {
     while(true) {
        // Display all sensor readings
        writeDebugStreamLine("\n--- SENSOR READINGS ---");
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
    
            
            wait1Msec(1000); // Reduced delay for more frequent updates
        }
    }
    
//============================================================ Phase Functions ==================================================================

void searchPhase() {
    startTask(searchingBallTask); // Start searching for the ball

    while(currentState == SEARCH) {

        // Step 1: Handle boundary first
        if (status.isBoundary) {
            stopTask(searchingBallTask); 
            handleBoundary();
            wait1Msec(500); 
            startTask(searchingBallTask); // Restart searching after handling obstacle
            continue;       // restart loop
        }

        // If an obstacle is detected, handle it
        if(status.isFrontObstacle || status.isBackObstacle) {
            stopTask(searchingBallTask); 
            handleObstacle(); 
            wait1Msec(500); 
            startTask(searchingBallTask); // Restart searching after handling obstacle
            continue;       // restart loop
        }

        if(status.isBall) {
            stopTask(searchingBallTask); // Stop searching when ball is found
            motor[motorLeft] =0;
            motor[motorRight] =0;
            wait1Msec(100); // Wait for a moment to stabilize
            currentState = COLLECT;
            break;
        }
        
        if(status.isBallPicked) {  // Bypass collectPhase and jump straight to RETURN if ball is picked up by luck
            stopTask(searchingBallTask); // Stop searching if ball is picked
            currentState = RETURN;
            break;
        }

        wait1Msec(50); 
    }
}


void collectPhase() {
    startTask(moveTowardsBallTask);
    clearTimer(T1);  // Reset timer T1 at the beginning
    
    while(currentState == COLLECT) {
        // Step 1: Handle boundary first
        if (status.isBoundary) {
            stopTask(moveTowardsBallTask); 
            handleBoundary();
            wait1Msec(500); 
            currentState = SEARCH; // Go back to search state
            break;   
        }

        // If an obstacle is detected, handle it
        if(status.isFrontObstacle || status.isBackObstacle) {
            stopTask(moveTowardsBallTask); 
            handleObstacle(); // Stop search and handle the obstacle
            wait1Msec(500); 
            currentState = SEARCH; // Go back to search state
            break;      
        }

        if (status.isBallPicked){
            clearTimer(T1);  // Reset timer T1 
            stopTask(moveTowardsBallTask);
            currentState = RETURN;
            break;
        }

         // Timeout check - if more than 7 seconds without any reset conditions
         if (time1[T1] > 7000) {
            stopTask(moveTowardsBallTask);
            currentState = SEARCH;
            break;
        }
        wait1Msec(50);
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
        wait1Msec(50);
}
}

void deliverPhase() {
   while(true){

        if (!status.isBallPicked) {
            currentState = SEARCH; // Go back to search state
            break;
        }
        wait1Msec(50);
    }
}

//============================================================ Main Task ==================================================================
task main() {
    // Initialize all status flags to false
    resetStatus();

    //Start the front roller to intake the ball 
    frontRollerControl(INTAKE);

    // Start the roller indexer task
    startTask(roller_indexer); 

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