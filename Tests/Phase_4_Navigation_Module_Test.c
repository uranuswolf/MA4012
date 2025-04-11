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
const int BASE_POWER = 45;
const float DISTANCE_CORRECTION_FACTOR = 3.85;
const float OFFSET_POWER_FOR_LEFT_MOTOR = 1.28;
const int ROLLER_SPEED = 127;
const float PAN_ANGLE = 10;
const float INITIAL_DISTANCE = 1;
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
    bool isfirstBallDelivered; 
    bool panRight; 
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
void deliverPhase(void);

void decideTurn(float leftDist, float rightDist);
void handleObstacle(void);
void handleBoundary(void);
void returnToBase(void);
float compass(int heading);
void randomsearch(void);




//============================================================ Helper Functions ==================================================================

void resetStatus() {
    status.isBall = false;
    status.isFrontObstacle = false;
    status.isBackObstacle = false;
    status.isBallPicked = false;
    status.reachedBase = false;
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

    // Read IR sensors   
    IR_A_value = SensorValue[IR_A] < 2700 ? true : false;
    IR_B_value = SensorValue[IR_B] < 2700 ? true : false;
    IR_C_value = SensorValue[IR_C] < 2700 ? true : false;
    IR_D_value = SensorValue[IR_D] < 2700 ? true : false;

    // Read limit switches
    limitSwitches[0] = SensorValue[limitswitchLB];
    limitSwitches[1] = SensorValue[limitswitchRB];
    limitSwitches[2] = SensorValue[limitswitchBall];

    if (!panRightInitialized) {
        status.panRight = (limitSwitches[0] == 0) ? true : false; 
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
    status.isFrontObstacle = (distances.distFC < 40.0);
    status.isBackObstacle = (distances.distBC < 40.0);

    status.isBall = ((distances.distFL <= 50) || (distances.distFR <= 50)) &&
    (!status.isFrontObstacle);

    
    if (status.isBall && !status.isBallDetectedFlag) {
    status.isBallDetectedFlag = true;
    }
    
    // Only update based on the switch inside the DELIVER check
    if (currentState == DELIVER) {
        status.isBallPicked = (limitSwitches[2] == 0); 
    } else {
        writeDebugStreamLine("Ball picked: %d", limitSwitches[2]);
        status.isBallPicked = status.isBallPicked || (limitSwitches[2] == 0);
    }

    if (currentState == RETURN && !status.reachedBase) {
        if ((!limitSwitches[0] || !limitSwitches[1]) && // 0 means limit switch is pressed
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

    int reducedSpeed = 50;

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

void randomsearch() {
    int angle = 50 + rand()%(360-50+1);
    int direction = rand() % 2;
    turnDegrees(angle/2, direction);
    wait1Msec(500);
    turnDegrees(angle/2, direction);
    wait1Msec(500);

    motor[motorLeft] = 30;
    motor[motorRight] = 30;

}

void searchingAlgo() {

    // Phase 1: Runs exactly once at startup (before first delivery)
    if ( searchIteration == 0) {
        searchIteration++;  // Increment here to ensure Phase 1 runs ONLY ONCE

        // 1st pan in 1st row 
        moveDistance(INITIAL_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees((3*PAN_ANGLE), !status.panRight);
        wait1Msec(500);
       

        // 2nd pan in 2nd row 
        moveDistance(ROW_DISTANCE);
        wait1Msec(1000);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees(PAN_ANGLE, status.panRight);
        wait1Msec(500);
        turnDegrees((3*PAN_ANGLE), !status.panRight);
        wait1Msec(500);
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


// Boundary Handling Function
void handleBoundary() {
    
    if (status.reachedBase) return;

    motor[motorLeft] = -(127*OFFSET_POWER_FOR_LEFT_MOTOR);
    motor[motorRight] = -127;
    wait1Msec(100);
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    

    // Combined special cases first
    if (IR_A_value && IR_C_value  && limitSwitches[0] && limitSwitches[1]) { //Limit Switches not pressed
    	writeDebugStreamLine("boundary on right! turn left");
        turnDegrees(90,false);       // Turn left
        moveDistance(0.3,false);     // Move forward 30cm
        return;
    } if (IR_B_value && IR_D_value && limitSwitches[0] && limitSwitches[1]) {
        writeDebugStreamLine("boundary on left! turn right");
        turnDegrees(90,true);       // Turn right
        moveDistance(0.3,false);    // Move forward 30cm
        return;
    } if ((IR_A_value || IR_B_value) && limitSwitches[0] && limitSwitches[1]){
        writeDebugStreamLine("boundary in front! turn back");
	    if (IR_A_value && !IR_B_value){
	       turnDegrees(75,false); //turn left
           moveDistance(0.3,false); // Move forward 30cm
           return;
	   
        }if (IR_B_value && !IR_A_value){
	       turnDegrees(75,true);
	       moveDistance(0.3,false);
           return;
	    }if(IR_B_value && IR_A_value){
            moveDistance(0.3,true);
	       turnDegrees(180,false);
           return;
	    }
    } else if ((IR_C_value || IR_D_value) && limitSwitches[0] && limitSwitches[1]){
        writeDebugStreamLine("boundary in back! move front");
	    if (IR_C_value && !IR_D_value){
            moveDistance(0.3,false); // Move forward 30cm
            turnDegrees(35,true); 
            return;
        
         }if (IR_D_value && !IR_C_value){
            moveDistance(0.3,false);
            turnDegrees(35,false); //turn left
            return;
         }if(IR_C_value && IR_D_value){
             moveDistance(0.3,true);
            return;
         }
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

    motor[motorLeft] = -(127*OFFSET_POWER_FOR_LEFT_MOTOR);
    motor[motorRight] = -127;
    wait1Msec(100);
    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    
    // Handle front obstacles if moving forward
    if (status.isFrontObstacle) {
        if (distances.distFC <= 25) {
            moveDistance(0.2, true);  // Reverse 20cm away from the obstacle
        }
        
        wait1Msec(1500);  // Let the background task update sensor readings

        // Recheck if the front obstacle is still present
        if (status.isFrontObstacle) {
            decideTurn(distances.distFL, distances.distFR);
            moveDistance(0.2,false);  // Move forward slightly after turn
            return;
        }
        return;
    } 
    
    // Handle back obstacles if reversing
    else if (status.isBackObstacle && motor[motorLeft] < 0 && motor[motorRight] < 0) {
        if (distances.distBC <= 25) {
            writeDebugStreamLine("HANDLING BACK OBSTACLE NOW");
            moveDistance(0.2,true);  // Move forward 20cm
        }

        wait1Msec(1500);  // Allow time for sensor update

        // Recheck if the back obstacle is still present
        if (status.isBackObstacle) {
            writeDebugStreamLine("HANDLING BACK OBSTACLE NOWWWWWWWWWW");
            decideTurn(distances.distFL, distances.distFR);
            moveDistance(0.2,true);  // Move forward slightly after turn
        }
        return;
    } 
}


void returnToBase() {
    //HOME_BASE_HEADING = 180;
	int reducedSpeed = 35; // Adjust speed as needed
    while(!(compass(heading) == 180)) {
        writeDebugStreamLine("TURNING TO THE REQUIRED HOMEBASED HEADING");
        motor[motorLeft] = (OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed);
        motor[motorRight] = -reducedSpeed;
        wait1Msec(50);
        }

        // Move backward continuously
        motor[motorLeft] = -(OFFSET_POWER_FOR_LEFT_MOTOR*reducedSpeed*0.8);
        motor[motorRight] = -(reducedSpeed*0.8);
    }



//============================================================ Task Definitions ==================================================================

task readSensorsTask() {
    while(true) {
        readSensors();
        wait1Msec(5); 
    }
}

task searchingBallTask() {
    searchingAlgo(); //the second time this function is called, then it will be in the else statement
}

task moveTowardsBallTask() {
    moveTowardsBall();
}

task returnToBaseTask(){
        returnToBase();
    }

task startBallSecuring() {
        motor[BACK_ROLLER] = -50;  // Close the flapper
        frontRollerControl(OUTPUT); // Reverse the front roller to prevent extra balls from being picked up
        wait1Msec(200);             // Wait for 300 milliseconds
        motor[BACK_ROLLER] = 0;    // Stop the back roller
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
        writeDebugStreamLine("Ball Picked: %d | Pan Right: %d",
                               status.isBallPicked, status.panRight);
        writeDebugStreamLine("Raw IR: A=%d B=%d C=%d D=%d", 
        SensorValue[IR_A], SensorValue[IR_B], SensorValue[IR_C], SensorValue[IR_D]);
    
        // Additional status checks
        if (!status.isBallDetectedFlag) {
            writeDebugStreamLine("Status: Ball detected flag is not set.");
        }

        // Additional status checks
        if (status.isBallDetectedFlag) {
            writeDebugStreamLine("Status: Ball detected flag is set.");
        }


        // Display current robot state
        writeDebugStreamLine("Current State: %d (0=SEARCH, 1=COLLECT, 2=RETURN , 3=DELIVER)", currentState);    
        
        wait1Msec(60); // Reduced delay for more frequent updates
        }
    }
    
//============================================================ Phase Functions ==================================================================

void searchPhase() {
    frontRollerControl(INTAKE); // Start the front roller to intake the ball
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

        if(status.isBall || status.isBallDetectedFlag) {
            writeDebugStreamLine("ROBOT SEES A BALL");
            stopTask(searchingBallTask);
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            wait1Msec(1000);
            currentState = COLLECT;
            break;
        }
        
        if(status.isBallPicked) {
            stopTask(searchingBallTask);
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            startTask(startBallSecuring);
            currentState= RETURN;
            break;
        }

        wait1Msec(60); 
    }
}


void collectPhase() {
    writeDebugStreamLine("ROBOT IS MOVING TOWARDS A BALL");
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
            stopTask(moveTowardsBallTask); // Stop moving towards the ball
            motor[motorLeft] = 0;  
            motor[motorRight] = 0;
            startTask(startBallSecuring); // Start securing the ball
            currentState = RETURN;
            break;
        }

          // Timeout check - if more than 6 seconds without picking ball
        if (time1[T1] > 6000) {  // 6 seconds
            writeDebugStreamLine("Timeout: No ball picked in 6 seconds.");
            clearTimer(T1);  // Stop the timer
            stopTask(moveTowardsBallTask);  // Stop moving towards the ball
            motor[motorLeft] = 0;  
            motor[motorRight] = 0;
            currentState = SEARCH;  // Restart the search phase if no ball is picked
            break;
        }

        wait1Msec(60);
    }
}

void returnPhase() {
    startTask(returnToBaseTask);
    while(currentState == RETURN) {
        // Only handle boundaries if we haven't reached base yet
        if (!status.reachedBase) {
            if (status.isBoundary) {
                stopTask(returnToBaseTask); 
                handleBoundary();
                wait1Msec(500);
                startTask(returnToBaseTask);
                continue;
            }
            
            if(status.isFrontObstacle || status.isBackObstacle) {
                stopTask(returnToBaseTask); 
                handleObstacle();
                wait1Msec(500);
                startTask(returnToBaseTask);
                continue;
            }
        }
      
        if (status.reachedBase) {
            stopTask(returnToBaseTask); 
            frontRollerControl(STOP);   // Stop the front roller
            motor[motorLeft] = -20;
            motor[motorRight] = -20;
            wait1Msec(300);
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            
            // Additional verification
            wait1Msec(10);  // Let robot settle
            
            // Double-check we're still at base
            if (!limitSwitches[0] ||!limitSwitches[1] && 
                IR_C_value && IR_D_value) {
                currentState = DELIVER;
            } else {
                status.reachedBase = false;  // Reset if not properly aligned
                startTask(returnToBaseTask); // Try again
            }
            break;  
        }
        wait1Msec(60);
    }
}

void deliverPhase() {

    motor[BACK_ROLLER] = -50;   // Reverse the back roller to dispense the ball
    wait1Msec(800);             
    //Open the flapper again
    motor[BACK_ROLLER] = 50;    // Move the back roller forward to ensure ball is fully dispensed
    wait1Msec(2000);            // Wait for 1000 milliseconds for the ball to be fully dispensed
    motor[BACK_ROLLER] = 0;     // Stop the back roller
    frontRollerControl(INTAKE);
    moveDistance(1.2,false);
    wait1Msec(1000);
    currentState = SEARCH;
    return;
          
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

    //Start the front roller to intake the ball 
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
