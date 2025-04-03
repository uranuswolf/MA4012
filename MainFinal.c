// ================================================================== pragma configuration ==================================================================
#pragma config(Sensor, in1,    sharpTC,        sensorAnalog)
#pragma config(Sensor, in2,    sharpBL,        sensorAnalog)
#pragma config(Sensor, in3,    sharpBC,        sensorAnalog)
#pragma config(Sensor, in4,    sharpBR,        sensorAnalog)
#pragma config(Sensor, in5,    IR_A,           sensorAnalog)
#pragma config(Sensor, in6,    IR_B,           sensorAnalog)
#pragma config(Sensor, in7,    IR_C,           sensorAnalog)
#pragma config(Sensor, in8,    IR_D,           sensorAnalog)
#pragma config(Sensor, dgtl1,  limitswitchLB,  sensorDigitalIn)
#pragma config(Sensor, dgtl2,  limitswitchRB,  sensorDigitalIn)
#pragma config(Sensor, dgtl3,  limitswitchBall,sensorDigitalIn)
#pragma config(Sensor, dgtl4,  RIGHT_ENCODER,  sensorQuadEncoder) 
#pragma config(Sensor, dgtl6,  LEFT_ENCODER,   sensorQuadEncoder) 
#pragma config(Sensor, dgtl10, compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl8,  compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl7,  compass_MSB,    sensorDigitalIn)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,  FRONT_ROLLER,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10, BACK_ROLLER,    tmotorVex393_MC29, openLoop)


// ================================================================== Function prototypes ==================================================================
// **************************Khirdir please update the function prototypes as needed**************************
void moveDistance(float distance, bool backward = false); 
void turnDegrees(float degrees, bool right = false);
int distanceToTicks(float distance);
void searchPhase(void);
void collectPhase(void);
void returnPhase(void);
void deliverPhase(void);
void scanBall(void);
void scanBoundary(void);
void scanObstacle(void);
void returnToBase(void);
void deliver(void);
void pickUpBall(void);
void moveTowardsBall(void);
void handleBoundary(void);
void handleObstacle(void);
void readIR(void);
void readDistanceSensor(void);
void scanSequenceLeft(void);
void scanSequenceRight(void);
void checkBoundary(void);
void frontRollerIntake(void);
void frontRollerStop(void);
void frontRollerOutput(void);
void FlapperPush(void);
void FlapperStop(void);
int compass(void);
void readlimitswitch(void);
void searchingAlgoLeft(void);
void searchingAlgoRight(void); 
void searchingAlgo(void);
task searchAlgoTask(void);
task scanBallTask(void);
task scanBoundaryTask(void);
task scanObstacleTask(void);
task returnToBaseTask(void);
task deliverTask(void);
task readIRTask(void);
task readDistanceSensorTask(void);
task releaseExtraBallsTask(void);
task check_current_headingTask(void);
task moveTowardsBallTask(void);
task pickUpBallTask(void);




// ================================================================== Define Robot states  ==================================================================
typedef enum {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
  } RobotState;


// ================================================================== Define MACROS ==================================================================
#define PI 3.14159265359

// ================================================================== Global variables ==================================================================
RobotState currentState = SEARCH;  // Start with the 'Search' phase
int IR_A = 1; // IR sensor A = 0 or 1, 0 means at boundary, 1 means not at boundary, 
int IR_B = 1;// IR sensor B = 0 or 1
int IR_C = 1;// IR sensor C = 0 or 1
int IR_D = 1;// IR sensor D = 0 or 1
bool isBoundary = false;  // Flag to indicate if the robot is at the boundary
bool isBall = false;  // Flag to indicate if the ball is detected
bool isObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBallPicked = false;  // Flag to indicate if the ball is picked up
bool reachedBase = false;  // Flag to indicate if the robot has reached the base
bool isDelivered = false;  // Flag to indicate if the ball is delivered
bool leftScanBoundary = false; //Flag to indicate if the left boundary is detected
bool rightScanBoundary = false; //Flag to indicate if the right boundary is detected
int heading ; // Variable to store the compass heading
int limitswitchLB = 0; // Variable to store the value of the left limit switch
int limitswitchRB = 0; // Variable to store the value of the right limit switch
int limitswitchBall = 0; // Variable to store the value of the ball limit switc
// **************************Khirdir please update the Global variables as needed**************************

// Mutex for shared variables
TMutex mutex;


// ================================================================== Define constants ==================================================================
const int basePower = 50;  // Power level for the base motor
const float wheelDiameter = 0.06985; // meters
const float wheelCircumference = wheelDiameter * PI; // meters
const int ticksPerRevolution = 90; // encoder ticks per revolution
const float distancePerTick = wheelCircumference / ticksPerRevolution; // meters per tick
const float wheelBase = 0.188; // distance between wheels (meters)
const int rollerSpeed = 127; // speed of the roller motor
const int MAX_DISTANCE = 300; // 300cm max distance for return, CHECK AGAIN 
// **************************Khirdir please update the constants as needed**************************


// ================================================================== Phase functions ==================================================================
void searchPhase(void) {
    startTask(searchAlgoTask);  // Start the search algorithm task
    startTask(scanBallTask);    // Start the scan ball task
    startTask(scanBoundaryTask);  // Start the scan boundary task
    startTask(scanObstacleTask);  // Start the scan Obstacle task
    
    // Keep checking for ball,boundary and obstacles in this phase, if not, keep doing the above 4 tasks concurrently
    while (true) {
      if (isBall) {
        stopTask(searchAlgoTask); // Stop the search algorithm task
        stopTask(scanBallTask);   // Stop the scan ball task
        stopTask(scanBoundaryTask);  // Start the scan boundary task
        stopTask(scanObstacleTask);  // Start the scan Obstacle task
        isBall = false;  // Reset the flag
        currentState = COLLECT;  // Change to collect phase when the ball is detected
        break;  // Exit the while loop
      }
      else if (isBoundary) {
        stopTask(searchAlgoTask);  // Stop the search algorithm task
        stopTask(scanBallTask);  // Stop the scan obstacle task
        stopTask(scanBoundaryTask);  // Stop the scan boundary task
        stopTask(scanObstacleTask);  // Stop the scan obstacle task
        
        handleBoundary();  // Handle boundary if the robot reaches the boundary during search

        //resume normal tasks in the search phase
        isBoundary = false;  // Reset the flag
        startTask(searchAlgoTask);  // Start the search algorithm task
        startTask(scanBallTask);    // Start the scan ball task
        startTask(scanBoundaryTask);  // Start the scan boundary task
        startTask(scanObstacleTask);  // Start the scan Obstacle task

      }

      else if (isObstacle) { //if an obstacle is detected
        stopTask(searchAlgoTask);  // Stop the search algorithm task
        stopTask(scanBallTask);  // Stop the scan obstacle task
        stopTask(scanBoundaryTask);  // Stop the scan boundary task
        stopTask(scanObstacleTask);  // Stop the scan obstacle task

        handleObstacle();  // Handle the obstacle 

        //resume normal tasks in the search phase
        isObstacle = false;  // Reset the flag
        startTask(searchAlgoTask);  // Start the search algorithm task
        startTask(scanBallTask);    // Start the scan ball task
        startTask(scanBoundaryTask);  // Start the scan boundary task
        startTask(scanObstacleTask);  // Start the scan Obstacle task
      } 

      wait1Msec(100);  // Small delay to prevent locking up the CPU, represent how often is the checking

    }
  }

void collectPhase(void) {
    startTask(scanBoundaryTask);  // Start the scan boundary task
    startTask(scanObstacleTask);  // Start the scan Obstacle task
    startTask(moveTowardsBallTask);  // Start the move towards ball task
    startTask(pickUpBallTask);       // Start the pick up ball task, means the robot can pickup any ball as it moves towards the target ball
    
    // Keep checking for boundary and obstacles and if ball is picked in this phase, if not, keep doing the above 2 tasks concurrently
    while(true) {
        
        if (isBoundary) {
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            stopTask(moveTowardsBallTask);  // Stop the move towards ball task
            stopTask(pickUpBallTask);       // Stop the pick up ball task

            handleBoundary();  // Handle boundary if the robot reaches the boundary during search

             //resume normal tasks in the collect phase
            isBoundary = false;  // Reset the flag
            startTask(scanBoundaryTask);  // Start the scan boundary task
            startTask(scanObstacleTask);  // Start the scan Obstacle task
            startTask(moveTowardsBallTask);  // Start the move towards ball task
            startTask(pickUpBallTask);       // Start the pick up ball task
          }
    
        else if (isObstacle) { //if an obstacle is detected
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            stopTask(moveTowardsBallTask);  // Stop the move towards ball task
            stopTask(pickUpBallTask);       // Stop the pick up ball task
            
            handleObstacle();  // Handle the obstacle

            //resume normal tasks in the collect phase
            isObstacle = false;  // Reset the flag
            startTask(scanBoundaryTask);  // Start the scan boundary task
            startTask(scanObstacleTask);  // Start the scan Obstacle task 
            startTask(moveTowardsBallTask);  // Start the move towards ball task
            startTask(pickUpBallTask);      // Start the pick up ball task
          }
        else if (isBallPicked) {  // Check if the ball has been successfully picked up
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            stopTask(moveTowardsBallTask);  // Stop the move towards ball task
            stopTask(pickUpBallTask);       // Stop the pick up ball task
            currentState = RETURN;  // Change to return phase when the ball is picked
            isBallPicked = false;  // Reset the flag
            break;  // Exit the while loop
        }

        wait1Msec(100);  // Small delay to prevent locking up the CPU, represent how often is the checking 
    }
}

void returnPhase(void) {
        startTask(scanBoundaryTask);  // Start the scan boundary task
        startTask(scanObstacleTask);  // Start the scan Obstacle task
        startTask(returnToBaseTask);  // Start the return to base task
        startTask(releaseExtraBallsTask);  // Start the release extra balls task, continuously, so prevent ball from rolling inside the robot while robot is moving back to base
        
         // Keep checking for boundary and obstacles and if robot successfully return to base in this phase, if not, keep doing the above 3 tasks concurrently
        while(true) {
        
            if (isBoundary) {
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                stopTask(returnToBaseTask); // Stop the return to base task
                stopTask(releaseExtraBallsTask);  // Stop the release extra balls task

                handleBoundary();  // Handle boundary if the robot reaches the boundary during search

                //resume normal tasks in the return phase
                isBoundary = false;  // Reset the flag
                startTask(scanBoundaryTask);  // Start the scan boundary task
                startTask(scanObstacleTask);  // Start the scan Obstacle task
                startTask(returnToBaseTask);  // Start the return to base task
                startTask(releaseExtraBallsTask);  // Start the release extra balls task
              }
        
            else if (isObstacle) { //if an obstacle is detected
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                stopTask(returnToBaseTask); // Stop the return to base task
                stopTask(releaseExtraBallsTask);  // Stop the release extra balls task

                handleObstacle();  // Handle the obstacle 

                //resume normal tasks in the return phase
                isObstacle = false;  // Reset the flag
                startTask(scanBoundaryTask);  // Start the scan boundary task
                startTask(scanObstacleTask);  // Start the scan Obstacle task
                startTask(returnToBaseTask);  // Start the return to base task
                startTask(releaseExtraBallsTask);  // Start the release extra balls task
              }
            else if (reachedBase) {  // Check if robot has reached the base
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                stopTask(returnToBaseTask);  // stop the return to base task
                stopTask(releaseExtraBallsTask);  // Stop the release extra balls task
                currentState = DELIVER;  // Change to deliver phase when robot reaches the base
                reachedBase = false;  // Reset the flag
                break;  // Exit the while loop
            }
    
            wait1Msec(100);  // Small delay to prevent locking up the CPU, represent how often is the checking 
        }
      
      }

void deliverPhase(void) {
        startTask(deliverTask);  // Start deliver task
        
        while(true){
            if (isDelivered) {  // Keep checking if the ball has been successfully delivered, means top limit switch is no longer triggered
                stopTask(deliverTask);  // Stop the deliver task if it has delivered
                currentState = SEARCH;  // Change to search phase after delivery
                isDelivered = false;  // Reset the flag
                break;  // Exit the while loop
            }
            wait1Msec(100);  // Small delay to prevent locking up the CPU, represent how often is the checking 
        }
       
      }

// ================================================================== Put all the functions here  ==================================================================

// **************************Khirdir please update the functions as needed**************************
void frontRollerIntake(void){
  motor[FRONT_ROLLER] = -rollerSpeed; 
}       

void frontRollerStop(void){
  motor[FRONT_ROLLER] = 0; 
}

void frontRollerOutput(void){
  motor[FRONT_ROLLER] = rollerSpeed; //REVERSE DIRECTION TO RELEASE EXTRA BALLS 
}     

void FlapperPush(void){
  motor[BACK_ROLLER] = -rollerSpeed; 
}       

void FlapperStop(void){
  motor[BACK_ROLLER] = 0; 
}

int compass(void){
	int num;
	num = SensorValue[compass_MSB]*pow(2,3) + SensorValue[compass_Bit2]*pow(2,2) + SensorValue[compass_Bit3]*2 + SensorValue[compass_LSB];
	
  switch(num){
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
	case 14: return 270; 	//N
		break;
	case 6: return 315; 	//NW
		break;
	}
	return -1;
}

void readIR(void) {
  while(true) {
      AcquireMutex(mutex);
      IR_A = SensorValue[IR_A] < 300 ? 0 : 1; // Threshold for detection is 300 
      IR_B = SensorValue[IR_B] < 300 ? 0 : 1; 
      IR_C = SensorValue[IR_C] < 300 ? 0 : 1;
      IR_D = SensorValue[IR_D] < 300 ? 0 : 1;
      ReleaseMutex(mutex);
      
      wait1Msec(50);
  }
}

void readlimitswitch(void) {
  while(true) {
      AcquireMutex(mutex);
      limitswitchLB = SensorValue[limitswitchLB];
      limitswitchRB = SensorValue[limitswitchRB];
      limitswitchBall = SensorValue[limitswitchBall];
      ReleaseMutex(mutex);
      
      wait1Msec(50);
  }
}

void scanBoundary(void) {
  if ((IR_A == 0 && IR_B == 0) || (IR_A == 0 && IR_C == 0) || (IR_A == 0 && IR_D == 0) || 
      (IR_B == 0 && IR_C == 0) || (IR_B == 0 && IR_D == 0) || (IR_C == 0 && IR_D == 0)) {
      AcquireMutex(mutex);
      isBoundary = true;
      ReleaseMutex(mutex);
  }
}

void handleBoundary(void){

    int IR_State = 0;

    if (IR_A == 0 && IR_B == 0 && IR_C == 1 && IR_D == 1) {
        IR_State = 1;  // Represents case 1
    } else if (IR_A == 0 && IR_B == 1 && IR_C == 0 && IR_D == 0) {
        IR_State = 2;  // Represents case 2
    } else if (IR_A == 1 && IR_B == 0 && IR_C == 1 && IR_D == 0) {
        IR_State = 3;  // Represents case 3
    } else if (IR_A == 1 && IR_B == 1 && IR_C == 0 && IR_D == 0) {
        IR_State = 4;  // Represents case 4
    }
    
    switch (IR_State) {
        case 1:
            // move backwards for 0.5m and make a 180 degrees turn
            moveDistance(0.5, true);
            turnDegrees(180);
            break;
        
        case 2:
            // turn left 90 degrees
            turnDegrees(90);
            break;
        
        case 3:
            // turn right 90 degrees
            turnDegrees(90, true);
            break;
    
        case 4:
            // move forward for 0.5m
            moveDistance(0.5);
            break;
        
        default: //This provides a reasonable recovery behavior when the boundary detection isn't perfectly clear.
            moveDistance(0.3, true);  // Back up slightly
            turnDegrees(45);          // Turn left 45 degrees
            break;
    }
    
}

void readDistanceSensor(){
    // Implement logic here to read the distance sensor 
    // ****************************Khirdir please update this function as needed*******************************
}

void scanObstacle(){
    startTask(readDistanceSensor);  // Start the read IR task 
    // Implement logic here
    // ****************************Khirdir please update this function as needed*******************************
    // isObstacle = true if an obstacle is detected
}

void handleObstacle(){
    // Implement logic here to handle the obstacle
    // ****************************Khirdir please update this function as needed*******************************
}

void scanBall(){
    startTask(readDistanceSensor);  // Start the read IR task 
    // Implement logic here to scan for the ball
    // ****************************Khirdir please update this function as needed*******************************
    //  isBall = true if the ball is detected
}

//void returnToBase(void){
    //startTask(check_current_headingTask); // Start the reading of the compass heading
    // Implement the return to base algorithm here
    // use simple movement function + compass
    //float degree = abs(270 - heading);
    //turnDegrees(degree, bool right = true); //turn
    //moveDistance(300, bool backward = true);
    //if(heading == 270 && (limitswitchLB == 0 || limitswitchRB == 0) && (IR_C == 0 && IR_D == 0){//condition to check if the robot has reached the base, make use of the limit switches and other indicators 
    //reachedBase = true;
//}
//}

void returnToBase(void) {
  float degree = abs(270 - heading);
  turnDegrees(degree, true);
  moveDistance(MAX_DISTANCE/100.0, true); // Convert cm to meters
  
  if(heading == 270 && (limitswitchLB == 1 || limitswitchRB == 1) && (IR_C == 0 && IR_D == 0)) { //checking that the robot is back is facing the deliver
      AcquireMutex(mutex);
      reachedBase = true;
      ReleaseMutex(mutex);
  }
}

void deliver(){
    // Implement the deliver algorithm here
    if //condition to check if the ball has been delivered, make use of the limit switches and other indicators
    // ****************************Yu Shun and KC please update this function as needed*******************************
    return isDelivered = true;
}

void pickUpBall(void) {
  frontRollerIntake();
  while (true) {
      if (limitswitchBall == 1) {
          frontRollerStop();
          AcquireMutex(mutex);
          isBallPicked = true;
          ReleaseMutex(mutex);
          break;
      }
      wait1Msec(100);
  }
}    

void moveTowardsBall(void){  
   //implement logic to move towards the ball after detecting it
   // ****************************Khirdir please update this function as needed*******************************
}  

// Convert desired distance in meters to encoder ticks
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

// Move forward or backward by distance (in meters)
void moveDistance(float distance, bool backward) {
	float realDistance = distance * 3.5; //offset = 3.5
  int targetTicks = distanceToTicks(realDistance);

  SensorValue[LEFT_ENCODER] = 0;
  SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = 1.28 * basePower; //offset =1.28
        motor[motorRight] = -basePower;
    } else {
        motor[motorLeft] = -1.28 * basePower;
        motor[motorRight] = basePower;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Turn left or right by angle (in degrees)
void turnDegrees(float degrees, bool right) {
    float realDegrees = degrees * 2.5; //offset is 2.5
    float turningCircumference = PI * wheelBase;  // Full turning circle
    float arcLength = (realDegrees / 180.0) * turningCircumference; // One wheel arc distance
	
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (right) {
        motor[motorLeft] = -127;
        motor[motorRight] = -127;
    } else {
        motor[motorLeft] = 127;
        motor[motorRight] =127;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void checkBoundary(void) {
  if ((IR_A == 1 && IR_C == 1) && (IR_B == 0 && IR_D == 0)) {
      leftScanBoundary = true;
      rightScanBoundary = false;  // Ensure this flag is reset
  } 
  else if ((IR_B == 1 && IR_D == 1) && (IR_A == 0 && IR_C == 0)) {
      rightScanBoundary = true;
      leftScanBoundary = false;  // Ensure this flag is reset
  } 
}


void scanSequenceLeft(void) {
  turnDegrees(20); //scan 20 degree to the left 
  wait1Msec(1000);
  turnDegrees(40, true); //scan 40 degree to the right 
  wait1Msec(1000);
  turnDegrees(20); //scan 20 degree to the left 
  wait1Msec(1000);
}

void scanSequenceRight(void) {
  turnDegrees(40); //scan 40 degree to the left 
  wait1Msec(1000);
  turnDegrees(20, true); //scan 20 degree to the right 
  wait1Msec(1000);
  turnDegrees(40); //scan 40 degree to the left 
  wait1Msec(1000);
}


void searchingAlgoLeft(void) {
  moveDistance(1.2);  //move a distance of 1.2 meter forward
  wait1Msec(1000);  //wait first before scan
  scanSequenceLeft(); //execute scan function 

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000);  //wait first before scan
  scanSequenceLeft(); //execute scan function

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000);  //wait first before scan
  scanSequenceLeft(); //execute scan function

  moveDistance(1.8, true); //move a distance of 1.8 meter backward
  wait1Msec(1000); //wait first before scan

  turnDegrees(20, true); //turn right 20 degrees 
  wait1Msec(1000); //wait first before move
  moveDistance(1.25); //move a distance of 1.25 meter forward
  wait1Msec(1000); //wait first before turn
  turnDegrees(40); //turn left 40 degrees 
  wait1Msec(1000); //wait first before turn
  turnDegrees(20, true); //turn right 20 degrees 
  wait1Msec(1000);//wait first before move

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000); //wait first before scan
  scanSequenceLeft(); //execute scan function

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000); //waitfirst before scan
  scanSequenceLeft(); //execute scan function

  moveDistance(1.8, true); //move a distance of 1.8 meter backward
  wait1Msec(1000);//wait 
}


void searchingAlgoRight(void) {
  moveDistance(1.2);
  wait1Msec(1000);
  scanSequenceRight();

  moveDistance(0.3);
  wait1Msec(1000);
  scanSequenceRight();

  moveDistance(0.3);
  wait1Msec(1000);
  scanSequenceRight();

  moveDistance(1.8, true);
  wait1Msec(1000);

  turnDegrees(20); // turn left
  wait1Msec(1000);
  moveDistance(1.25);
  wait1Msec(1000);
  turnDegrees(40, true);
  wait1Msec(1000);
  turnDegrees(20);
  wait1Msec(1000);

  moveDistance(0.3);
  wait1Msec(1000);
  scanSequenceRight();

  moveDistance(0.3);
  wait1Msec(1000);
  scanSequenceRight();

  moveDistance(1.8, true);
  wait1Msec(1000);
}

void searchingAlgo(void) {
  startTask(readIRTask);  // Start reading IR sensor values continuously
  wait1Msec(200);  // Give some time for the IR sensor task to gather initial readings

  checkBoundary();  // Check the boundary once

  if (leftScanBoundary) {  // Trigger left search algorithm if left boundary detected
      stopTask(readIRTask);
      searchingAlgoLeft();
  } 
  else if (rightScanBoundary) {  // Trigger right search algorithm if right boundary detected
      stopTask(readIRTask);
      searchingAlgoRight();
  } 
}

// ==================================================================  Concurrent tasks for required in different phases ==================================================================
// **************************Khirdir please update the concurrent tasks as needed**************************

// Execute the search algorithm during the search phase 
task searchAlgoTask(void) {
    searchingAlgo();  
  }

// Constantly scan for the ball during the search phase  
task scanBallTask(void) {
  while(task){
    scanBall();  
    wait1Msec(100);
  }
}

// Constantly scan for the boundary during all the phase
task scanBoundaryTask(void) {
  while(task){
    scanBoundary();  
    wait1Msec(100);
  }
}

// Constantly scan for obstacles during the all phase
task scanObstacleTask(void) {
  while(true){  
     scanObstacle(); 
     wait1Msec(100);
  }
}

// Execute the return to base algorithm during the return phase
task returnToBaseTask(void) {
    returnToBase();  
  }

// Execute the deliver algorithm during the deliver phase
task deliverTask(void) {
    deliver();  
  }

 // Read IR sensor values continuously
task readIRTask(void) {
    readIR(); 
  } 

// Read distance sensor values continuously
task readDistanceSensorTask(void) {
    readDistanceSensor();  
  }

// Release extra balls
task releaseExtraBallsTask(void) {
    frontRollerOutput();  
  }

task check_current_headingTask(void){
	while(true){
		heading = compass();
    wait1Msec(100);
	}
}

task moveTowardsBallTask(void) {
  moveTowardsBall();  
}

task pickUpBallTask(void) {
  pickUpBall();  
}

// ==================================================================  MAIN PROGRAM STARTS HERE ==================================================================
 task main() {
    // Initialize tasks
    startTask(check_current_heading);
    startTask(readIRTask);
    startTask(readDistanceSensorTask);

    while (true) {

      // Switch based on the current state
      switch (currentState) {
        case SEARCH:
          searchPhase();
          break; // just exit the switch statement , not the WHILE loop
        case COLLECT:
          collectPhase();
          break;
        case RETURN:
          returnPhase();
          break;
        case DELIVER:
          deliverPhase();
          break;
        default:
          currentState = SEARCH;  // Default to search if invalid state
      }
      wait1Msec(100);  // Small delay to prevent locking up the CPU, the main program keep checking the current state every 100ms
    }
  }  
