// pragma config ## WORK ON THIS TMR 



// Define states
typedef enum {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
  } RobotState;


// Define MACROS
#define PI 3.14159265359

//Global variables 
RobotState currentState = SEARCH;  // Start with the 'Search' phase
int IR_A // IR sensor A = 0 or 1, 0 means at boundary, 1 means not at boundary
int IR_B // IR sensor B = 0 or 1
int IR_C // IR sensor C = 0 or 1
int IR_D // IR sensor D = 0 or 1
bool isBoundary = false;  // Flag to indicate if the robot is at the boundary
bool isBall = false;  // Flag to indicate if the ball is detected
bool isObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBallPicked = false;  // Flag to indicate if the ball is picked up
bool reachedBase = false;  // Flag to indicate if the robot has reached the base
bool isDelivered = false;  // Flag to indicate if the ball is delivered

// Define constants
int basePower = 50;  // Power level for the base motor
float wheelDiameter = 0.06985; // meters
float wheelCircumference = wheelDiameter * PI; // meters
int ticksPerRevolution = 90; // encoder ticks per revolution
float distancePerTick = wheelCircumference / ticksPerRevolution; // meters per tick
float wheelBase = 0.188; // distance between wheels (meters)





// Phase functions with subfunctions 
void searchPhase() {
    startTask(searchAlgoTask);  // Start the search algorithm task
    startTask(scanBallTask);    // Start the scan ball task
    startTask(scanBoundaryTask);  // Start the scan boundary task
    startTask(scanObstacleTask);  // Start the scan Obstacle task
    
    // Keep checking for ball,boundary and obstacles in this phase, if not, keep doing the above 4 tasks concurrently
    while (true) {
      if (isBall) {
        currentState = COLLECT;  // Change to collect phase when the ball is detected
        stopTask(searchAlgoTask); // Stop the search algorithm task
        stopTask(scanBallTask);   // Stop the scan ball task
        isBall = false;  // Reset the flag
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

      wait1Msec(50);  // Small delay to prevent locking up the CPU, represent how often is the checking

    }
  }

void collectPhase() {
    startTask(scanBoundaryTask);  // Start the scan boundary task
    startTask(scanObstacleTask);  // Start the scan Obstacle task
    moveTowardsBall();  // Move towards the ball
    pickUpBall();       // Pick up the ball
    
    // Keep checking for boundary and obstacles and if ball is picked in this phase, if not, keep doing the above 2 tasks concurrently
    while(true) {
        
        if (isBoundary) {
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            handleBoundary();  // Handle boundary if the robot reaches the boundary during search
             //resume normal tasks in the collect phase
            isBoundary = false;  // Reset the flag
            startTask(scanBoundaryTask);  // Start the scan boundary task
            startTask(scanObstacleTask);  // Start the scan Obstacle task
          }
    
        else if (isObstacle) { //if an obstacle is detected
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            handleObstacle();  // Handle the obstacle
            //resume normal tasks in the collect phase
            isObstacle = false;  // Reset the flag
            startTask(scanBoundaryTask);  // Start the scan boundary task
            startTask(scanObstacleTask);  // Start the scan Obstacle task 
          }
        else if (isBallPicked) {  // Check if the ball has been successfully picked up
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            currentState = RETURN;  // Change to return phase when the ball is picked
            isBallPicked = false;  // Reset the flag
            break;  // Exit the while loop
        }

        wait1Msec(50);  // Small delay to prevent locking up the CPU, represent how often is the checking 
    }
}

void returnPhase() {
        startTask(scanBoundaryTask);  // Start the scan boundary task
        startTask(scanObstacleTask);  // Start the scan Obstacle task
        startTask(returnToBaseTask);  // Start the return to base task
        
         // Keep checking for boundary and obstacles and if robot successfully return to base in this phase, if not, keep doing the above 3 tasks concurrently
        while(true) {
        
            if (isBoundary) {
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                stopTask(returnToBaseTask); // Stop the return to base task
                handleBoundary();  // Handle boundary if the robot reaches the boundary during search
                //resume normal tasks in the return phase
                isBoundary = false;  // Reset the flag
                startTask(scanBoundaryTask);  // Start the scan boundary task
                startTask(scanObstacleTask);  // Start the scan Obstacle task
                startTask(returnToBaseTask);  // Start the return to base task
              }
        
            else if (isObstacle) { //if an obstacle is detected
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                handleObstacle();  // Handle the obstacle 
                //resume normal tasks in the return phase
                isObstacle = false;  // Reset the flag
                startTask(scanBoundaryTask);  // Start the scan boundary task
                startTask(scanObstacleTask);  // Start the scan Obstacle task
                startTask(returnToBaseTask);  // Start the return to base task
              }
            else if (reachedBase) {  // Check if robot has reached the base
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                stopTask(returnToBaseTask);  // stop the return to base task
                currentState = DELIVER;  // Change to deliver phase when robot reaches the base
                reachedBase = false;  // Reset the flag
                break;  // Exit the while loop
            }
    
            wait1Msec(50);  // Small delay to prevent locking up the CPU, represent how often is the checking 
        }
      
      }

void deliverPhase() {
        startTask(deliverTask);  // Start the scan boundary task
        
        while(true){
            if (isDelivered) {  // Check if the ball has been successfully delivered
                stopTask(deliverTask);  // Stop the deliver task
                currentState = SEARCH;  // Change to search phase after delivery
                isDelivered = false;  // Reset the flag
                break;  // Exit the while loop
            }
            wait1Msec(50);  // Small delay to prevent locking up the CPU, represent how often is the checking 
        }
       
      }

// Put all the functions here 

readIR(void){
    while (true){
        IR_A = SensorValue[IR_A]; 
        IR_B = SensorValue[IR_B]; 
        IR_C = SensorValue[IR_C]; 
        IR_D = SensorValue[IR_D]; 
    
        // Print digital sensor value to debug stream
        writeDebugStreamLine("IR_A: %d", IR_A_VALUE);
        writeDebugStreamLine("IR_B: %d", IR_B_VALUE);
        writeDebugStreamLine("IR_C: %d", IR_C_VALUE);
        writeDebugStreamLine("IR_D: %d", IR_D_VALUE);

    wait1Msec(50); // read IR values every 50ms
}

scanBoundary(void){
    startTask(readIRTask);  // Start the read IR task 
        // Check if any two IR sensors are 0
        if ((IR_A == 0 && IR_B == 0) || (IR_A == 0 && IR_C == 0) || (IR_A == 0 && IR_D == 0) || 
            (IR_B == 0 && IR_C == 0) || (IR_B == 0 && IR_D == 0) || (IR_C == 0 && IR_D == 0)) {
            isBoundary = true;  // Set isBoundary to true if any two IR sensors are 0
        }
}

handleBoundary(IR_A, IR_B, IR_C, IR_D){

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
        
        default:
            // do nothing just break
            break;
    }
    
}

readDistanceSensor(void){
    // Implement logic here to read the distance sensor
}

scanObstacle(){
    startTask(readDistanceSensor);  // Start the read IR task 
    // Implement logic here
    // return isObstacle = true if an obstacle is detected
}

handleObstacle(){
    // Implement logic here to handle the obstacle
}

searchingAlgo(){
    // Implement the search algorithm here
}

scanBall(){
    startTask(readDistanceSensor);  // Start the read IR task 
    // Implement logic here to scan for the ball
    // return isBall = true if the ball is detected
}

returnToBase(){
    // Implement the return to base algorithm here
    // use simple movement function + compass
    if //condition to check if the robot has reached the base, make use of the limit switches and other indicators 
    return reachedBase = true;
}

deliver(){
    // Implement the deliver algorithm here
    if //condition to check if the ball has been delivered, make use of the limit switches and other indicators
    return isDelivered = true;
}

pickUpBall(){
}       // Pick up the ball, yuwei work on it tmr 

moveTowardsBall(){
}  // Move towards the ball

// Convert desired distance in meters to encoder ticks
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}
// Convert degrees to Radians 
float degreesToRadians(float degrees) {
    return degrees * PI / 180.0;
}

// Move forward or backward by distance (in meters)
void moveDistance(float distance, bool backward = false) {
	float realDistance = distance *10; 
    int targetTicks = distanceToTicks(realDistance);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = basePower;
        motor[motorRight] = -basePower;
    } else {
        motor[motorLeft] = -basePower;
        motor[motorRight] = basePower;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Turn left or right by angle (in degrees)
void turnDegrees(float degrees, bool right = false) {
    //float realDegrees = degrees * 2.5;
    //float turningCircumference = 3.14159 * wheelBase;  // Full turning circle
    //float arcLength = (realDegrees / 180.0) * turningCircumference; // One wheel arc distance

    //Arc length formula : ArcLength=r*theta, where r is in radians 
    float angleRadian = degreesToRadians(degrees); 
    float arcLength = angleRadian * (wheelBase/2);
	
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

// Concurrent tasks for required in different phases
task searchAlgoTask() {
    searchingAlgo();  // Execute the search algorithm during the search phase 
  }
  
task scanBallTask() {
    scanBall();  // Constantly scan for the ball during the search phase
  }

task scanBoundaryTask() {
    scanBoundary();  // Constantly scan for the boundary during all the phase
  }

task scanObstacleTask() {
    scanObstacle();  // Constantly scan for obstacles during the all phase
  }
task returnToBaseTask() {
    returnToBase();  // Execute the return to base algorithm during the return phase
  }
task deliverTask() {
    deliver();  // Execute the deliver algorithm during the deliver phase
  }
task readIRTask() {
    readIR();  // Read IR sensor values continuously
  } 
task readDistanceSensorTask() {
    readDistanceSensor();  // Read distance sensor values continuously
  }



void task main() {
    while (true) {

      // Switch based on the current state
      switch (currentState) {
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
        default:
          currentState = SEARCH;  // Default to search if invalid state
      }
      
      wait1Msec(50);  // Small delay to prevent locking up the CPU, the main program keep checking the current state every 50ms
    }
  }  