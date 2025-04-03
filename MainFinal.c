// ================================================================== pragma configuration ==================================================================
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
#pragma config(Sensor, dgtl10, compass_LSB,    sensorDigitalIn) //PLEASE CHECK AGAIN TMR 
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn) //PLEASE CHECK AGAIN TMR 
#pragma config(Sensor, dgtl8,  compass_Bit2,   sensorDigitalIn) //PLEASE CHECK AGAIN TMR 
#pragma config(Sensor, dgtl7,  compass_MSB,    sensorDigitalIn) //PLEASE CHECK AGAIN TMR 
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9, BACK_ROLLER,    tmotorVex393_MC29, openLoop)


// ================================================================== Function prototypes ==================================================================
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
void scanSequenceLeft(void);
void scanSequenceRight(void);
void checkBoundary(void);
void frontRollerIntake(void);
void frontRollerStop(void);
void frontRollerOutput(void);
void FlapperPush(void);
void FlapperStop(void);
void FlapperReset(void);
int compass(void);
void readlimitswitch(void);
void searchingAlgoLeft(void);
void searchingAlgoRight(void); 
void searchingAlgo(void);
void readSharpSensor(void);
task searchAlgoTask(void);
task scanBallTask(void);
task scanBoundaryTask(void);
task scanObstacleTask(void);
task returnToBaseTask(void);
task deliverTask(void);
task readIRTask(void);
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
bool isFrontObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBackObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBallPicked = false;  // Flag to indicate if the ball is picked up
bool reachedBase = false;  // Flag to indicate if the robot has reached the base
bool isDelivered = false;  // Flag to indicate if the ball is delivered
bool leftScanBoundary = false; //Flag to indicate if the left boundary is detected
bool rightScanBoundary = false; //Flag to indicate if the right boundary is detected
int heading ; // Variable to store the compass heading
int limitswitchLB = 1; // Variable to store the value of the left limit switch, 1 means not pressed 
int limitswitchRB = 1; // Variable to store the value of the right limit switch
int limitswitchBall = 1; // Variable to store the value of the ball limit switch
float distFC = 0.0; // Variable to store the distance from the front center sharp sensor
float distFR = 0.0; // Variable to store the distance from the front right sharp sensor
float distFL = 0.0; // Variable to store the distance from the front left sharp sensor
float distBC = 0.0; // Variable to store the distance from the back center sharp sensor
int SharpFC_Value; // Variable to store the value of the front center sharp sensor
int SharpFR_Value; // Variable to store the value of the front right sharp sensor
int SharpFL_Value; // Variable to store the value of the front left sharp sensor
int SharpBC_Value; // Variable to store the value of the back center sharp sensor
bool robotMovingBack; // Flag to indicate if the robot is moving front or backwards 
bool robotMovingFront; // Flag to indicate if the robot is moving front or backwards

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



// ================================================================== Phase functions ==================================================================
void searchPhase(void) {
  startTask(searchAlgoTask);  // Start the search algorithm task
  startTask(scanBallTask);    // Start the scan ball task
  startTask(scanBoundaryTask);  // Start the scan boundary task
  startTask(scanObstacleTask);  // Start the scan obstacle task

  while (true) {
      AcquireMutex(mutex);  // Ensure mutual exclusion for shared variables

      // If ball is detected, stop tasks and switch to collect phase
      if (isBall) {
          stopTask(searchAlgoTask);
          stopTask(scanBallTask);
          stopTask(scanBoundaryTask);
          stopTask(scanObstacleTask);

          isBall = false;  // Reset the ball detection flag
          currentState = COLLECT;  // Change to collect phase
          releaseMutex(mutex);  // Release the mutex
          break;  // Exit the loop and move to the collect phase
      }
      AcquireMutex(mutex);
      // If boundary or obstacle is detected, stop all tasks except scanBoundaryTask and scanObstacleTask
      if (isBoundary || isFrontObstacle || isBackObstacle) {
          if (!robotMovingBack) {  // Stop tasks only if robot is not moving backward
              stopTask(searchAlgoTask);  // Stop search algorithm task
              stopTask(scanBallTask);    // Stop scan ball task
          }

          // Handle boundary first if detected
          if (isBoundary) {
              releaseMutex(mutex);
              handleBoundary();  // Handle boundary issue
              wait1Msec(200);  // Wait to see if scanBoundary() detects the boundary again
              // After handling boundary, check if it's resolved
              AcquireMutex(mutex);
              if (!isBoundary) {
                  // If boundary is resolved, check for obstacles again
                  if (isFrontObstacle || isBackObstacle) {
                      // If obstacles are detected, handle them
                      if (robotMovingBack) {
                          stopTask(searchAlgoTask);  // Stop search algorithm task
                          stopTask(scanBallTask);    // Stop scan ball task
                      }
                      handleObstacle();  // Handle obstacle if detected
                      releaseMutex(mutex);
                      wait1Msec(200);  // Wait before double-checking the obstacle

                      // After handling the obstacle, double-check if the obstacle is still present
                      AcquireMutex(mutex);
                      if (isFrontObstacle || isBackObstacle) {
                          handleObstacle();  // Handle it again if still detected
                          releaseMutex(mutex);
                      }
                  }
              }
              // If boundary is resolved, continue to next loop to check for remaining conditions
              // Resume the tasks after boundary is handled
              startTask(searchAlgoTask);  // Resume the search algorithm task
              startTask(scanBallTask);    // Resume the scan ball task
              wait1Msec(100);  // Small delay to prevent locking up the CPU
              continue;
          }

          // Optionally, handle obstacle if detected and boundary wasn't detected
          AcquireMutex(mutex);
          if (isFrontObstacle || isBackObstacle) {
              // If obstacles are detected, handle them
              if (robotMovingBack) {
                stopTask(searchAlgoTask);  // Stop search algorithm task
                stopTask(scanBallTask);    // Stop scan ball task
            }

            handleObstacle();  // Handle obstacle if detected
            releaseMutex(mutex);
            wait1Msec(200);  // Wait before double-checking the obstaclen
            
              // After handling the obstacle, double-check if the obstacle is still present
              AcquireMutex(mutex);
              if (isFrontObstacle || isBackObstacle) {
                  handleObstacle();  // Handle it again if still detected
                  releaseMutex(mutex);
              }

              // Resume tasks after handling the obstacle
              startTask(searchAlgoTask);  // Resume the search algorithm task
              startTask(scanBallTask);    // Resume the scan ball task
              wait1Msec(100);  // Small delay to prevent locking up the CPU
              continue;
          }
      }
  }
}

void collectPhase(void) {
    startTask(scanBoundaryTask);  // Start the scan boundary task
    startTask(scanObstacleTask);  // Start the scan Obstacle task
    startTask(moveTowardsBallTask);  // Start the move towards ball task
    startTask(pickUpBallTask);       // Start the pick up ball task, means the robot can pickup any ball as it moves towards the target ball
    
    // Keep checking for boundary and obstacles and if ball is picked in this phase, if not, keep doing the above 2 tasks concurrently
    while(true) {
        AcquireMutex(mutex);  // Ensure mutual exclusion for shared variables
        if (isBallPicked) {  // Check if the ball has been successfully picked up
            stopTask(scanBoundaryTask);  // Stop the scan boundary task
            stopTask(scanObstacleTask);  // Stop the scan obstacle task
            stopTask(moveTowardsBallTask);  // Stop the move towards ball task
            stopTask(pickUpBallTask);       // Stop the pick up ball task
            currentState = RETURN;  // Change to return phase when the ball is picked
            isBallPicked = false;  // Reset the flag
            releaseMutex(mutex);  // Release the mutex
            break;  // Exit the while loop
    }
    AcquireMutex(mutex);
      // If boundary or obstacle is detected, stop all tasks except scanBoundaryTask and scanObstacleTask
      if (isBoundary || isFrontObstacle || isBackObstacle) {
          if (!robotMovingBack) {  // Stop tasks only if robot is not moving backward
              stopTask(moveTowardsBallTask);  
              stopTask(pickUpBallTask);   
          }

          // Handle boundary first if detected
          if (isBoundary) {
              releaseMutex(mutex);
              handleBoundary();  // Handle boundary issue
              wait1Msec(200);  // Wait to see if scanBoundary() detects the boundary again
              // After handling boundary, check if it's resolved
              AcquireMutex(mutex);
              if (!isBoundary) {
                  // If boundary is resolved, check for obstacles again
                  if (isFrontObstacle || isBackObstacle) {
                      // If obstacles are detected, handle them
                      if (robotMovingBack) {
                          stopTask(moveTowardsBallTask);  // Stop search algorithm task
                          stopTask(pickUpBallTask);    // Stop scan ball task
                      }
                      handleObstacle();  // Handle obstacle if detected
                      releaseMutex(mutex);
                      wait1Msec(200);  // Wait before double-checking the obstacle

                      // After handling the obstacle, double-check if the obstacle is still present
                      AcquireMutex(mutex);
                      if (isFrontObstacle || isBackObstacle) {
                          handleObstacle();  // Handle it again if still detected
                          releaseMutex(mutex);
                      }
                  }
              }
              // If boundary is resolved, continue to next loop to check for remaining conditions
              // Resume the tasks after boundary is handled
              startTask(moveTowardsBallTask);  // Resume the search algorithm task
              startTask(pickUpBallTask);    // Resume the scan ball task
              wait1Msec(100);  // Small delay to prevent locking up the CPU
              continue;
          }

          // Optionally, handle obstacle if detected and boundary wasn't detected
          AcquireMutex(mutex);
          if (isFrontObstacle || isBackObstacle) {
              // If obstacles are detected, handle them
              if (robotMovingBack) {
                stopTask(moveTowardsBallTask);  // Stop search algorithm task
                stopTask(pickUpBallTask);    // Stop scan ball task
            }

            handleObstacle();  // Handle obstacle if detected
            releaseMutex(mutex);
            wait1Msec(200);  // Wait before double-checking the obstaclen
            
              // After handling the obstacle, double-check if the obstacle is still present
              AcquireMutex(mutex);
              if (isFrontObstacle || isBackObstacle) {
                  handleObstacle();  // Handle it again if still detected
                  releaseMutex(mutex);
              }

              // Resume tasks after handling the obstacle
              startTask(moveTowardsBallTask);  // Resume the search algorithm task
              startTask(pickUpBallTask);    // Resume the scan ball task
              wait1Msec(100);  // Small delay to prevent locking up the CPU
              continue;
          }
      }
  }
}
        
        

void returnPhase(void) {
        startTask(scanBoundaryTask);  // Start the scan boundary task
        startTask(scanObstacleTask);  // Start the scan Obstacle task
        startTask(returnToBaseTask);  // Start the return to base task
        startTask(releaseExtraBallsTask);  // Start the release extra balls task, continuously, so prevent ball from rolling inside the robot while robot is moving back to base
        
         // Keep checking for boundary and obstacles and if robot successfully return to base in this phase, if not, keep doing the above 3 tasks concurrently
        while(true) {

              AcquireMutex(mutex);  // Ensure mutual exclusion for shared variables
              if (reachedBase) {  // Check if robot has reached the base
                stopTask(scanBoundaryTask);  // Stop the scan boundary task
                stopTask(scanObstacleTask);  // Stop the scan obstacle task
                stopTask(returnToBaseTask);  // stop the return to base task
                stopTask(releaseExtraBallsTask);  // Stop the release extra balls task
                currentState = DELIVER;  // Change to deliver phase when robot reaches the base
                reachedBase = false;  // Reset the flag
                releaseMutex(mutex);  // Release the mutex
                break;  // Exit the while loop
            }

            AcquireMutex(mutex);
            // If boundary or obstacle is detected, stop all tasks except scanBoundaryTask and scanObstacleTask
            if (isBoundary || isFrontObstacle || isBackObstacle) {
                if (!robotMovingBack) {  // Stop tasks only if robot is not moving backward
                    stopTask(returnToBaseTask);  
                    stopTask(releaseExtraBallsTask);    
                }
      
                // Handle boundary first if detected
                if (isBoundary) {
                    releaseMutex(mutex);
                    handleBoundary();  // Handle boundary issue
                    wait1Msec(200);  // Wait to see if scanBoundary() detects the boundary again
                    // After handling boundary, check if it's resolved
                    AcquireMutex(mutex);
                    if (!isBoundary) {
                        // If boundary is resolved, check for obstacles again
                        if (isFrontObstacle || isBackObstacle) {
                            // If obstacles are detected, handle them
                            if (robotMovingBack) {
                                stopTask(returnToBaseTask);  
                                stopTask(releaseExtraBallsTask);    
                            }
                            handleObstacle();  // Handle obstacle if detected
                            releaseMutex(mutex);
                            wait1Msec(200);  // Wait before double-checking the obstacle
      
                            // After handling the obstacle, double-check if the obstacle is still present
                            AcquireMutex(mutex);
                            if (isFrontObstacle || isBackObstacle) {
                                handleObstacle();  // Handle it again if still detected
                                releaseMutex(mutex);
                            }
                        }
                    }
                    // If boundary is resolved, continue to next loop to check for remaining conditions
                    // Resume the tasks after boundary is handled
                    startTask(returnToBaseTask);  // Resume the search algorithm task
                    startTask(releaseExtraBallsTask);    // Resume the scan ball task
                    wait1Msec(100);  // Small delay to prevent locking up the CPU
                    continue;
                }
      
                // Optionally, handle obstacle if detected and boundary wasn't detected
                AcquireMutex(mutex);
                if (isFrontObstacle || isBackObstacle) {
                    // If obstacles are detected, handle them
                    if (robotMovingBack) {
                      stopTask(returnToBaseTask);  // Stop search algorithm task
                      stopTask(releaseExtraBallsTask);    // Stop scan ball task
                  }
      
                  handleObstacle();  // Handle obstacle if detected
                  releaseMutex(mutex);
                  wait1Msec(200);  // Wait before double-checking the obstaclen
                  
                    // After handling the obstacle, double-check if the obstacle is still present
                    AcquireMutex(mutex);
                    if (isFrontObstacle || isBackObstacle) {
                        handleObstacle();  // Handle it again if still detected
                        releaseMutex(mutex);
                    }
      
                    // Resume tasks after handling the obstacle
                    startTask(returnToBaseTask);  // Resume the search algorithm task
                    startTask(releaseExtraBallsTask);    // Resume the scan ball task
                    wait1Msec(100);  // Small delay to prevent locking up the CPU
                    continue;
                }
            }
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

void FlapperReset(void){
  motor[BACK_ROLLER] = rollerSpeed; 
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
      IR_A = SensorValue[IR_A] < 300 ? 0 : 1; // Threshold for detection boundary is 300 
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
  while(true){
  AcquireMutex(mutex);
  if ((IR_A == 0 && IR_B == 0) || (IR_A == 0 && IR_C == 0) || (IR_A == 0 && IR_D == 0) || 
      (IR_B == 0 && IR_C == 0) || (IR_B == 0 && IR_D == 0) || (IR_C == 0 && IR_D == 0)) {
      isBoundary = true;
  }
  ReleaseMutex(mutex);
  wait1Msec(100); // Small delay to prevent locking up the CPU
  }
}


void handleBoundary(void){

    int IR_State = 0;
    AcquireMutex(mutex);
    if (IR_A == 0 && IR_B == 0 && IR_C == 1 && IR_D == 1) {
        IR_State = 1;  // Represents case 1
    } else if (IR_A == 0 && IR_B == 1 && IR_C == 0 && IR_D == 0) {
        IR_State = 2;  // Represents case 2
    } else if (IR_A == 1 && IR_B == 0 && IR_C == 1 && IR_D == 0) {
        IR_State = 3;  // Represents case 3
    } else if (IR_A == 1 && IR_B == 1 && IR_C == 0 && IR_D == 0) {
        IR_State = 4;  // Represents case 4
    }
    releaseMutex(mutex);
    
    switch (IR_State) {
        case 1:
            // move backwards for 0.5m and make a 180 degrees turn
            moveDistance(0.5, true);
            wait1Msec(1000); // wait for 1 second to ensure the robot has moved back
            turnDegrees(180); 
            wait1Msec(1000); // wait for 1 second to ensure the robot has turned
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
            moveDistance(0.3, true):  // Back up slightly
            wait1Msec(1000); // wait for 1 second to ensure the robot has moved back
            turnDegrees(45);          // Turn left 45 degrees
            break;
    }
    
}

void convertSharpToDistance(tSensors sensor) {  
  int sum = 0;
  int samples = 5;  // Number of samples to average
  
  // Read multiple samples and calculate the sum
  for (int i = 0; i < samples; i++) {
      sum += SensorValue[sensor];
      wait1Msec(10);  // Wait between each reading to avoid flooding the sensors
  }

  // Calculate the average sensor value
  int avgAnalog = sum / samples;

  // Convert the average analog reading to voltage (5V reference)
  float voltage = (avgAnalog * 5.0) / 4095.0;
  float distance = 80.0;  // Default fallback value for distance

  // If the sensor reading is 0 (no object detected), use a maximum distance
  if (avgAnalog == 0) {
      distance = 80.0;  // No object, maximum distance
  }
  // Otherwise, calculate the distance based on the sensor type
  else if (sensor == sharpFC) {
      distance = 25.99 / pow(voltage, 1.22);  // FC sensor formula
  } else if (sensor == sharpFR) {
      distance = 28.37 / pow(voltage, 1.08);  // FR sensor formula
  } else if (sensor == sharpFL) {
      distance = 26.82 / pow(voltage, 1.28);  // FL sensor formula
  } else if (sensor == sharpBC) {
      distance = 10.02 / pow(voltage, 1.26);  // BC sensor formula
  }

  // Update the global variable corresponding to the sensor
  AcquireMutex(mutex); 
  if (sensor == sharpFC) {
      distFC = distance;
  } else if (sensor == sharpFR) {
      distFR = distance;
  } else if (sensor == sharpFL) {
      distFL = distance;
  } else if (sensor == sharpBC) {
      distBC = distance;
  }
  ReleaseMutex(mutex);
}

void scanObstacle() {
  while(true){
    AcquireMutex(mutex); 
    // Read the distance from the front center sensor at one instance 
    if (distFC >= 10.0 && distFC <= 80.0){
      isFrontObstacle = true;
    else if (distFC >= 10.0 && distFC <= 80.0){
      isBackObstacle = true;
    }
    }
    ReleaseMutex(mutex);
    wait1Msec(100); // Small delay to prevent locking up the CPU
  }
}

void handleObstacle(){
  AcquireMutex(mutex);
  if (robotMovingFront && isFrontObstacle) {
      // SOME ACTIONS 
  } else if (robotMovingBack && isBackObstacle) {
      // SOME ACTIONS 
  }
  ReleaseMutex(mutex);
}

// === Detects Ball === 
void scanBall() {
  while(true){
      AcquireMutex(mutex);  // Ensure mutual exclusion for shared variables
      if ((distFL >= 10.0 && distFL <= 70.0) || 
          (distFR >= 10.0 && distFR <= 70.0)) {
          isBall = true;  // Ball detected
      }
      ReleaseMutex(mutex);  // Release the mutex after modifying shared variables
      wait1Msec(100);  // Small delay to prevent locking up the CPU
  }
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
    // Continuously check the condition while moving towards the base
    while (true) {
        // Adjust the robot's heading to 270 degrees
        float degree = abs(270 - heading);
        turnDegrees(degree, true);

        // Move a specific distance towards the base (converted from cm to meters)
        moveDistance(MAX_DISTANCE / 100.0, true);

        // Check if the robot is facing the base, and other conditions are met
        if (heading == 270 && (limitswitchLB == 1 || limitswitchRB == 1) && (IR_C == 0 && IR_D == 0)) {
            // If all conditions are met, mark the base as reached
            AcquireMutex(mutex);  // Ensure mutual exclusion when accessing shared variable
            reachedBase = true;    // Set reachedBase flag to true
            ReleaseMutex(mutex);   // Release mutex after updating reachedBase
            break;  // Exit the loop as the base has been reached
        }

        // Optionally, you can include a small delay to prevent continuous checking from overwhelming the CPU
        wait1Msec(100);
    }
}


void deliver(){
  FlapperPush(); //push the ball out
  wait1Msec(1000); //wait for 1 second to deliver the ball, Flapper at its negative maximum position
  FlapperStop(); //stop the flapper motor

  while(true){
  if (limitswitchBall == 1){ //means there is no more ball
    FlapperReset();
    wait1Msec(1000); //wait for 1 second to deliver the ball, Flapper at its positive maximum position
    FlapperStop(); //stop the flapper motor
    // Set the flag to indicate delivery is complete
    AcquireMutex(mutex);
    isDelivered = true;
    ReleaseMutex(mutex);
  }
  wait1Msec(100); // Small delay to prevent locking up the CPU
  }
}

void pickUpBall(void) {
  frontRollerIntake(); // Start the front roller to pick up the ball
  while (true) { //keep checking if the ball is picked up, if not front roller keeps rolling 
      if (SensorValue[limitswitchBall] == 0) {
          frontRollerStop();
          AcquireMutex(mutex);
          isBallPicked = true;
          ReleaseMutex(mutex);
          break;
      }
      wait1Msec(100);
  }
}    


void MoveTowardsBall() {
  AcquireMutex(mutex);  // Acquire mutex to ensure mutual exclusion for shared variables

  // Continue moving towards the ball until the condition is met
  while (distFL >= 20.0 || distFR >= 20.0) {
      motor[motorLeft] = 30;  // Move forward by setting motor speeds
      motor[motorRight] = 30;
  }

  // Stop the motors when either distFL or distFR is less than 20.0
  motor[motorLeft] = 0;
  motor[motorRight] = 0;

  if (distFL < 20.0) {
      turnDegrees(30, true);  // Turn right if the left sensor detects ball
  } else if (distFR < 20.0) {
      turnDegrees(30); // Turn left if the right sensor detects ball
  }

  releaseMutex(mutex);  // Release the mutex once the condition is checked and motors are stopped
}


// Convert desired distance in meters to encoder ticks
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

// Move forward or backward by distance (in meters)
void moveDistance(float distance, bool backward = false ) {
	float realDistance = distance * 3.5; //offset = 3.5
  int targetTicks = distanceToTicks(realDistance);

  SensorValue[LEFT_ENCODER] = 0;
  SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = 1.28 * basePower; //offset =1.28
        motor[motorRight] = -basePower;
        AcquireMutex(mutex);
        robotMovingBack = true; // Set the flag to indicate the robot is moving backward
        robotMovingFront = false; // Set the flag to indicate the robot is moving backward
        ReleaseMutex(mutex);
    } else {
        motor[motorLeft] = -1.28 * basePower;
        motor[motorRight] = basePower;
        AcquireMutex(mutex);
        robotMovingFront = true; // Set the flag to indicate the robot is moving backward
        robotMovingBack = false;
        ReleaseMutex(mutex);
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    AcquireMutex(mutex);
    robotMovingFront = false; // Set the flag to indicate the robot is moving backward
    robotMovingBack = false; // Set the flag to indicate the robot is moving backward
    ReleaseMutex(mutex);
}

// Turn left or right by angle (in degrees)
void turnDegrees(float degrees, bool right) {
    float realDegrees = degrees * 2.5; //offset is 2.5
    float turningCircumference = PI * wheelBase;  // Full turning circle
    float arcLength = (realDegrees / 180.0) * turningCircumference; // One wheel arc distance
	
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int reducedSpeed = 127 * 0.7;  // Reduce speed by 30%, approximately 89

    if (right) {
        motor[motorLeft] = -reducedSpeed;
        motor[motorRight] = -reducedSpeed;
    } else {
        motor[motorLeft] = reducedSpeed;
        motor[motorRight] = reducedSpeed;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void checkBoundary(void) {
  //Robot starts at the left Boundary
  if ((IR_A == 1 && IR_C == 1) && (IR_B == 0 && IR_D == 0)) {
      leftScanBoundary = true;
      rightScanBoundary = false;  // Ensure this flag is reset
  } 
  //Robot starts at the right Boundary
  else if ((IR_B == 1 && IR_D == 1) && (IR_A == 0 && IR_C == 0)) {
      rightScanBoundary = true;
      leftScanBoundary = false;  // Ensure this flag is reset
  } 
}


void scanSequenceLeft(void) {
  turnDegrees(40, true); //scan 40 degree to the right 
  wait1Msec(1000);
  turnDegrees(40); //scan 40 degree to the left 
  wait1Msec(1000);
}

void scanSequenceRight(void) {
  turnDegrees(40); //scan 40 degree to the left 
  wait1Msec(1000);
  turnDegrees(40,true); //scan 40 degree to the right
  wait1Msec(1000);
}


void searchingAlgoLeft(void) {
  moveDistance(1.2);  //move a distance of 1.2 meter forward
  wait1Msec(1000);  //wait first before scan
  scanSequenceLeft(); //execute scan function to pan left and right

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000);  //wait first before scan
  scanSequenceLeft(); //execute scan function to pan left and right

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000);  //wait first before scan
  scanSequenceLeft(); //execute scan function to pan left and right

  moveDistance(1.8, true); //move a distance of 1.8 meter backward
  wait1Msec(1000); //wait first before scan

  turnDegrees(20, true); //turn right 20 degrees 
  wait1Msec(1000); //wait first before move

  moveDistance(1.25); //move a distance of 1.25 meter forward
  wait1Msec(1000); //wait first before turn

  turnDegrees(40); //turn left 40 degrees 
  wait1Msec(1000); //wait first before turn

  turnDegrees(20, true); //turn right 20 degrees , now robot is back straight
  wait1Msec(1000);//wait first before move

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000); //wait first before scan
  scanSequenceLeft(); //execute scan function

  moveDistance(0.3); //move a distance of 0.3 meter forward
  wait1Msec(1000); //waitfirst before scan
  scanSequenceLeft(); //execute scan function

  moveDistance(1.8, true); //move a distance of 1.8 meter backward
  wait1Msec(1000); //wait 
}


void searchingAlgoRight(void) {
  moveDistance(1.2); // move a distance of 1.2 meter forward
  wait1Msec(1000); // wait first before scan
  scanSequenceRight(); // execute scan function to pan left and right

  moveDistance(0.3); // move a distance of 0.3 meter forward
  wait1Msec(1000); // wait first before scan
  scanSequenceRight(); // execute scan function to pan left and right

  moveDistance(0.3); // move a distance of 0.3 meter forward
  wait1Msec(1000); // wait first before scan
  scanSequenceRight(); // execute scan function to pan left and right

  moveDistance(1.8, true); // move a distance of 1.8 meter backward
  wait1Msec(1000); // wait first before scan

  turnDegrees(20); // turn left 20 degrees
  wait1Msec(1000); // wait first before move

  moveDistance(1.25); // move a distance of 1.25 meter forward
  wait1Msec(1000); // wait first before turn

  turnDegrees(40, true); // turn right 40 degrees
  wait1Msec(1000); // wait first before turn

  turnDegrees(20); // turn left 20 degrees , now robot is back straight
  wait1Msec(1000); // wait first before move

  moveDistance(0.3); // move a distance of 0.3 meter forward
  wait1Msec(1000); // wait first before scan
  scanSequenceRight(); // execute scan function to pan left and right

  moveDistance(0.3); // move a distance of 0.3 meter forward
  wait1Msec(1000); // wait first before scan
  scanSequenceRight(); // execute scan function to pan left and right

  moveDistance(1.8, true); // move a distance of 1.8 meter backward
  wait1Msec(1000);
}

void searchingAlgo(void) {

  checkBoundary();  // Check the boundary once

  if (leftScanBoundary) {  // Trigger left search algorithm if left boundary detected
      searchingAlgoLeft();
  } 
  else if (rightScanBoundary) {  // Trigger right search algorithm if right boundary detected
      searchingAlgoRight();
  } 
}

// ==================================================================  Concurrent tasks for required in different phases ==================================================================
// Execute the search algorithm during the search phase 
task searchAlgoTask(void) {
    searchingAlgo();  
  }

// Constantly scan for the ball during the search phase  
task scanBallTask(void) {
    scanBall();  
}

// Constantly scan for the boundary during all the phase
task scanBoundaryTask(void) {
     scanBoundary();  
}
// Constantly scan for obstacles during the all phase
task scanObstacleTask(void) {
     scanObstacle(); 
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

task readSharpFC_Task() {
  while(true) {
      convertSharpToDistance(sharpFC);
      wait1Msec(50);
  }
}

task readSharpFR_Task() {
  while(true) {
      convertSharpToDistance(sharpFR);
      wait1Msec(50);
  }
}

task readSharpFL_Task() {
  while(true) {
      convertSharpToDistance(sharpFL);
      wait1Msec(50);
  }
}

task readSharpBC_Task() {
  while(true) {
      convertSharpToDistance(sharpBC);
      wait1Msec(50);
  }
}

// ==================================================================  MAIN PROGRAM STARTS HERE ==================================================================
 task main() {
    // Initialize basic tasks
    startTask(check_current_headingTask);
    startTask(readIRTask);

    //Read Sharp distance sensor in the background 
    startTask(readSharpFC_Task);
    startTask(readSharpFR_Task);
    startTask(readSharpFL_Task);
    startTask(readSharpBC_Task);
    wait1Msec(500); //Let the sensors stabilise first 

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
