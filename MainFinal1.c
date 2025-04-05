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
#pragma config(Sensor, dgtl8, compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10,  compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11,  compass_MSB,    sensorDigitalIn)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorVex393_MC29, openLoop)

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
void readSharpInDistance(void);
void convertSharpToDistance(tSensors sensor);
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
task readSharpFC_Task(void);
task readSharpFR_Task(void);
task readSharpFL_Task(void);
task readSharpBC_Task(void);

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

int IR_A_val; // Line follow 0 means at boundary, 1 means not at boundary
int IR_B_val;
int IR_C_val;
int IR_D_val ;
int limitswitchLB_val;  // 0 means pressed,  1 means not pressed 
int limitswitchRB_val;  
int limitswitchBall_val; 
int heading; // Variable to store the compass heading
int SharpFC_Value; // Variable to store the value of the front center sharp sensor
int SharpFR_Value; // Variable to store the value of the front right sharp sensor
int SharpFL_Value; // Variable to store the value of the front left sharp sensor
int SharpBC_Value; // Variable to store the value of the back center sharp sensor

bool isBoundary = false;  // Flag to indicate if the robot is at the boundary
bool isBall = false;  // Flag to indicate if the ball is detected
bool isFrontObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBackObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBallPicked = false;  // Flag to indicate if the ball is picked up
bool reachedBase = false;  // Flag to indicate if the robot has reached the base
bool isDelivered = false;  // Flag to indicate if the ball is delivered
bool leftScanBoundary = false; //Flag to indicate if the left boundary is detected
bool rightScanBoundary = false; //Flag to indicate if the right boundary is detected
bool robotMovingBack = false; // Flag to indicate if the robot is moving backwards
bool robotMovingFront = false; // Flag to indicate if the robot is moving forwards
bool firstStart = true;


float distFC = 0.0; // Variable to store the distance from the front center sharp sensor
float distFR = 0.0; // Variable to store the distance from the front right sharp sensor
float distFL = 0.0; // Variable to store the distance from the front left sharp sensor
float distBC = 0.0; // Variable to store the distance from the back center sharp sensor






// ================================================================== Define constants ==================================================================
const int basePower = 50;  // Power level for the base motor
const float wheelDiameter = 0.06985; // meters
const float wheelCircumference = wheelDiameter * PI; // meters
const int ticksPerRevolution = 90; // encoder ticks per revolution
const float distancePerTick = wheelCircumference / ticksPerRevolution; // meters per tick
const float wheelBase = 0.188; // distance between wheels (meters)
const int rollerSpeed = 127; // speed of the roller motors
const int MAX_DISTANCE = 300; // 300cm max distance for return

// ================================================================== Phase functions ==================================================================
void searchPhase(void) {
    //Do all these tasks simultaneously 
    startTask(searchAlgoTask);
    startTask(scanBallTask);
    startTask(scanBoundaryTask);
    startTask(scanObstacleTask);

    while (true) {
          
        if (isBall) {
            stopTask(searchAlgoTask);
            stopTask(scanBallTask);
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            isBall = false;
            currentState = COLLECT;
              
            break;
        }
            // Release if condition not met
          
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) {
                stopTask(searchAlgoTask);
                stopTask(scanBallTask);
                  
            }

            if (isBoundary) {
                  
                handleBoundary();
                wait1Msec(200);
                  
                if (!isBoundary) {
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) {
                            stopTask(searchAlgoTask);
                            stopTask(scanBallTask);
                        }
                          
                        handleObstacle();
                        wait1Msec(200);
                          
                        if (isFrontObstacle || isBackObstacle) {
                              
                            handleObstacle();
                              
                        }
                    }
                }
                returnToBase();
                wait1Msec(200);
                startTask(searchAlgoTask);
                startTask(scanBallTask);
                  
                wait1Msec(100);
                continue;
            }

            if (isFrontObstacle || isBackObstacle) {
                if (robotMovingBack) {
                    stopTask(searchAlgoTask);
                    stopTask(scanBallTask);
                }
                  
                handleObstacle();
                wait1Msec(200);
                  
                if (isFrontObstacle || isBackObstacle) {
                      
                    handleObstacle();
                      
                }
                startTask(searchAlgoTask);
                startTask(scanBallTask);
                  
                wait1Msec(100);
                continue;
            }
        }
          
    }
}

void collectPhase(void) {
    startTask(scanBoundaryTask);
    startTask(scanObstacleTask);
    startTask(moveTowardsBallTask);
    startTask(pickUpBallTask);
    
    while(true) {
          
        if (!isBall) {
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            stopTask(moveTowardsBallTask);
            stopTask(pickUpBallTask);
            currentState = SEARCH;
              
            break;
        }
          
        if (isBallPicked) {
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            stopTask(moveTowardsBallTask);
            stopTask(pickUpBallTask);
            currentState = RETURN;
            isBallPicked = false;
              
            break;
        }
          
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) {
                stopTask(moveTowardsBallTask);
                stopTask(pickUpBallTask);
            }

            if (isBoundary) {
                  
                handleBoundary();
                wait1Msec(200);
                  
                if (!isBoundary) {
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) {
                            stopTask(moveTowardsBallTask);
                            stopTask(pickUpBallTask);
                        }
                          
                        handleObstacle();
                        wait1Msec(200);
                          
                        if (isFrontObstacle || isBackObstacle) {
                              
                            handleObstacle();
                              
                        }
                    }
                }
                startTask(moveTowardsBallTask);
                startTask(pickUpBallTask);
                  
                wait1Msec(100);
                continue;
            }

            if (isFrontObstacle || isBackObstacle) {
                if (robotMovingBack) {
                    stopTask(moveTowardsBallTask);
                    stopTask(pickUpBallTask);
                }
                  
                handleObstacle();
                wait1Msec(200);
                  
                if (isFrontObstacle || isBackObstacle) {
                      
                    handleObstacle();
                      
                }
                startTask(moveTowardsBallTask);
                startTask(pickUpBallTask);
                  
                wait1Msec(100);
                continue;
            }
        }
          
    }
}

void returnPhase(void) {
    startTask(scanBoundaryTask);
    startTask(scanObstacleTask);
    startTask(returnToBaseTask);
    startTask(releaseExtraBallsTask);
    
    while(true) {
          
        if (reachedBase) {
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            stopTask(returnToBaseTask);
            stopTask(releaseExtraBallsTask);
            currentState = DELIVER;
            reachedBase = false;
              
            break;
        }
        
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) {
                stopTask(returnToBaseTask);
                stopTask(releaseExtraBallsTask);
            }

            if (isBoundary) {
                  
                handleBoundary();
                wait1Msec(200);
                  
                if (!isBoundary) {
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) {
                            stopTask(returnToBaseTask);
                            stopTask(releaseExtraBallsTask);
                        }
                          
                        handleObstacle();
                        wait1Msec(200);
                          
                        if (isFrontObstacle || isBackObstacle) {
                              
                            handleObstacle();
                              
                        }
                    }
                }
                startTask(returnToBaseTask);
                startTask(releaseExtraBallsTask);
                  
                wait1Msec(100);
                continue;
            }

            if (isFrontObstacle || isBackObstacle) {
                if (robotMovingBack) {
                    stopTask(returnToBaseTask);
                    stopTask(releaseExtraBallsTask);
                }
                  
                handleObstacle();
                wait1Msec(200);
                  
                if (isFrontObstacle || isBackObstacle) {
                      
                    handleObstacle();
                      
                }
                startTask(returnToBaseTask);
                startTask(releaseExtraBallsTask);
                  
                wait1Msec(100);
                continue;
            }
        }
          
    }
}

void deliverPhase(void) {
    startTask(deliverTask);
    
    while(true) {
        if (isDelivered) {
            stopTask(deliverTask);
            currentState = SEARCH;
            isDelivered = false;
            break;
        }
        wait1Msec(100);
    }
}

// ================================================================== Helper functions ==================================================================
void frontRollerIntake(void) {
    motor[FRONT_ROLLER] = -rollerSpeed;
}       

void frontRollerStop(void) { 
    motor[FRONT_ROLLER] = 0; 
}

void frontRollerOutput(void) {
    motor[FRONT_ROLLER] = rollerSpeed;
}     

void FlapperPush(void) {
    motor[BACK_ROLLER] = -rollerSpeed; 
}       

void FlapperStop(void) {
    motor[BACK_ROLLER] = 0; 
}

void FlapperReset(void) {
    motor[BACK_ROLLER] = rollerSpeed; 
}

void compass(void) {
    while (true){
    // Read 4-bit compass value from digital sensors
    int compassValue = SensorValue[compass_MSB] * 8 + 
                       SensorValue[compass_Bit2] * 4 + 
                       SensorValue[compass_Bit3] * 2 + 
                       SensorValue[compass_LSB];
    
    // Map compass value to heading (degrees)
    switch(compassValue) {
        case 7:  heading = 0;    break;   // West
        case 3:  heading = 45;   break;   // Southwest
        case 11: heading = 90;   break;   // South
        case 9:  heading = 135;  break;   // Southeast
        case 13: heading = 180;  break;   // East
        case 12: heading = 225;  break;   // Northeast
        case 14: heading = 270;  break;   // North
        case 6:  heading = 315;  break;   // Northwest
    }
    wait1Msec(50); // Adjust delay as needed
    }
}


void readIR(void) {
    while(true) {
          
        IR_A_val = SensorValue[IR_A] < 300 ? 0 : 1;
        IR_B_val = SensorValue[IR_B] < 300 ? 0 : 1; 
        IR_C_val = SensorValue[IR_C] < 300 ? 0 : 1;
        IR_D_val = SensorValue[IR_D] < 300 ? 0 : 1;
          
        wait1Msec(100);
    }
}

void readlimitswitch(void) {
    
    while(true) {
          
        limitswitchLB_val = SensorValue[limitswitchLB];
        limitswitchRB_val = SensorValue[limitswitchRB];
        limitswitchBall_val = SensorValue[limitswitchBall];
          
        wait1Msec(100);
    }
}

void scanBoundary(void) {

    while(true) {
        // Set boundary to true if any IR sensor reads 0
        if (IR_A_val == 0 || IR_B_val == 0 || IR_C_val == 0 || IR_D_val == 0) {
            isBoundary = true;
        } 
        else {
            isBoundary = false;
        }
        wait1Msec(50);
    }
}

void handleBoundary(void) {
        // Sensor Key:
        // IR_A_val = FR (Front Right), 0=boundary
        // IR_B_val = FL (Front Left), 0=boundary  
        // IR_C_val = BR (Back Right), 0=boundary
        // IR_D_val = BL (Back Left), 0=boundary
    
        // 1. Full frontal collision
        if (!IR_A_val && !IR_B_val) {         // FR+FL both detect boundary
            moveDistance(0.3, true);          // Back up
            turnDegrees(135);                 // Sharp left turn
        }
        // 2. Front-right corner
        else if (!IR_A_val && !IR_C_val) {    // FR+BR detect
            moveDistance(0.25, true);         // Gentle backup
            turnDegrees(120);                 // Hard left turn
        }
        // 3. Front-left corner  
        else if (!IR_B_val && !IR_D_val) {    // FL+BL detect
            moveDistance(0.25, true);
            turnDegrees(120, true);           // Hard right turn
        }
        // 4. Right side scrape
        else if (!IR_A_val) {                 // Only FR detects
            moveDistance(0.15, true);         // Tiny backup
            turnDegrees(60);                  // Moderate left turn
        }
        // 5. Left side scrape
        else if (!IR_B_val) {                 // Only FL detects
            moveDistance(0.15, true);
            turnDegrees(60, true);            // Moderate right turn
        }
        // 6. Rear collision
        else if (!IR_C_val || !IR_D_val) {    // Either back sensor
            moveDistance(0.3);                // Drive forward
            turnDegrees(90);                  // Quarter turn
        }
        // 7. Default safety
        else {
            moveDistance(0.2, true);          // Default backup
            turnDegrees(90);                  // Default turn
        }
    
        isBoundary = false;
        wait1Msec(200);  // Let sensors stabilize
    }


void convertSharpToDistance(tSensors sensor) {  
    int sum = 0;
    int samples = 5;
    
    for (int i = 0; i < samples; i++) {
        sum += SensorValue[sensor];
        wait1Msec(10);
    }

    int avgAnalog = sum / samples;
    float voltage = (avgAnalog * 5.0) / 4095.0;
    float distance = 80.0;

    if (avgAnalog == 0) {
        distance = 80.0;
    }
    else if (sensor == sharpFC) {
        distance = 25.99 / pow(voltage, 1.22);
    } else if (sensor == sharpFR) {
        distance = 28.37 / pow(voltage, 1.08);
    } else if (sensor == sharpFL) {
        distance = 26.82 / pow(voltage, 1.28);
    } else if (sensor == sharpBC) {
        distance = 10.02 / pow(voltage, 1.26);
    }

       
    if (sensor == sharpFC) {
        distFC = distance;
    } else if (sensor == sharpFR) {
        distFR = distance;
    } else if (sensor == sharpFL) {
        distFL = distance;
    } else if (sensor == sharpBC) {
        distBC = distance;
    }
      
}

void scanObstacle() {
    while(true) {
           
        if (distFC >= 10.0 && distFC <= 40.0) { //the robot detects an obstacle if the obstacle is between 10cm to 40cm away from the robot's front
            isFrontObstacle = true;
        }if (distBC >= 10.0 && distBC <= 40.0) { //the robot detects an obstacle if the obstacle is between 10cm to 40cm away from the robot's back
            isBackObstacle = true;
        } else { // the robot does not detect any obstacle
            isFrontObstacle = false;
            isBackObstacle = false;
        }
        wait1Msec(50);
    }
}

void handleObstacle() {
      
    if (robotMovingFront && isFrontObstacle) {
        // Obstacle avoidance logic for front obstacle
        moveDistance(0.3, true);
        turnDegrees(45);
    } else if (robotMovingBack && isBackObstacle) {
        // Obstacle avoidance logic for back obstacle
        moveDistance(0.3);
        turnDegrees(45, true);
    }
      
}

void scanBall() {
    
    while(true) {
          
        if (((distFL >= 10.0 && distFL <= 70.0) || 
            (distFR >= 10.0 && distFR <= 70.0)) &&
            !(distFC >= 10.0 && distFC <= 40.0)) {
            isBall = true;
        } else {
            isBall = false;
        }
        wait1Msec(50);
    }
}

void returnToBase(void) {
    while (true) {
        int target = 135;
        int heading = compass();
        int degree = heading - target;
        if(degree <0){
            turnDegrees(degree, true);
        }
        else{
            turnDegrees(degree);
        }
        moveDistance(MAX_DISTANCE / 100.0, true);

        if (heading == 135 && (limitswitchLB_val == 1 || limitswitchRB_val == 1) && 
            (IR_C_val == 0 && IR_D_val == 0)) {
            reachedBase = true;
            break;
        }
        wait1Msec(50);
    }
}

void deliver() {
    FlapperPush();
    wait1Msec(1000);
    FlapperStop();

    while(true) {
        if (limitswitchBall_val == 1) {
            FlapperReset();
            wait1Msec(1000);
            FlapperStop();
              
            isDelivered = true;
              
            break;
        }
        wait1Msec(100);
    }
}

void pickUpBall(void) {
    frontRollerIntake();
    while (true) {
        if (SensorValue[limitswitchBall] == 0) {
            frontRollerStop();
              
            isBallPicked = true;
              
            break;
        }
        wait1Msec(100);
    }
}    

void MoveTowardsBall() {
    while (true) {
          
        if (!((distFL >= 10.0 && distFL <= 70.0) || 
             (distFR >= 10.0 && distFR <= 70.0)) ||
             (distFC >= 10.0 && distFC <= 70.0)) {
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            isBall = false;
              
            break;
        }

        if (distFL >= 20.0 || distFR >= 20.0) {
            motor[motorLeft] = 30;
            motor[motorRight] = 30;
        } else {
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            
            if (distFL < 20.0) {
                  
                turnDegrees(30);
                moveDistance(0.1);
                  
            } else if (distFR < 20.0) {
                  
                turnDegrees(30,true);
                moveDistance(0.1);
                  
            }
        }
          
        wait1Msec(50);
    }
}

int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

void moveDistance(float distance, bool backward) {
    float realDistance = distance * 3.5;
    int targetTicks = distanceToTicks(realDistance);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = 1.28 * basePower;
        motor[motorRight] = -basePower;
          
        robotMovingBack = true;
        robotMovingFront = false;
          
    } else {
        motor[motorLeft] = -1.28 * basePower;
        motor[motorRight] = basePower;
          
        robotMovingFront = true;
        robotMovingBack = false;
          
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
           abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
      
    robotMovingFront = false;
    robotMovingBack = false;
      
}

void turnDegrees(float degrees, bool right) {
    float realDegrees = degrees * 2.5;
    float turningCircumference = PI * wheelBase;
    float arcLength = (realDegrees / 180.0) * turningCircumference;
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    int reducedSpeed = 127 * 0.7;

    if (right) {
        motor[motorLeft] = -reducedSpeed;
        motor[motorRight] = -reducedSpeed;
    } else {
        motor[motorLeft] = reducedSpeed;
        motor[motorRight] = reducedSpeed;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
           abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void checkBoundary(void) {

    if ((IR_A_val == 1 && IR_C_val == 1) && (IR_B_val == 0 && IR_D_val == 0)) {
        leftScanBoundary = true;
        rightScanBoundary = false;
    } else if ((IR_B_val == 1 && IR_D_val == 1) && (IR_A_val == 0 && IR_C_val == 0)) {
        rightScanBoundary = true;
        leftScanBoundary = false;
    } 

}

void scanSequence(void) {
    if (leftScanBoundary) {
        turnDegrees(20, true); //angle should not be too wide as the main objective to get the first ball 
        wait1Msec(1000);
        turnDegrees(20);
        wait1Msec(1000);
    } 
    else if (rightScanBoundary) {
        turnDegrees(20); //angle should not be too wide as the main objective to get the first ball
        wait1Msec(1000);
        turnDegrees(20, true);
        wait1Msec(1000);
    } 
}

// Main searching algorithm
void searchingAlgo() {
    checkBoundary(); // Update boundary flags once 
    wait1Msec(500); 
    moveDistance(1.0); //move up to the first row 
    wait1Msec(500); 

    if (leftScanBoundary && firstStart && !isDelivered) {
       // if robot is at the left boundary and in its 1st search and has not delieverd any balls
        scanSequence();     // Scan the 1st row 

        moveDistance(0.2); //assuming the 1st row is 0.2m, move up to the 2nd row
        wait1Msec(500);
        scanSequence(); //Scan the 2nd row
        wait1Msec(500);


        moveDistance(0.2); //move up to the 3rd row 
        wait1Msec(500);
        scanSequence(); //Scan the 3rd row 
        wait1Msec(500);

        firstStart = false;
    } 
    else if (rightScanBoundary && firstStart && !isDelivered) {
        // if robot is at the right boundary and in its 1st search and has not delieverd any balls
        scanSequence();      

        moveDistance(0.2); //assuming the 1st row is 0.2m, move up to the 2nd row
        wait1Msec(500);
        scanSequence(); //Scan the 2nd row
        wait1Msec(500);


        moveDistance(0.2); //move up to the 3rd row 
        wait1Msec(500);
        scanSequence(); //Scan the 3rd row 
        wait1Msec(500);

        firstStart = false;
    } 
    else if (!firstStart) {
        // Assume robot delivered the ball back to the same starting position 
        if (rightScanBoundary) {
            turnDegrees(90);   // Turn left if right boundary detected
            wait1Msec(500);
            moveDistance(1.0);    // Move forward

//          can be the code for spiral
        } 
        else if (leftScanBoundary) {
            turnDegrees(90,true);  // Turn right if left boundary detected
            wait1Msec(500);
            moveDistance(1.0);        // Move forward
//          can be the code for spiral
        }
    }
}


// ================================================================== Task definitions ==================================================================
task searchAlgoTask(void) {
    //Function will only run once 
    searchingAlgo();  
}

task scanBallTask(void) {
    scanBall();  
}

task scanBoundaryTask(void) {
    scanBoundary();  
}

task scanObstacleTask(void) {
    scanObstacle(); 
}

task returnToBaseTask(void) {
    returnToBase();  
}

task deliverTask(void) {
    deliver();  
}

task readIRTask(void) {
    //Function will run indefinitely due to while loop in readIR()
    readIR(); 
} 


task releaseExtraBallsTask(void) {
    frontRollerOutput();  
}

task check_current_headingTask(void) {
    compass();
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

// ================================================================== MAIN PROGRAM ==================================================================
task main() {
    // Initialize basic tasks
    startTask(check_current_headingTask);
    startTask(readIRTask);
    startTask(readSharpFC_Task);
    startTask(readSharpFR_Task);
    startTask(readSharpFL_Task);
    startTask(readSharpBC_Task);
    wait1Msec(500);

    while (true) {
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
        }
        wait1Msec(100);
    }
}
