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
#pragma config(Sensor, dgtl10, compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl8,  compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl7,  compass_MSB,    sensorDigitalIn)
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
int IR_A_val = 1; // IR sensor A = 0 or 1, 0 means at boundary, 1 means not at boundary
int IR_B_val = 1; // IR sensor B = 0 or 1
int IR_C_val = 1; // IR sensor C = 0 or 1
int IR_D_val = 1; // IR sensor D = 0 or 1
bool isBoundary = false;  // Flag to indicate if the robot is at the boundary
bool isBall = false;  // Flag to indicate if the ball is detected
bool isFrontObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBackObstacle = false;  // Flag to indicate if an obstacle is detected
bool isBallPicked = false;  // Flag to indicate if the ball is picked up
bool reachedBase = false;  // Flag to indicate if the robot has reached the base
bool isDelivered = false;  // Flag to indicate if the ball is delivered
bool leftScanBoundary = false; //Flag to indicate if the left boundary is detected
bool rightScanBoundary = false; //Flag to indicate if the right boundary is detected
int heading; // Variable to store the compass heading
int limitswitchLB_val = 1; // Variable to store the value of the left limit switch, 1 means not pressed 
int limitswitchRB_val = 1; // Variable to store the value of the right limit switch
int limitswitchBall_val = 1; // Variable to store the value of the ball limit switch
float distFC = 0.0; // Variable to store the distance from the front center sharp sensor
float distFR = 0.0; // Variable to store the distance from the front right sharp sensor
float distFL = 0.0; // Variable to store the distance from the front left sharp sensor
float distBC = 0.0; // Variable to store the distance from the back center sharp sensor
int SharpFC_Value; // Variable to store the value of the front center sharp sensor
int SharpFR_Value; // Variable to store the value of the front right sharp sensor
int SharpFL_Value; // Variable to store the value of the front left sharp sensor
int SharpBC_Value; // Variable to store the value of the back center sharp sensor
bool robotMovingBack = false; // Flag to indicate if the robot is moving backwards
bool robotMovingFront = false; // Flag to indicate if the robot is moving forwards

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
const int MAX_DISTANCE = 300; // 300cm max distance for return

// ================================================================== Phase functions ==================================================================
void searchPhase(void) {
    startTask(searchAlgoTask);
    startTask(scanBallTask);
    startTask(scanBoundaryTask);
    startTask(scanObstacleTask);

    while (true) {
        AcquireMutex(mutex);
        if (isBall) {
            stopTask(searchAlgoTask);
            stopTask(scanBallTask);
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            isBall = false;
            currentState = COLLECT;
            ReleaseMutex(mutex);
            break;
        }
        ReleaseMutex(mutex);  // Release if condition not met
        AcquireMutex(mutex);
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) {
                stopTask(searchAlgoTask);
                stopTask(scanBallTask);
            }

            if (isBoundary) {
                ReleaseMutex(mutex);
                handleBoundary();
                wait1Msec(200);
                AcquireMutex(mutex);
                if (!isBoundary) {
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) {
                            stopTask(searchAlgoTask);
                            stopTask(scanBallTask);
                        }
                        ReleaseMutex(mutex);
                        handleObstacle();
                        wait1Msec(200);
                        AcquireMutex(mutex);
                        if (isFrontObstacle || isBackObstacle) {
                            ReleaseMutex(mutex);
                            handleObstacle();
                            AcquireMutex(mutex);
                        }
                    }
                }
                startTask(searchAlgoTask);
                startTask(scanBallTask);
                ReleaseMutex(mutex);
                wait1Msec(100);
                continue;
            }

            if (isFrontObstacle || isBackObstacle) {
                if (robotMovingBack) {
                    stopTask(searchAlgoTask);
                    stopTask(scanBallTask);
                }
                ReleaseMutex(mutex);
                handleObstacle();
                wait1Msec(200);
                AcquireMutex(mutex);
                if (isFrontObstacle || isBackObstacle) {
                    ReleaseMutex(mutex);
                    handleObstacle();
                    AcquireMutex(mutex);
                }
                startTask(searchAlgoTask);
                startTask(scanBallTask);
                ReleaseMutex(mutex);
                wait1Msec(100);
                continue;
            }
        }
        ReleaseMutex(mutex);
    }
}

void collectPhase(void) {
    startTask(scanBoundaryTask);
    startTask(scanObstacleTask);
    startTask(moveTowardsBallTask);
    startTask(pickUpBallTask);
    
    while(true) {
        AcquireMutex(mutex);
        if (!isBall) {
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            stopTask(moveTowardsBallTask);
            stopTask(pickUpBallTask);
            currentState = SEARCH;
            ReleaseMutex(mutex);
            break;
        }
        AcquireMutex(mutex);
        if (isBallPicked) {
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            stopTask(moveTowardsBallTask);
            stopTask(pickUpBallTask);
            currentState = RETURN;
            isBallPicked = false;
            ReleaseMutex(mutex);
            break;
        }
        AcquireMutex(mutex);
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) {
                stopTask(moveTowardsBallTask);
                stopTask(pickUpBallTask);
            }

            if (isBoundary) {
                ReleaseMutex(mutex);
                handleBoundary();
                wait1Msec(200);
                AcquireMutex(mutex);
                if (!isBoundary) {
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) {
                            stopTask(moveTowardsBallTask);
                            stopTask(pickUpBallTask);
                        }
                        ReleaseMutex(mutex);
                        handleObstacle();
                        wait1Msec(200);
                        AcquireMutex(mutex);
                        if (isFrontObstacle || isBackObstacle) {
                            ReleaseMutex(mutex);
                            handleObstacle();
                            AcquireMutex(mutex);
                        }
                    }
                }
                startTask(moveTowardsBallTask);
                startTask(pickUpBallTask);
                ReleaseMutex(mutex);
                wait1Msec(100);
                continue;
            }

            if (isFrontObstacle || isBackObstacle) {
                if (robotMovingBack) {
                    stopTask(moveTowardsBallTask);
                    stopTask(pickUpBallTask);
                }
                ReleaseMutex(mutex);
                handleObstacle();
                wait1Msec(200);
                AcquireMutex(mutex);
                if (isFrontObstacle || isBackObstacle) {
                    ReleaseMutex(mutex);
                    handleObstacle();
                    AcquireMutex(mutex);
                }
                startTask(moveTowardsBallTask);
                startTask(pickUpBallTask);
                ReleaseMutex(mutex);
                wait1Msec(100);
                continue;
            }
        }
        ReleaseMutex(mutex);
    }
}

void returnPhase(void) {
    startTask(scanBoundaryTask);
    startTask(scanObstacleTask);
    startTask(returnToBaseTask);
    startTask(releaseExtraBallsTask);
    
    while(true) {
        AcquireMutex(mutex);
        if (reachedBase) {
            stopTask(scanBoundaryTask);
            stopTask(scanObstacleTask);
            stopTask(returnToBaseTask);
            stopTask(releaseExtraBallsTask);
            currentState = DELIVER;
            reachedBase = false;
            ReleaseMutex(mutex);
            break;
        }
        
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) {
                stopTask(returnToBaseTask);
                stopTask(releaseExtraBallsTask);
            }

            if (isBoundary) {
                ReleaseMutex(mutex);
                handleBoundary();
                wait1Msec(200);
                AcquireMutex(mutex);
                if (!isBoundary) {
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) {
                            stopTask(returnToBaseTask);
                            stopTask(releaseExtraBallsTask);
                        }
                        ReleaseMutex(mutex);
                        handleObstacle();
                        wait1Msec(200);
                        AcquireMutex(mutex);
                        if (isFrontObstacle || isBackObstacle) {
                            ReleaseMutex(mutex);
                            handleObstacle();
                            AcquireMutex(mutex);
                        }
                    }
                }
                startTask(returnToBaseTask);
                startTask(releaseExtraBallsTask);
                ReleaseMutex(mutex);
                wait1Msec(100);
                continue;
            }

            if (isFrontObstacle || isBackObstacle) {
                if (robotMovingBack) {
                    stopTask(returnToBaseTask);
                    stopTask(releaseExtraBallsTask);
                }
                ReleaseMutex(mutex);
                handleObstacle();
                wait1Msec(200);
                AcquireMutex(mutex);
                if (isFrontObstacle || isBackObstacle) {
                    ReleaseMutex(mutex);
                    handleObstacle();
                    AcquireMutex(mutex);
                }
                startTask(returnToBaseTask);
                startTask(releaseExtraBallsTask);
                ReleaseMutex(mutex);
                wait1Msec(100);
                continue;
            }
        }
        ReleaseMutex(mutex);
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

int compass(void) {
    int num;
    num = SensorValue[compass_MSB]*8 + SensorValue[compass_Bit2]*4 + 
          SensorValue[compass_Bit3]*2 + SensorValue[compass_LSB];
    
    switch(num) {
        case 7: return 0;    // W
        case 3: return 45;   // SW
        case 11: return 90;  // S
        case 9: return 135;  // SE
        case 13: return 180; // E
        case 12: return 225; // NE
        case 14: return 270; // N
        case 6: return 315;  // NW
    }
    return -1;
}

void readIR(void) {
    while(true) {
        AcquireMutex(mutex);
        IR_A_val = SensorValue[IR_A] < 300 ? 0 : 1;
        IR_B_val = SensorValue[IR_B] < 300 ? 0 : 1; 
        IR_C_val = SensorValue[IR_C] < 300 ? 0 : 1;
        IR_D_val = SensorValue[IR_D] < 300 ? 0 : 1;
        ReleaseMutex(mutex);
        wait1Msec(50);
    }
}

void readlimitswitch(void) {
    while(true) {
        AcquireMutex(mutex);
        limitswitchLB_val = SensorValue[limitswitchLB];
        limitswitchRB_val = SensorValue[limitswitchRB];
        limitswitchBall_val = SensorValue[limitswitchBall];
        ReleaseMutex(mutex);
        wait1Msec(50);
    }
}

void scanBoundary(void) {
    while(true) {
        AcquireMutex(mutex);
        if ((IR_A_val == 0 && IR_B_val == 0) || (IR_A_val == 0 && IR_C_val == 0) || 
            (IR_A_val == 0 && IR_D_val == 0) || (IR_B_val == 0 && IR_C_val == 0) || 
            (IR_B_val == 0 && IR_D_val == 0) || (IR_C_val == 0 && IR_D_val == 0)) {
            isBoundary = true;
        } else {
            isBoundary = false;
        }
        ReleaseMutex(mutex);
        wait1Msec(100);
    }
}

void handleBoundary(void) {
    int IR_State = 0;
    AcquireMutex(mutex);
    if (IR_A_val == 0 && IR_B_val == 0 && IR_C_val == 1 && IR_D_val == 1) {
        IR_State = 1;
    } else if (IR_A_val == 0 && IR_B_val == 1 && IR_C_val == 0 && IR_D_val == 0) {
        IR_State = 2;
    } else if (IR_A_val == 1 && IR_B_val == 0 && IR_C_val == 1 && IR_D_val == 0) {
        IR_State = 3;
    } else if (IR_A_val == 1 && IR_B_val == 1 && IR_C_val == 0 && IR_D_val == 0) {
        IR_State = 4;
    }
    ReleaseMutex(mutex);
    
    switch (IR_State) {
        case 1:
            moveDistance(0.5, true);
            wait1Msec(1000);
            turnDegrees(180); 
            wait1Msec(1000);
            break;
        case 2:
            turnDegrees(90);
            break;
        case 3:
            turnDegrees(90, true);
            break;
        case 4:
            moveDistance(0.5);
            break;
        default:
            moveDistance(0.3, true);
            wait1Msec(1000);
            turnDegrees(45);
            break;
    }
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
    while(true) {
        AcquireMutex(mutex); 
        if (distFC >= 10.0 && distFC <= 80.0) {
            isFrontObstacle = true;
        } else if (distBC >= 10.0 && distBC <= 80.0) {
            isBackObstacle = true;
        } else {
            isFrontObstacle = false;
            isBackObstacle = false;
        }
        ReleaseMutex(mutex);
        wait1Msec(100);
    }
}

void handleObstacle() {
    AcquireMutex(mutex);
    if (robotMovingFront && isFrontObstacle) {
        // Obstacle avoidance logic for front obstacle
        moveDistance(0.3, true);
        turnDegrees(45);
    } else if (robotMovingBack && isBackObstacle) {
        // Obstacle avoidance logic for back obstacle
        moveDistance(0.3);
        turnDegrees(45, true);
    }
    ReleaseMutex(mutex);
}

void scanBall() {
    while(true) {
        AcquireMutex(mutex);
        if (((distFL >= 10.0 && distFL <= 70.0) || 
            (distFR >= 10.0 && distFR <= 70.0)) &&
            !(distFC >= 10.0 && distFC <= 70.0)) {
            isBall = true;
        } else {
            isBall = false;
        }
        ReleaseMutex(mutex);
        wait1Msec(100);
    }
}

void returnToBase(void) {
    while (true) {
        float degree = abs(270 - heading);
        turnDegrees(degree, true);
        moveDistance(MAX_DISTANCE / 100.0, true);

        if (heading == 270 && (limitswitchLB_val == 0 || limitswitchRB_val == 0) && 
            (IR_C_val == 0 && IR_D_val == 0)) {
            AcquireMutex(mutex);
            reachedBase = true;
            ReleaseMutex(mutex);
            break;
        }
        wait1Msec(100);
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
            AcquireMutex(mutex);
            isDelivered = true;
            ReleaseMutex(mutex);
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
            AcquireMutex(mutex);
            isBallPicked = true;
            ReleaseMutex(mutex);
            break;
        }
        wait1Msec(100);
    }
}    

void MoveTowardsBall() {
    while (true) {
        AcquireMutex(mutex);
        if (!((distFL >= 10.0 && distFL <= 70.0) || 
             (distFR >= 10.0 && distFR <= 70.0)) ||
             (distFC >= 10.0 && distFC <= 70.0)) {
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            isBall = false;
            ReleaseMutex(mutex);
            break;
        }

        if (distFL >= 20.0 || distFR >= 20.0) {
            motor[motorLeft] = 30;
            motor[motorRight] = 30;
        } else {
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            
            if (distFL < 20.0) {
                ReleaseMutex(mutex);
                turnDegrees(30, true);
                AcquireMutex(mutex);
            } else if (distFR < 20.0) {
                ReleaseMutex(mutex);
                turnDegrees(30);
                AcquireMutex(mutex);
            }
        }
        ReleaseMutex(mutex);
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
        AcquireMutex(mutex);
        robotMovingBack = true;
        robotMovingFront = false;
        ReleaseMutex(mutex);
    } else {
        motor[motorLeft] = -1.28 * basePower;
        motor[motorRight] = basePower;
        AcquireMutex(mutex);
        robotMovingFront = true;
        robotMovingBack = false;
        ReleaseMutex(mutex);
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && 
           abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
    AcquireMutex(mutex);
    robotMovingFront = false;
    robotMovingBack = false;
    ReleaseMutex(mutex);
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

void scanSequenceLeft(void) {
    turnDegrees(40, true);
    wait1Msec(1000);
    turnDegrees(40);
    wait1Msec(1000);
}

void scanSequenceRight(void) {
    turnDegrees(40);
    wait1Msec(1000);
    turnDegrees(40, true);
    wait1Msec(1000);
}

void searchingAlgoLeft(void) {
    moveDistance(1.2);
    wait1Msec(1000);
    scanSequenceLeft();

    moveDistance(0.3);
    wait1Msec(1000);
    scanSequenceLeft();

    moveDistance(0.3);
    wait1Msec(1000);
    scanSequenceLeft();

    moveDistance(1.8, true);
    wait1Msec(1000);

    turnDegrees(20, true);
    wait1Msec(1000);

    moveDistance(1.25);
    wait1Msec(1000);

    turnDegrees(40);
    wait1Msec(1000);

    turnDegrees(20, true);
    wait1Msec(1000);

    moveDistance(0.3);
    wait1Msec(1000);
    scanSequenceLeft();

    moveDistance(0.3);
    wait1Msec(1000);
    scanSequenceLeft();

    moveDistance(1.8, true);
    wait1Msec(1000);
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

    turnDegrees(20);
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
    checkBoundary();

    if (leftScanBoundary) {
        searchingAlgoLeft();
    } else if (rightScanBoundary) {
        searchingAlgoRight();
    } 
}

// ================================================================== Task definitions ==================================================================
task searchAlgoTask(void) {
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
    readIR(); 
} 


task releaseExtraBallsTask(void) {
    frontRollerOutput();  
}

task check_current_headingTask(void) {
    while(true) {
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

// ================================================================== MAIN PROGRAM ==================================================================
task main() {
    // Initialize basic tasks
    startTask(check_current_headingTask);
    startTask(readIRTask);

    // Start sensor reading tasks
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
            default:
                currentState = SEARCH;
        }
        wait1Msec(100);
    }
}
