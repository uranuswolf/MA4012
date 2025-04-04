#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)
#pragma config(Sensor, dgtl11,    IR_A,           sensorDigitalIn)
#pragma config(Sensor, dgtl12,    IR_B,           sensorDigitalIn)
#pragma config(Sensor, dgtl5,    IR_C,           sensorDigitalIn)
#pragma config(Sensor, dgtl6,    IR_D,           sensorDigitalIn)
#pragma config(Sensor, dgtl6,  limitswitchLB,  sensorDigitalIn)
#pragma config(Sensor, dgtl5,  limitswitchRB,  sensorDigitalIn)
#pragma config(Sensor, dgtl7,  limitswitchBall,sensorDigitalIn)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder) 
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder) 
#pragma config(Sensor, in3, compass_LSB,    sensorAnalog)
#pragma config(Sensor, in4,  compass_Bit3,   sensorAnalog)
#pragma config(Sensor, in5,  compass_Bit2,   sensorAnalog)
#pragma config(Sensor, in6,  compass_MSB,    sensorAnalog)
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,  FRONT_ROLLER,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,  BACK_ROLLER,    tmotorVex393_MC29, openLoop)

typedef enum {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
} RobotState;

#define PI 3.14159265359

// Global variables
bool isBoundary = false;
bool isBall = false;
bool isFrontObstacle = false;
bool isBackObstacle = false;
bool robotMovingBack = false;
bool robotMovingFront = false;
bool leftScanBoundary = false;
bool rightScanBoundary = false;

int IR_A_val = 1;
int IR_B_val = 1;
int IR_C_val = 1;
int IR_D_val = 1;

float distFC = 0.0;
float distFR = 0.0;
float distFL = 0.0;
float distBC = 0.0;

int heading;

// Constants
const int basePower = 50;
const float wheelDiameter = 0.06985;
const float wheelCircumference = wheelDiameter * PI; // meters
const int ticksPerRevolution = 90;
const float distancePerTick = wheelCircumference / ticksPerRevolution;
const float wheelBase = 0.188;



// Function prototypes
void moveDistance(float distance, bool backward = false);
void turnDegrees(float degrees, bool right = false);
int distanceToTicks(float distance);
void searchPhase(void);
void handleBoundary(void);
void handleObstacle(void);
void scanBoundary(void);
void scanObstacle(void);
void scanBall(void);
void readIR(void);
void checkBoundary(void);
void scanSequenceLeft(void);
void scanSequenceRight(void);
void searchingAlgoLeft(void);
void searchingAlgoRight(void);
void searchingAlgo(void);
void convertSharpToDistance(tSensors sensor);
int compass(void);

// Task prototypes
task searchAlgoTask(void);
task scanBallTask(void);
task scanBoundaryTask(void);
task scanObstacleTask(void);
task readSharpFC_Task(void);
task readSharpFR_Task(void);
task readSharpFL_Task(void);
task readSharpBC_Task(void);
task check_current_headingTask(void);
task readIRTask(void);

//Phase definition 
void searchPhase(void) {
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
        if (isBoundary || isFrontObstacle || isBackObstacle) {
            if (!robotMovingBack) { //means the robot is either stopping or moving forward 
                stopTask(searchAlgoTask);
                stopTask(scanBallTask);
            }

            if (isBoundary) {
                handleBoundary();
                wait1Msec(200);
                if (!isBoundary) { //check if boundary is cleared
                    if (isFrontObstacle || isBackObstacle) {
                        if (robotMovingBack) { //means robot is moving backwards
                            stopTask(searchAlgoTask);
                            stopTask(scanBallTask);
                        }
                        handleObstacle();
                        wait1Msec(200);
                        if (isFrontObstacle || isBackObstacle) { //check if obstacle is cleared
                            handleObstacle();
                        }
                    }
                }// if boundary is not cleared 
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

// Helper functions implementation
int distanceToTicks(float distance) {
    return distance / distancePerTick;
}

void moveDistance(float distance, bool backward = false) {
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

void handleObstacle() {
    AcquireMutex(mutex);
    if (robotMovingFront && isFrontObstacle) {
        moveDistance(0.3, true);
        turnDegrees(45);
    } else if (robotMovingBack && isBackObstacle) {
        moveDistance(0.3);
        turnDegrees(45, true);
    }
    ReleaseMutex(mutex);
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


float compass(){
	int MSBit = SensorValue[compass_MSB] > 0 ? 1 : 0;;
	int Bittwo = SensorValue[compass_Bit2]> 0 ? 1 : 0;;
	int Bitthree = SensorValue[compass_Bit3]> 0 ? 1 : 0;;
	int LSBit = SensorValue[compass_LSB] > 0 ? 1 : 0;;
	int num;
	num = MSBit*pow(2,3) + Bittwo*pow(2,2) + Bitthree*2 + LSBit;
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
	case 14: return 270; 	//W
		break;
	case 6: return 315; 	//NW
		break;
	}
	return -1;
}


void readIR(void) {
    while(true) {
        AcquireMutex(mutex);
        IR_A_val = SensorValue[IR_A] 
        IR_B_val = SensorValue[IR_B] 
        IR_C_val = SensorValue[IR_C] 
        IR_D_val = SensorValue[IR_D] 
        ReleaseMutex(mutex);
        wait1Msec(50);
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

// Task implementations
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

task check_current_headingTask(void) {
    while(true) {
        heading = compass();
        wait1Msec(100);
    }
}

task readIRTask(void) {
    readIR(); 
}

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

    // Run search phase
    searchPhase();
}