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
#pragma config(Motor,  port3,  motorLeft,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port2,  motorRight,     tmotorVex393_MC29, openLoop)
#pragma config(Sensor, dgtl1,  RIGHT_ENCODER,  sensorQuadEncoder) 
#pragma config(Sensor, dgtl3,  LEFT_ENCODER,   sensorQuadEncoder) 

// Sensor MACROS
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

const float PI = 3.14159265359;
const float WHEEL_DIAMETER = 0.06985;
const float WHEEL_BASE = 0.188;
const int TICKS_PER_REV = 90;
const int BASE_POWER = 30;
const float DISTANCE_CORRECTION_FACTOR = 3.85;
const float OFFSET_POWER_FOR_LEFT_MOTOR = 1.28;


const float CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float DISTANCE_PER_TICK = CIRCUMFERENCE / TICKS_PER_REV;

// Sensor Data Structure
typedef struct {
    float distFC;
    float distFR;
    float distFL;
    float distBC;
} DistanceSensors;

// Status Flags Structure
typedef struct {
    bool isBoundary;
    bool isFrontObstacle;
    bool isBackObstacle;
} StatusFlags;

// Global Variables
DistanceSensors distances;
StatusFlags status;
int limitSwitches[2];
bool IR_A_value;
bool IR_B_value;
bool IR_C_value;
bool IR_D_value;

// Helper Functions
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



    // Read sharp sensors
    distances.distFC = getSharpDistance(sharpFC, SensorValue[sharpFC]);
    distances.distFR = getSharpDistance(sharpFR, SensorValue[sharpFR]);
    distances.distFL = getSharpDistance(sharpFL, SensorValue[sharpFL]);
    distances.distBC = getSharpDistance(sharpBC, SensorValue[sharpBC]);

    // Update status flags
    status.isBoundary = (IR_A_value || IR_B_value ||IR_C_value || IR_D_value);
    status.isFrontObstacle = (distances.distFC < 30.0);
    status.isBackObstacle = (distances.distBC < 30.0);
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

// Test Task
task testObstacleBoundaryDetection() {
    while(true) {
        readSensors();
        
        // Display sensor readings
        writeDebugStreamLine("\n--- SENSOR READINGS ---");
        writeDebugStreamLine("Distances: FC=%.1f FR=%.1f FL=%.1f BC=%.1f",
                           distances.distFC, distances.distFR, distances.distFL, distances.distBC);
        writeDebugStreamLine("Status: Boundary=%d FrontObst=%d BackObst=%d",
                           status.isBoundary, status.isFrontObstacle, status.isBackObstacle);
        
        
        // Boundary detection has priority
        if (status.isBoundary) {
            handleBoundary();
        } 
        // Then obstacle detection
        else if (status.isFrontObstacle || status.isBackObstacle) {
            handleObstacle();
        }
        
        wait1Msec(100);
    }
}

task main() {
    // Start the test task
    startTask(testObstacleBoundaryDetection);
    
    // Keep main task running
    while(true) {
        wait1Msec(1000);
    }
}