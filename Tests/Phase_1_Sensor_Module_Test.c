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
#pragma config(Sensor, dgtl8,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11, compass_MSB,    sensorDigitalIn)


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
} StatusFlags;

DistanceSensors distances;
StatusFlags status;
int IR_values[4];
int limitSwitches[3];
int heading;
bool panRightInitialized = false;

typedef enum RobotState {
    SEARCH,
    COLLECT,
    RETURN,
    DELIVER
} RobotState;

RobotState currentState = SEARCH;

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


float getSharpDistance(tSensors sensor, int analogValue) {
    if (analogValue == 0) return 90.0; // max range

    float voltage = (analogValue * 5.0) / 4095.0; // ADC to voltage

    // Calibration for each sensor
    if (sensor == sharpFC) return 25.99 / pow(voltage, 1.22);
    if (sensor == sharpFR) return 28.37 / pow(voltage, 1.08);
    if (sensor == sharpFL) return 26.82 / pow(voltage, 1.28);
    if (sensor == sharpBC) return 10.02 / pow(voltage, 1.26);

    return 80.0; // default if unknown sensor
}

void readSensors() {
    // Read IR sensors
    tSensors IR_ports[4] = {IR_A, IR_B, IR_C, IR_D};
    for (int i = 0; i < 4; i++) {
        IR_values[i] = SensorValue[IR_ports[i]] < 2700 ? 1 : 0;
    }

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
    status.isBoundary = (IR_values[0] || IR_values[1]  || 
                         IR_values[2] || IR_values[3] );

    status.isFrontObstacle = (distances.distFC < 30.0);

    status.isBackObstacle = (distances.distBC < 30.0);

    status.isBall = ((distances.distFL <= 50) || 
    (distances.distFR <=50)) &&
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
            (IR_values[2] == 0 && IR_values[3] == 0)) {
            status.reachedBase = true;
        }
    }

}

void testSensorModule() {
    while(true) {
        readSensors();
        
        // Display all sensor readings
        writeDebugStreamLine("\n--- SENSOR READINGS ---");
        writeDebugStreamLine("IR Sensors: FR=%d FL=%d BR=%d BL=%d", 
                           IR_values[0], IR_values[1], IR_values[2], IR_values[3]);
        writeDebugStreamLine("Limit Switches: LB=%d RB=%d Ball=%d",
                           limitSwitches[0], limitSwitches[1], limitSwitches[2]);
        writeDebugStreamLine("Compass Heading: %d", compass(heading));
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

task main() {
    // Clear the debug stream (no need for NXT LCD display setting)
    clearDebugStream();
    
    // Run the sensor test
    testSensorModule();
}