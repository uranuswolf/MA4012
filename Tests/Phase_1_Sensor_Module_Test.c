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
#pragma config(Sensor, dgtl8,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11, compass_MSB,    sensorDigitalIn)

#define SAMPLE_SIZE 30  // Increased from 5 for more stable readings
#define DEBUG_RAW_VALUES false  // Set to true to see raw sensor values

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

float getSharpDistance(tSensors sensor, int analogValue) {
    if (analogValue == 0) return 80.0; // Maximum distance
    
    float voltage = (analogValue * 5.0) / 4095.0; // Convert to voltage
    
    // Different calibration formulas for each sensor
    if (sensor == sharpFC) return 25.99 / pow(voltage, 1.22);
    if (sensor == sharpFR) return 28.37 / pow(voltage, 1.08);
    if (sensor == sharpFL) return 26.82 / pow(voltage, 1.28);
    if (sensor == sharpBC) return 10.02 / pow(voltage, 1.26);
    
    return 80.0; // Default if sensor not matched
}

void readSensors() {
    // Read IR sensors
    for(int i = 0; i < 4; i++) {
        IR_values[i] = SensorValue[IR_A + i] < 300 ? 0 : 1;
    }

    // Read limit switches
    limitSwitches[0] = SensorValue[limitswitchLB];
    limitSwitches[1] = SensorValue[limitswitchRB];
    limitSwitches[2] = SensorValue[limitswitchBall];

    // Read compass
    heading = SensorValue[compass_MSB] * 8 + 
             SensorValue[compass_Bit2] * 4 + 
             SensorValue[compass_Bit3] * 2 + 
             SensorValue[compass_LSB];

    // Read sharp sensors with improved averaging
    tSensors sharpSensors[] = {sharpFC, sharpFR, sharpFL, sharpBC};
    float* distPtr[] = {&distances.distFC, &distances.distFR, 
                       &distances.distFL, &distances.distBC};
    
    for(int i = 0; i < 4; i++) {
        int sum = 0;
        for(int j = 0; j < SAMPLE_SIZE; j++) {
            sum += SensorValue[sharpSensors[i]];
            wait1Msec(10); // Increased delay between samples
        }
        
        int avgValue = sum / SAMPLE_SIZE;
        
        #if DEBUG_RAW_VALUES
        writeDebugStreamLine("Raw Sharp %d: %d", i, avgValue);
        #endif
        
        *distPtr[i] = getSharpDistance(sharpSensors[i], avgValue);
    }

    // Update status flags
    status.isBoundary = (IR_values[0] == 0 || IR_values[1] == 0 || 
                       IR_values[2] == 0 || IR_values[3] == 0);
    status.isFrontObstacle = (distances.distFC >= 10.0 && distances.distFC <= 40.0);
    status.isBackObstacle = (distances.distBC >= 10.0 && distances.distBC <= 40.0);
    status.isBallPicked = (limitSwitches[2] == 0);
    status.panRight = (IR_values[1] == 0 || IR_values[3] == 0);
    status.isBall = ((distances.distFL >= 10.0 && distances.distFL <= 70.0) || 
                   (distances.distFR >= 10.0 && distances.distFR <= 70.0)) &&
                  !(distances.distFC >= 10.0 && distances.distFC <= 40.0);
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
        
        wait1Msec(500); // Reduced delay for more frequent updates
    }
}

task main() {
    // Initialize debug stream
    bNxtLCDStatusDisplay = 2; // Enable debug stream
    clearDebugStream();
    
    // Run sensor test
    testSensorModule();
}