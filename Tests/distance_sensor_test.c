#pragma config(Sensor, in8,    sharpFC,        sensorAnalog)
#pragma config(Sensor, in2,    sharpFL,        sensorAnalog)
#pragma config(Sensor, in7,    sharpBC,        sensorAnalog)
#pragma config(Sensor, in1,    sharpFR,        sensorAnalog)

#define SAMPLE_SIZE 5

float getDistance(tSensors sensorPort, int sensorIndex) {
    int sum = 0;
    for(int i = 0; i < SAMPLE_SIZE; i++) {
        sum += SensorValue[sensorPort];
        wait1Msec(2);
    }

    float voltage = ((sum / SAMPLE_SIZE) * 5.0) / 4095.0;

    // Return distance based on sensor index
    if (voltage == 0) return 80.0;
    switch(sensorIndex) {
        case 0: return 25.99 / pow(voltage, 1.22); // FC
        case 1: return 28.37 / pow(voltage, 1.08); // FR
        case 2: return 26.82 / pow(voltage, 1.28); // FL
        case 3: return 10.02 / pow(voltage, 1.26); // BC
    }
    return -1;
}

task main() {
    tSensors sensors[] = {sharpFC, sharpFR, sharpFL, sharpBC};
    string sensorNames[] = {"FC", "FR", "FL", "BC"};

    while(true) {
        writeDebugStreamLine("--- Distance Sensor Test ---");
        for(int i = 0; i < 4; i++) {
            float dist = getDistance(sensors[i], i);
            writeDebugStreamLine("Sensor %s: %.2f cm", sensorNames[i], dist);
        }
        writeDebugStreamLine("");
        wait1Msec(1000);
    }
}
