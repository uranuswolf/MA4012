#pragma config(Sensor, in1, sharpSensor, sensorAnalog)
// Set `sharpSensor` to the correct analog port (e.g., `in1`)

task main() {
    int samples = 10; // Number of readings to average
    float sum = 0;

    while (true) {
        sum = 0; // Reset sum

        // Take multiple readings for averaging
        for (int i = 0; i < samples; i++) {
            sum += SensorValue[sharpSensor];
            wait1Msec(10);
        }

        float avgAnalog = sum / samples; // Compute average
        float voltage = (avgAnalog * 5.0) / 4095.0; // Convert ADC value to voltage

        // Print results to debug console
        writeDebugStreamLine("Analog: %.2f - Voltage: %.2f V", avgAnalog, voltage);

        wait1Msec(1000); // Wait before next reading
    }
}
