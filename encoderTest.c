#pragma config(Sensor, dgtl1, LEFT_ENCODER, sensorQuadEncoder)
#pragma config(Sensor, dgtl3, RIGHT_ENCODER, sensorQuadEncoder)

task main() {
    int initialLeftTicks = 0;
    int initialRightTicks = 0;
    int finalLeftTicks;
    int finalRightTicks;
    int leftTicksPerSecond;
    int rightTicksPerSecond;

    // Set motors to full power for forward movement
    motor[port4] = 127;  // Left motor at full power
    motor[port5] = -127; // Right motor at full reverse power

    // Move forward for 5 seconds
    wait1Msec(5000);  // Wait for 5 seconds

    // Get initial encoder values after 5 seconds 
    initialLeftTicks = LEFT_ENCODER;
    initialRightTicks = RIGHT_ENCODER;

    wait1Msec(1000); // Wait for 1 second

    // Get final encoder values
    finalLeftTicks = LEFT_ENCODER;
    finalRightTicks = RIGHT_ENCODER;

    // Calculate ticks per second for both motors
    leftTicksPerSecond = finalLeftTicks - initialLeftTicks;
    rightTicksPerSecond = finalRightTicks - initialRightTicks;

    // Stop the motors
    motor[port4] = 0;
    motor[port5] = 0;

    // Print the actual speed in ticks per second
    writeDebugStreamLine("Move Forward: Left ticks per second: %d, Right ticks per second: %d", leftTicksPerSecond, rightTicksPerSecond);

    