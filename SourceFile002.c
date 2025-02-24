#pragma config(Sensor, dgtl2,  encoderL1,      sensorQuadEncoder, dgtl3)
#pragma config(Sensor, dgtl7,  encoderR1,      sensorQuadEncoder, dgtl8)
#pragma config(Motor,  port2,  rightmotor,     tmotorVex393_MC29, openLoop, encoderPort, dgtl7)
#pragma config(Motor,  port3,  leftmotor,      tmotorVex393_MC29, openLoop, encoderPort, dgtl2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
    // Reset encoders
    SensorValue[encoderR1] = 0;
    SensorValue[encoderL1] = 0;
    
    // Start motors
    motor[leftmotor] = 50;
    motor[rightmotor] = -50;
    
    // Run for 3 seconds
    clearTimer(T1);
    while(time1[T1] < 3000) // Run for 3000ms (3 seconds)
    {
        // Display encoder values on the debug stream
        writeDebugStreamLine("Left Encoder: %d", SensorValue[encoderL1]);
        writeDebugStreamLine("Right Encoder: %d", SensorValue[encoderR1]);
        wait1Msec(100); // Small delay to reduce spam in debug stream
    }
    
    // Stop motors
    motor[leftmotor] = 0;
    motor[rightmotor] = 0;
}
