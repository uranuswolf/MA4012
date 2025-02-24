float pidControl(int setpoint, int currentSpeed) {
    // Scale setpoint to actual motor speed
    float expectedSpeed = (setpoint / 127.0) * maxMotorSpeed; // maxMotorSpeed is a predefined constant

    // Calculate error between desired speed and current speed
    float error = expectedSpeed - currentSpeed;

    integral += error;
    float derivative = error - previousError;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    // Limit motor power to 127 max
    if (output > 127) output = 127;
    if (output < -127) output = -127;

    return output;
}
 
 
