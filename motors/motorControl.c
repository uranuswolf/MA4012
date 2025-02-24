// motorControl.c - Function to control the motors

void controlMotors(int leftSpeed, int rightSpeed)
{
    motor[leftmotor] = leftSpeed;
    motor[rightmotor] = rightSpeed;
}
