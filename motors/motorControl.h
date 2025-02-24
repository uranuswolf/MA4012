#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

extern int maxMotorSpeed; // Maximum speed in encoder ticks per second

void controlMotors(int leftPower, int rightPower);
int getEncoderSpeed(int previousEncoder, int currentEncoder, int deltaTime);

#endif
