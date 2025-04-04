#pragma config(Sensor, dgtl9, compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl10,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl11,  compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl12,  compass_MSB,    sensorDigitalIn)


float compass(){
	
	int num;
	num = SensorValue[compass_MSB]*pow(2,3) + SensorValue[compass_Bit2]*pow(2,2) + SensorValue[compass_Bit3]*2 + SensorValue[compass_LSB];
	switch(num){
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



task check_current_heading(){
	while(1){
		int heading = compass();
		 writeDebugStreamLine("compass", heading);
	}
}

// Move forward or backward by distance (in meters)
void moveDistance(float distance, bool backward = false) {
	float realDistance = distance *10; 
    int targetTicks = distanceToTicks(realDistance);
    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (backward) {
        motor[motorLeft] = basePower;
        motor[motorRight] = -basePower;
    } else {
        motor[motorLeft] = -basePower;
        motor[motorRight] = basePower;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

// Turn left or right by angle (in degrees)
void turnDegrees(float degrees, bool right = false) {
    //float realDegrees = degrees * 2.5;
    //float turningCircumference = 3.14159 * wheelBase;  // Full turning circle
    //float arcLength = (realDegrees / 180.0) * turningCircumference; // One wheel arc distance

    //Arc length formula : ArcLength=r*theta, where r is in radians 
    float angleRadian = degreesToRadians(degrees); 
    float arcLength = angleRadian * (wheelBase/2);
	
    int targetTicks = distanceToTicks(arcLength);

    SensorValue[LEFT_ENCODER] = 0;
    SensorValue[RIGHT_ENCODER] = 0;

    if (right) {
        motor[motorLeft] = -127;
        motor[motorRight] = -127;
    } else {
        motor[motorLeft] = 127;
        motor[motorRight] =127;
    }

    while (abs(SensorValue[LEFT_ENCODER]) < targetTicks && abs(SensorValue[RIGHT_ENCODER]) < targetTicks) {
        wait1Msec(10);
    }

    motor[motorLeft] = 0;
    motor[motorRight] = 0;
}

void backBase() {
    float heading = compass();  // Get current compass heading
    float degree;
    bool rightTurn;

    // Determine shortest turn direction to 270° (North)
    if (heading == 270) {
        rightTurn = true;  // Already facing North, no need to turn
        degree = 0;
    } 
    else if ((heading > 270 && heading <= 360) || (heading >= 0 && heading < 90)) {
        // If facing between (270-360) or (0-90) → Turn right
        rightTurn = true;
        degree = (heading > 270) ? heading - 270 : 270 - heading;
    } 
    else {
        // If facing between (90-270) → Turn left
        rightTurn = false;
        degree = 270 - heading;
    }

    // Turn towards 270° (North)
    if (degree > 0) {
        turnDegrees(degree, rightTurn);
    }

    // Move backward towards base
    moveDistance(300, true);

    // Check if the robot has reached the base using limit switches and IR sensors
    if (compass() == 270 && (SensorValue[limitswitchLB] == 0 || SensorValue[limitswitchRB] == 0) 
        && (SensorValue[IR_C] == 0 && SensorValue[IR_D] == 0)) { 
        reachedBase = true;
    }
}

