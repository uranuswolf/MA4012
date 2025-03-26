#pragma config(Sensor, dgtl10,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl8, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl7, compass_MSB,    sensorDigitalIn)

int compass(){
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
	case 14: return 270; 	//N
		break;
	case 6: return 315; 	//NW
		break;
	}
	return -1;
}

task check_current_orientation(){
	while(1){
		current_orientation = compass();
	}
}
