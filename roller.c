#pragma config(Motor, port9, backRoller, tmotorVex393_MC29, openLoop)

// Function to activate back roller
void activateBackRoller(int speed) {
    motor[backRoller] = speed;
}

// Function to stop back roller
void stopBackRoller() {
    motor[backRoller] = 0;
}

// Main task to test motor functions
task main() {
    activateBackRoller(127);  // Activate back roller at 50% speed
    wait1Msec(15000);         // Wait for 2 seconds
    stopBackRoller();        // Stop back roller
}
