// Lab 1 Jordan Millett


// Compile using:
//      g++ filename.c -o filename -L "/usr/linux/public/labvolt" -l labvolt -l usb
// Make sure output file permissions are set to include x (execute)

#include "/home/jurden/Desktop/labvolt/inc/labvolt.h"
#include <unistd.h>
void Degrees(int Base, int Shoulder, int Elbow, int M4, int M5) {
	moverel(Base*(6500/360), Shoulder*(8600/360), Elbow*(8600/360), M4*(6500/360), M5*(6500/360));
}

int main(void)
{
        init();
        zero();               // Remember the home position
        Degrees(30,0,0,0,0);   // Rotate base 20 degrees clockwise
	Degrees(0,-20,20,20,-20);	// Move shoulder 20 degrees down, keep elbow still
	Degrees(0,0,15,15,-15);	// Move elbow 15 degrees toward base, keep wrist still
	Degrees(0,0,0,-20,-20);	// Rotate wrist 20 degrees CW
	Degrees(0,0,0,-20,20);	// Pitch wrist 20 degrees up
	sleep(5);
        gripperClose();       // Close the gripper
        nest();               // Return arm to home position

        shutdown();
        return 0;
}

