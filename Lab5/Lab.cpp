// Lab 1 Jordan Millett


// Compile using:
//      g++ filename.c -o filename -L "/usr/linux/public/labvolt" -l labvolt -l usb
// Make sure output file permissions are set to include x (execute)

#include "/home/jurden/Desktop/417Lab/labvolt/inc/labvolt.h"
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <cmath>

using namespace std;

void Degrees(int Base, int Shoulder, int Elbow, int M4, int M5) {
	moverel(Base*(6500/360), Shoulder*(8600/360), Elbow*(8600/360), M4*(6500/360), M5*(6500/360));
}

int main(void)
{
	int x=1,p,r,t;
	int a,b,c,d,e;
	int A=0,B=0,C=0,D=0,E=0;
	init();
	zero();               // Remember the home position
	Degrees(0,-132,-3,0,0);
	while(x){
		cout << "Enter 0 to quit, 1 to enter angles" << endl;
		cin >> x;
		if(x!=0){
			cout << endl << "Input 5 joint angles" << endl;
			cin >> a >> b >> c >> p >> r;		
			//b-=132;
			//c-=135;
			c-=b;
			d=-r;
			e=-r;
			d+=p;
			e-=p;
			d+=c;
			e-=c;
			
		//	d+=p;
		//	d+=r;
		//	e+=p;
		//	e-=r;
			Degrees(a-A,b-B,c-C,d-D,e-E);
			A=a;
			B=b;
			C=c;
			D=d;
			E=e;
		}
	}
		
		
		
	//gripperClose();       // Close the gripper
    nest();               // Return arm to home position

    shutdown();
    return 0;
}

