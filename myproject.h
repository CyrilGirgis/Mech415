
#include <cmath>
#include <iostream>
#include <fstream>

#include <windows.h> // for keyboard input

// user defined functions

#include "timer.h"
#include "ran.h"

//#include "rotation.h" //rotation matrices 
//#include "3D_graphics.h" // user functions for DirectX 3D graphics
//#include "graphics.h" // graphics functions for various geometries
using namespace std;

class robot {
	
public:
	
	double k;
	double data[100][2];
	double x[7 + 1]; // state variable x = [x y z th]
	double u[4 + 1]; // inputs u = [v vz w]
	double t;
	double calculate_lift();
	double radius;
	const double g = 9.81;
		double mass;
	ifstream infile;
	// for a member variable with non-default
	// constructor then you need to have a pointe to that
	// object and init to a dynamic object

	// in this case it might be more useful to pass the name of the 
	// mesh x-file to the robot constructor rather than
	// send a mesh pointer to the constructor

	robot(double x[], double u[], double radius,char filename[],double mass);
	double calculate_accelerationx();
	double calculate_accelerationy();

	~robot();

	void draw();

	// read the keyboard and move around
	void move();

	void keyboard_input();

	// simulate equations for one time step with Euler's method
	void simulate_step(double dt);

};