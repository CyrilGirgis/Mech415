
#include <cmath>
#include <iostream>
#include <fstream>

#include <windows.h> // for keyboard input

// user defined functions

#include "timer.h"
#include "ran.h"

#include "rotation.h" //rotation matrices 
#include "3D_graphics.h" // user functions for DirectX 3D graphics
#include "graphics.h" // graphics functions for various geometries
#include"myproject.h"
//screen set up section

// 3D graphics window size
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1600; // increase this to increase the window width
int HEIGHT_MAX = 900; // increase this to increase the window height
static double k;
// background colour for the scene
float BACK_R = 0.0; // red colour component (0 to 1)
float BACK_G = 0.0; // green colour component (0 to 1)
float BACK_B = 0.5; // blue colour component (0 to 1)

// default min and max viewing distances.
// objects closer than VMIN and farther than VMAX are not drawn (ie cannot be seen).
// note the ratio of VMAX/VMIN should be less than 60,000 for most graphics cards.
double VMIN = 1.0; // units of m (or whatever units you draw your object in)
double VMAX = 10000.0; // units of m

// directional light #0 (the default light) magnitude and direction
double LIGHT0_R = 0.8; // red (0 to 1)
double LIGHT0_G = 0.8; // green (0 to 1)
double LIGHT0_B = 0.8; // blue (0 to 1)
double LIGHT0_X = -1.0; // x direction
double LIGHT0_Y = -1.0; // y direction
double LIGHT0_Z = -1.0; // z direction
// (-1,-1,-1) -> towards the origin along the default viewing direction

int LIGHT0_SWITCH = 1; // 1 - on, 0 - off

// ambient light level
float AMBIENT_R = 0.7; // red (0 to 1)
float AMBIENT_G = 0.7; // green (0 to 1)
float AMBIENT_B = 0.7; // blue (0 to 1)

const double PI = 4 * atan(1.0);

using namespace std;


// output file for debugging / testing
// -- this essentially replaces cout for this program since cout doesn't work
ofstream dout("debug.txt");
// hold all the variables that describe your object you want to simulate

void draw_3D_graphics()
{
	static double Px, Py, Pz, roll, pitch, yaw;
	static int init = 0; // initialization flag

	static double t; // clock time from t0
	static double t0; // initial clock time



	double x1[7 + 1] = { 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0 };
	double u[4 + 1] = { 0.0 };
	double radius = 1.0;

	static robot *plane[100]; // an array of robot pointers
	static mesh m3("plane.x");
	static mesh m5("tiger.x");
	m5.Scale = 2.0;

	static int n = 1;
	static int i;
	static double c;

	m3.Scale = 0.2;
	// initalization section 
	if (!init) {

		// dynamic robot objects

		plane[0] = new robot(x1, u, radius, "naca-420.txt", 1);
		

		// this program has no explicit end point
		// so we will forgo the delete operator
		// -> the memory for these will be released
		// when the program ends.

		t0 = high_resolution_time(); // initial clock time (s)

		init = 1;
	} // end of initialization section



	// 2D orthogonal view in x-y coord 
	// the argument is the width and height of the graphics window
	// don't run this the same time as set_view()
	//	set_2D_view(5.0,5.0); // comment out the above if you use this

	static double fov;
	static double eye_point[3 + 1] = { 0.0 };
	static double lookat_point[3 + 1] = { 0.0 };
	static double up_dir[3 + 1] = { 0.0 };

	t = high_resolution_time() - t0;

	//3rd person view perspective following the plane

	fov = 3.14159 / 4; // 45 deg

	i = 1;
	/*
	eye_point[1] = (plane[i]->x[1] + cos(plane[i]->x[4]))/2;
	eye_point[2] = (plane[i]->x[2] + cos(plane[i]->x[4]))/2;
	eye_point[3] = (plane[i]->x[3])/2;
	*/
	eye_point[1] = plane[0]->x[1] - 5.0;
	eye_point[2] = plane[0]->x[2];
	eye_point[3] = plane[0]->x[3] + 2.0;

	lookat_point[1] = plane[0]->x[1];
	lookat_point[2] = plane[0]->x[2];
	lookat_point[3] = plane[0]->x[3];

	up_dir[3] = 1;

	set_view(eye_point, lookat_point, up_dir, fov);


	// draw the axes (red = x, y = green, z = blue)
	draw_XYZ(5.0);  // set axes of 5 m length

	// read clock time (resolution is 0.1 microseconds)
	t = high_resolution_time() - t0; // time since the program started (s)
	static mesh m2("background_z_up.x");
	static mesh m7("tiger.x");
	m2.Scale = 4.0;
	m2.draw(0, 0, 0, 0, 0, 0);
	//	robot1.move();
	//	robot2.move();

	double dt = 0.1;

	// read / calculate inputs
	//plane[0]->keyboard_input();
	c = plane[0]->calculate_lift();
	plane[0]->keyboard_input();
	//	robot1.keyboard_input();	
	//	robot2.keyboard_input();

	// non real-time simulation t_sim != t_clock / graphics time
	//	robot1.simulate_step(dt);
	//	robot2.simulate_step(dt);

	
	plane[0]->simulate_step(dt);



	// draw a mesh object ////////////////////////
	//	robot1.draw();
	//	robot2.draw();



	// x = [x y z th]
	Px = plane[0]->x[1];
	Py = plane[0]->x[2];// this is actually z
	Pz = plane[0]->x[3];// this is y

	yaw = 0; // yaw is rotation around the z-axis
	pitch = plane[0]->x[4];
	roll = plane[0]->x[7];
	text_xy("vx: ", 550, 60, 15);
	text_xy(plane[0]->x[6], 600, 60, 15); ////////////////mofied from////////////60 + 17 * 20, 15);

	text_xy("thrust%: ", 550, 90, 15);
	text_xy(plane[0]->u[1], 650, 90, 15);

	text_xy("pitch: ", 700, 60, 15);
	text_xy(sin(plane[0]->x[4]), 750, 60, 15);

	text_xy("vy: ", 1000, 90, 15);
	text_xy(plane[0]->x[5], 1050, 90, 15);

	text_xy("y-position: ", 800, 90, 15);
	text_xy(plane[0]->x[3], 900, 90, 15);

	m7.draw(Px + 5, Py, Pz, 0.0, 0.0, 0.0);
	m3.draw(Px, Py, Pz, yaw, pitch, roll);
	
		

}
