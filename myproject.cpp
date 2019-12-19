#include"myproject.h"
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
const double PI = 4 * atan(1.0);
ofstream test("test.csv");
ofstream a("AOA.txt");
ofstream ao("acceleration.txt");
ofstream c("Cl.txt");
void robot::keyboard_input()
{
	u[3] = 0;
	u[4] = 0;
	// you could also say += here

	if (KEY(VK_UP)) {
		u[1] += 2/(32.5*mass*2.0);
		if (u[1] > (32.5*mass*2.0))
			u[1] = 32.5*mass*2.0;
			

	}
	if (KEY(VK_PRIOR)) {//roll
		if (x[6] > 10){
			u[4] = -0.1;
			if (u[0] < 0){
				u[2] += 0.5;
			}
		}
	}

	if (KEY(VK_NEXT)) {//roll
		if (x[6] > 10){
			u[4] = 0.1;
			if (u[4] > 0){
				u[2] -= 0.5;
			}
		}
	}

	if (KEY(VK_DOWN)) {
		u[1] += -0.01;
		if (u[1] < 0.0)
			u[1] = 0.0;
	}

	

	

	if (KEY(VK_LEFT)) {
		u[3] = -0.1;
	}

	if (KEY(VK_RIGHT)) {
		u[3] = 0.1;

	}

}

// simulate equations for one time step with Euler's method
void robot::simulate_step(double dt)
{
	int i, N = 7;
	double xd[7 + 1];


	// calculate the derivative vector at time t
	xd[1] = x[6];//derivative calculated for x direction
	xd[2] = u[2];
	xd[3] = x[5];//x[3] is y position x[5] is velocity, for now im not using x[5] to calculate x[3] because its buggy
	xd[4] = u[3];
	//derivative calculated for vy
	xd[6] = calculate_accelerationx();
	xd[5] = 5*xd[6]*-1*sin(x[4]);
	xd[7] = u[4];//roll
	// Euler's equation
	for (i = 1; i <= N; i++) x[i] = x[i] + xd[i] * dt;
	if (x[3] < 10){
		x[5] = 0;
		x[3] = 10;
	}
	if (x[6] >= 35)  x[6] = 35;
	if (x[5] >= 35)	x[5] = 20;

	ao << "x[5]" << x[5] << endl;
	ao << "x[3]" << x[3] << endl;
	t = t + dt; // increment time
}


robot::robot(double x[], double u[], double radius, char filename[], double mass)
{
	this->x[1] = x[1];// x position
	this->x[2] = x[2];//y position
	this->x[3] = x[3];//z position
	this->x[4] = x[4];//pitch
	this->x[5] = x[5];//z  velocity
	this->x[6] = x[6];//x veolcity
	this->x[7] = x[7]; // roll
	this->u[1] = u[1];
	this->u[2] = u[2];//vy
	this->u[3] = u[3];//this is up and down
	this->u[4] = u[4];//roll
	this->mass = mass;
	t = 0.0;

	this->radius = radius;
	infile.open(filename);
	for (int i = 0; i < 100; i++){
		infile >> data[i][0];
		test << data[i][0] << ',';
		infile >> data[i][1];
		test << data[i][1] << endl;
	}
	infile.close();

}




robot::~robot()
{
}


void robot::draw()
{
	double Px, Py, Pz, yaw, pitch, roll;

	// x = [x y z th]
	Px = x[1];
	Py = x[2];
	Pz = x[3];
	if (Pz < 60)																				///////////watchout///
		Pz = 60;
	yaw = 0; // yaw is rotation around the z-axis
	pitch = x[4];
	roll = PI / 2;

	//	m1.draw(Px, Py, Pz, yaw, pitch, roll);

}
double robot::calculate_lift(){
	double lift;
	double cl = 0;

	for (int i = 0; i < 100; i++){

		a << PI*2.0*data[i][0] / 360.0 << "  " << -x[4] << " " << PI*2.0*data[i][0] / 360.0 - -x[4] << endl;
		if (abs(PI*2.0*data[i][0] / 360.0 - -x[4]) <= 0.001){
			cl = data[i][1];
			c << data[i][0] << " " << cl << endl;
			break;
		}
	
	}
	ao << "v[x] =" << x[6] << endl;
	if (x[4] > 0)
		cl = 0.4;
	else cl = 0.5;
	lift = 1* 0.5*x[6]*x[6] * 1.2*1.24;
	ao << "u[1] = " << u[1] << endl;
	ao << "lift " << lift << endl;
			////////////////////////watchout////////////////////////////////
	return lift;
}


double robot::calculate_accelerationy(){
	double a;

	a = (calculate_lift()*cos(x[4]) - mass*32.2 + 1*u[1] * -1*sin(x[4])) / mass;
	ao << "cosx[4]" << cos(x[4]);
	ao << "ay = " << a << endl;
	if (a >= 3) a = 3;									///////////////////////////////////watchout///////////////////////////////

	return a;
};

double robot::calculate_accelerationx(){
	double ax;
	ax = (u[1] * cos(x[4]) - calculate_lift()*-1*sin(x[4])) / mass;
	ao << "ax = " << ax << endl;

	if (ax >= 3) ax = 3;/////////////////////////////////////watchout////////////////////////////
	return ax;
};