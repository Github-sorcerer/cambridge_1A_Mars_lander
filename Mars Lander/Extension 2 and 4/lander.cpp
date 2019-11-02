// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#define INTERGRATION_METHOD 1
//0 Euler 1 verlet

#define Kh 0.04
#define Kh0 0.00001
#define Kh1 2.2
#define Kp 1
#define BIG_delta 0.1
//Big_delta should be between 0 and 1
/*best for senario 1
#define Kh 0.04
#define Kp 1
#define BIG_delta 0.1
*/
/*best for senario 5
#define Kh 0.017
#define Kp 1
#define BIG_delta 0.1
*/
ofstream fout;
//for autopilot
int re_enter_flag = 0;//0 if still doing periodic movement; 10  if lower apogee;  11 if lower perigee 2 if finished and entering exosphere
					  //improvement: several states braking in case of very high orbit; may add requirement 2a<constant
double perigee, apogee;
int at_apogee_flag;

void orientation_control(int dir=1)
// Three-axis stabilization to ensure the lander's base is always pointing downwards 
{
	vector3d up, left, out;
	double m[16];

	up = -velocity.norm()*dir; // this is the direction we want the lander's nose to point in, 

	// !!!!!!!!!!!!! HINT TO STUDENTS ATTEMPTING THE EXTENSION EXERCISES !!!!!!!!!!!!!!
	// For any-angle attitude control, we just need to set "up" to something different,
	// and leave the remainder of this function unchanged. For example, suppose we want
	// the attitude to be stabilized at stabilized_attitude_angle to the vertical in the
	// close-up view. So we need to rotate "up" by stabilized_attitude_angle degrees around
	// an axis perpendicular to the plane of the close-up view. This axis is given by the
	// vector product of "up"and "closeup_coords.right". To calculate the result of the
	// rotation, search the internet for information on the axis-angle rotation formula.

	// Set left to something perpendicular to up
	left.x = -up.y; left.y = up.x; left.z = 0.0;
	if (left.abs() < SMALL_NUM) { left.x = -up.z; left.y = 0.0; left.z = up.x; }
	left = left.norm();
	out = left ^ up;
	// Construct modelling matrix (rotation only) from these three vectors
	m[0] = out.x; m[1] = out.y; m[2] = out.z; m[3] = 0.0;
	m[4] = left.x; m[5] = left.y; m[6] = left.z; m[7] = 0.0;
	m[8] = up.x; m[9] = up.y; m[10] = up.z; m[11] = 0.0;
	m[12] = 0.0; m[13] = 0.0; m[14] = 0.0; m[15] = 1.0;
	// Decomponse into xyz Euler angles
	orientation = matrix_to_xyz_euler(m);
}
#define GravityMass 4.284066E13 
int judge_status(void)
{
	static vector3d ang_mom_vec;
	static double ang_momentum, energy;
	ang_mom_vec = position ^ velocity;
	ang_momentum = ang_mom_vec.abs();
	energy = 0.5 * velocity.abs2() - GravityMass / position.abs();
	if (ang_momentum == 0)
		return 2;//straight descent
	//cout << ang_momentum << " " << energy << endl;
	perigee = (GravityMass - sqrt(GravityMass*GravityMass + 2 * energy*ang_momentum*ang_momentum)) / (-2) / energy;
	apogee = (GravityMass + sqrt(GravityMass*GravityMass + 2 * energy*ang_momentum*ang_momentum)) / (-2) / energy;
	//cout << perigee << " " << apogee << endl;
	if (perigee<MARS_RADIUS+130000 && apogee<MARS_RADIUS+350000)
	    return 2;//descend
	if(apogee < MARS_RADIUS + 350000&&perigee>MARS_RADIUS+130000)
	    return 11;//lower perigee
	if (apogee > MARS_RADIUS + 2000000 && perigee < MARS_RADIUS + 200000)//decaying orbit with high energy
		return 12;//higher perigee
	return 10;//lower apogee
}

void re_enter_program(void)
{
	static double enter_height;
	enter_height = position.abs();
	if ((re_enter_flag == 11 && (enter_height > apogee - 5000)) || ((re_enter_flag == 10||re_enter_flag == 12)&& enter_height < perigee + 5000)||(re_enter_flag==10&&apogee-perigee<100000))
	{
		orientation_control(1);
		throttle = 1;
	}
	else if (enter_height > apogee - 8000&&re_enter_flag==12)
	{
		orientation_control(-1);
		throttle = 1;
	}
	else throttle = 0;
}

void autopilot(void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
	static double error, h, pout,error_horizontal;
    h = position.abs() - MARS_RADIUS;
	if (re_enter_flag != 2)//keep updating if not on re-enter orbit
	{
		re_enter_flag = judge_status();//use energy+angular momentum
	}
	//cout << perigee << " " << re_enter_flag << " ";//for debug
	if (re_enter_flag == 10||re_enter_flag==11||re_enter_flag==12)
	{
		re_enter_program();
		return;//no need for braking here
	}
	//cout<<perigee<<" "<<re_enter_flag<<endl;//for debug
	//error = - 0.5 - Kh * h - velocity * position.norm();
	//error = -0.5 - Kh0 * h * h - velocity * position.norm();
	if (velocity.abs() > 10.0)
		orientation_control();
	else
		attitude_stabilization();
	error = 1.7 - Kh1 * sqrt(h) - velocity * position.norm();
	pout = Kp * error;
	if (h < EXOSPHERE && parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute() && pout>0)//in case of early release
		parachute_status = DEPLOYED;
	//need:adjust relative speed to re-enter and horizontal brake
	//may use energy; when touching exosphere, first stage of re-enter successful
	//!may need extra boost when parachute deployed

	if (pout < -BIG_delta)
		throttle = 0;
	else if (pout < 1 - BIG_delta)
		throttle = BIG_delta + pout;
	else
		throttle = 1;
	if (position.abs() < MARS_RADIUS + 10000 && (velocity^position).abs() > 300)
		throttle = 1;
	if (!fout)
	{
		fout.open("trajectories.txt");
		fout << "write";
	}
	if (fout) { // file opened successfully
		fout << simulation_time << " " << h << " " << velocity * position.norm() << endl;
	}
}

double Area = LANDER_SIZE * LANDER_SIZE*3.1416;
double parachute_area = 3.1416 * (2 * LANDER_SIZE)*(2 * LANDER_SIZE);

void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
	///
	vector3d new_position;
	static vector3d previous_position;
	vector3d _thrust, _drag, _gravity, _acceleration;
	double _density, _mass;
	_mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY*fuel;
	_thrust = thrust_wrt_world();
	_density = atmospheric_density(position);
	_drag = velocity.norm() * (-0.5) * _density * DRAG_COEF_LANDER * Area * velocity.abs2();
	if (parachute_status == DEPLOYED)
		_drag += velocity * (-0.5) * _density * DRAG_COEF_CHUTE * parachute_area * velocity.abs();
	_gravity = -position.norm() * MARS_MASS * GRAVITY * _mass / position.abs2();
	_acceleration = (_thrust + _drag + _gravity) / _mass;

	if (INTERGRATION_METHOD == 0)//euler
	{
		position += delta_t * velocity;
		velocity += delta_t * _acceleration;
	}
	else//verlet
	{
		if (simulation_time == 0)//euler for the 1st time step
		{
			previous_position = position;
			position += delta_t * velocity;
			velocity += delta_t * _acceleration;
		}
		else
		{
			new_position = position * 2 - previous_position + _acceleration * delta_t*delta_t;
			previous_position = position;
			position = new_position;
			velocity = (position - previous_position) / delta_t;
		}
	}

	// Here we can apply an autopilot to adjust the thrust, parachute and attitude
	if (autopilot_enabled) autopilot();

	// Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
	if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation(void)
// Lander pose initialization - selects one of 10 possible scenarios
{
	// The parameters to set are:
	// position - in Cartesian planetary coordinate system (m)
	// velocity - in Cartesian planetary coordinate system (m/s)
	// orientation - in lander coordinate system (xyz Euler angles, degrees)
	// delta_t - the simulation time step
	// boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
	// scenario_description - a descriptive string for the help screen

	//initialization of autopilot variables
	re_enter_flag = 0;
	perigee = 0;
	apogee = 0;


	scenario_description[0] = "circular orbit";
	scenario_description[1] = "descent from 10km";
	scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
	scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
	scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
	scenario_description[5] = "descent from 200km";
	scenario_description[6] = "aerostationary";
	scenario_description[7] = "";
	scenario_description[8] = "";
	scenario_description[9] = "";

	switch (scenario) {

	case 0:
		// a circular equatorial orbit
		position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
		velocity = vector3d(0.0, -3247.087385863725, 0.0);
		orientation = vector3d(0.0, 90.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 1:
		// a descent from rest at 10km altitude
		position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
		velocity = vector3d(0.0, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = true;
		autopilot_enabled = false;
		break;

	case 2:
		// an elliptical polar orbit
		position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
		velocity = vector3d(3500.0, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 3:
		// polar surface launch at escape velocity (but drag prevents escape)
		position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
		velocity = vector3d(0.0, 0.0, 5027.0);
		orientation = vector3d(0.0, 0.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 4:
		// an elliptical orbit that clips the atmosphere each time round, losing energy
		position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
		velocity = vector3d(4000.0, 0.0, 0.0);
		orientation = vector3d(0.0, 90.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 5:
		// a descent from rest at the edge of the exosphere
		position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
		velocity = vector3d(0.0, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = true;
		autopilot_enabled = false;
		break;

	case 6:
		position = vector3d(0.0, 20429635.87, 0.0);
		velocity = vector3d(-1448.097, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = true;
		autopilot_enabled = false;
		break;

	case 7:
		break;

	case 8:
		break;

	case 9:
		break;

	}

}
