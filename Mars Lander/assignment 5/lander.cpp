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
#define Kp 1
#define BIG_delta 0.1
//Big_delta should be between 0 and 1
/*best
#define Kh 0.04
#define Kp 1
#define BIG_delta 0.1
*/
ofstream fout;

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
	static double error, h, pout;

	h = position.abs() - MARS_RADIUS;
	error = - 0.5 - Kh * h - velocity * position.norm();
	pout = Kp * error;
	if (pout < -BIG_delta)
		throttle = 0;
	else if (pout < 1 - BIG_delta)
		throttle = BIG_delta + pout;
	else
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

double Area = LANDER_SIZE*LANDER_SIZE*3.1416;
double parachute_area = 3.1416 * (2*LANDER_SIZE)*(2 * LANDER_SIZE);

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  ///
  vector3d new_position;
  static vector3d previous_position;
  vector3d _thrust, _drag, _gravity,_acceleration;
  double _density, _mass;
  _mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY*FUEL_DENSITY*fuel;
  _thrust = thrust_wrt_world();
  _density = atmospheric_density(position);
  _drag = velocity.norm() * (-0.5) * _density * DRAG_COEF_LANDER * Area * velocity.abs2();
  if(parachute_status == DEPLOYED)
  	_drag += velocity * (-0.5) * _density * DRAG_COEF_CHUTE * parachute_area * velocity.abs();
  _gravity = -position.norm() * MARS_MASS * GRAVITY * _mass / position.abs2();
  _acceleration = (_thrust + _drag + _gravity)/ _mass;
  
  if(INTERGRATION_METHOD == 0)//euler
  {
  	position += delta_t * velocity;
  	velocity += delta_t * _acceleration;
  }
  else//verlet
  {
  	if(simulation_time == 0)//euler for the 1st time step
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
        velocity = (position - previous_position)/delta_t;
    }
  }
  
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen



  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
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
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
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
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }

}
