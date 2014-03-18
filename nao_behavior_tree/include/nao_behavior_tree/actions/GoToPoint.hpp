/*
 * GoToPoint.hpp
 *
 *  Created on: Mar 17, 2014
 *      Author: Olivier BALLAND
 */


#ifndef GOTOPOINT_HPP
#define GOTOPOINT_HPP


// Command publisher
ros::Publisher cmd_pub;

// Motion proxy
AL::ALMotionProxy* motion_proxy_ptr;

// Use sensors
bool useSensorValues = true; // use Magnetic Rotary Encoder (MRE) sensor values

// Robot parameters
struct Robot
{
	std::string id;
	double x,y;
	double vx,vy;
	double theta;
};
Robot r;

// Point
struct Point
{
	double x,y;
};
Point p;

// Bearing
double dtheta,theta_temp;

// Velocity threshold
double velThreshold = 0.03;

// Controller parameters
const double rho = 0.5;
const double k = 1;
const double distThreshold = 0.1;
const double angleThreshold = 10;


#endif /* GOTOPOINT_HPP */
