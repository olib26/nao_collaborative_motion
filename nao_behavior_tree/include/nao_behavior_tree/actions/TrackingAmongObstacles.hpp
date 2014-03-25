/*
 * TrackingAmongObstacles.hpp
 *
 *  Created on: Mar 04, 2014
 *      Author: Olivier BALLAND
 */


#ifndef TRACKINGAMONGOBSTACLES_HPP
#define TRACKINGAMONGOBSTACLES_HPP


// Robot: Odometry and Id
struct Robot{
	std::string id;
	double x,y,theta;
	double vx,vy;
};
Robot r1,r2;

// Optimal velocity
ros::Subscriber vel_sub;
nao_behavior_tree::Velocity V;

// Motion proxy
AL::ALMotionProxy* motion_proxy_ptr;

// Walker publisher
ros::Publisher cmd_pub;

// Angles publisher
ros::Publisher bearing_pub;

// Image size
int height,width;
const int cutHeight = 150;

// Camera parameters
const double VFOV = 47.6*M_PI/180;
const double HFOV = 60.9*M_PI/180;

// Point in integral image
struct Point
{
	int x,y;
	Point(int x,int y) : x(x),y(y){}
};

// Detector frame size
int sx,sy;
const int sx_min = 20;
const int sy_min = 30;
const int sx_max = 200;
const int sy_max = 300;


// Diffusion standard deviation
const int sigma_diffusion = 50; // Position
const int s_diffusion = 10; // Size

// Object coordinate
int x,y;
double depth = INFINITY;

// Number of particles
const int N = 2000;

// Generate N random particles
struct Particle{
	int x,y;
	int sx,sy;
	double w;
};
Particle* particles = new Particle[N];

// Minimum weight
const double eps = 1E-3;

// Variance Threshold
const double StD_max = 4000;

// Robot detection
bool robotDetected;

// Controller parameters
const double alpha = 0.001;
const double rho = 0.8;
const double distThreshold = 0.6;
const double angularThreshold = 15*M_PI/180;

// Sonar
bool sonar;
double right = INFINITY;
double left = INFINITY;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* TRACKINGAMONGOBSTACLES_HPP */
