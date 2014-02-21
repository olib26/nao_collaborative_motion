/*
 * GoClose.hpp
 *
 *  Created on: Feb 13, 2014
 *      Author: Olivier BALLAND
 */


#ifndef GOCLOSE_HPP
#define GOCLOSE_HPP


// Robot selection
std::string nao;

// Walker publisher
ros::Publisher cmd_pub;

// Image size
int height,width;

// Camera parameters
const double VFOV = 47.6*M_PI/180;
const double HFOV = 60.9*M_PI/180;

// Nao head spot
const double H = 0.036;

// Point in integral image
struct Point
{
	int x,y;
	Point(int x,int y) : x(x),y(y){}
};

// Detector frame size
int sx,sy;
const int sx_min = 10;
const int sy_min = 15;
const int sx_max = 200;
const int sy_max = 300;


// Diffusion variance
const int sigma_diffusion = 8; // Position
const int s_diffusion = 2; // Size

// Object coordinate
int x,y;
double depth;

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
const double Var_max = 1000;

// Robot detection
bool robotDetected;

// Controller parameters
const double alpha = 0.001;
const double rho = 0.8;
const double dist_threshold = 0.8;

// Sonar
double right = INFINITY;
double left = INFINITY;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* GOCLOSE_HPP */
