/*
 * Search.hpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Olivier BALLAND
 */


#ifndef SEARCH_HPP
#define SEARCH_HPP


// Robot selection
std::string nao;

// Walker publisher
ros::Publisher cmd_pub;

// Image size
int height,width;
const int cutHeight = 100;

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

// Diffusion variance
const int sigma_diffusion = 50; // Position
const int s_diffusion = 10; // Size

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
const double Var_minx = 1000;
const double Var_miny = 1000;

// Robot detection
bool robotDetected;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* SEARCH_HPP */
