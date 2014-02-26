/*
 * PF.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Olivier Balland
 */

#ifndef PF_HPP_
#define PF_HPP_


// Camera index
int camera;

// Image size
int height,width;

// Detector frame size
const int sx_min = 10;
const int sy_min = 15;
const int sx_max = 200;
const int sy_max = 300;

// Diffusion variance
const int sigma_diffusion = 8; // Position
const int s_diffusion = 2; // Size

// Object coordinate
struct Robot
{
	int x,y;
	int sx,sy;
	Robot () : x(0),y(0),sx(sx_min),sy(sy_min) {}
};
Robot r1,r2;

// Odometry
struct Odometry
{
	double x,y;
};

// Number of particles
const int N = 1000;

// Generate N random particles
struct Particle{
	int x,y;
	int sx,sy;
	double w;
};
Particle* particles1 = new Particle[N];
Particle* particles2 = new Particle[N];

// Minimum weight
const double eps = 1E-3;

// Minimum distance between robots
const double distanceThreshold = 30;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min,hsv_max;


#endif /* PF_HPP_ */
