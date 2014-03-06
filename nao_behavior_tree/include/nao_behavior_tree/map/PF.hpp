/*
 * PF.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Olivier Balland
 */

#ifndef PF_HPP_
#define PF_HPP_


// Image size
int height,width;

// Camera coefficient
double k;

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
	int x_temp,y_temp;
	int sx,sy;
	double vx_temp,vy_temp;
	double absoluteBearing,relativeBearing;
	Robot () : x(0),y(0),x_temp(0),y_temp(0),sx(sx_min),sy(sy_min),vx_temp(0),vy_temp(0),absoluteBearing(0),relativeBearing(0) {}
};
Robot r1,r2;

// Odometry
nao_behavior_tree::Odometry odom1,odom2;
double timestamp1,timestamp2;
int counter1,counter2;

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

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min,hsv_max;

// Velocity
const int nbSamples = 10;


#endif /* PF_HPP_ */
