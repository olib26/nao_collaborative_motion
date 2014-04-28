/*
 * localization.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Olivier Balland
 */

#ifndef LOCALIZATION_HPP_
#define LOCALIZATION_HPP_


// Image size
int height,width;

// Camera coefficient
double k;

// Detector frame size
const int sx_min = 15;
const int sy_min = 15;
const int sx_max = 200;
const int sy_max = 300;

// Diffusion variance
const int sigma_diffusion = 8; // Position
const int s_diffusion = 5; // Size

// Object coordinate
struct Robot
{
	int x,y;
	int x_temp,y_temp;
	int sx,sy;
	double vx_temp,vy_temp;
	double absoluteBearing,relativeBearing;

	// Init temp position
	bool initTempPosition;

	Robot () : x(0),y(0),x_temp(0),y_temp(0),sx(sx_min),sy(sy_min),vx_temp(0),vy_temp(0),absoluteBearing(0),relativeBearing(0),initTempPosition(false) {}
};
Robot r1,r2;

bool robot1Detected,robot2Detected;

// Odometry
nao_behavior_tree::Odometry odom1,odom2;
double timestamp1,timestamp2;
int counter1,counter2;

// Number of particles
const int N = 1000;

// Particle Filter
particleFilter PF;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min,hsv_max;

// Velocity
const int nbSamples = 20;

// Optimal velocity
nao_behavior_tree::Velocity vel1;

// Path samples
int counter = 0;
const int nb = 10;
int x_temp1,x_temp2,y_temp1,y_temp2;


#endif /* LOCALIZATION_HPP_ */
