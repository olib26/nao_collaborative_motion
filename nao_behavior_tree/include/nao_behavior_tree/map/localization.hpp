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
const int N = 2000;

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
const int nb = 25; //100
int x_temp1,x_temp2,y_temp1,y_temp2;

// Estimated position
bool estimatedPosition;
AL::ALMotionProxy* motion_proxy_ptr1;
AL::ALMotionProxy* motion_proxy_ptr2;
std::string NAO_IP1,NAO_IP2;
int NAO_PORT1,NAO_PORT2;
double x_10,y_10,theta_10,x_20,y_20,theta_20;
double x_01,y_01,theta_01,x_02,y_02,theta_02;
double x_1,y_1,theta_1,x_2,y_2,theta_2;
bool init1,init2;
double x_temp_est1,y_temp_est1,x_temp_est2,y_temp_est2;


#endif /* LOCALIZATION_HPP_ */
