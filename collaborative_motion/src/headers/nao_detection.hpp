/*
 * nao_detection.hpp
 *
 *  Created on: Jan 22, 2014
 *      Author: Olivier BALLAND
 */

#ifndef NAO_DETECTION_HPP
#define NAO_DETECTION_HPP


// Publisher
ros::Publisher robotPosition_pub;

// Client
ros::ServiceClient client;

// Robot selection
std::string nao;

// Initialization
bool init = false;

// Image size
int height,width;

// Camera parameters
const double VFOV = 47.6*M_PI/180;

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
const int s_diffusion = 3;

// Object coordinate
int x,y;
double depth;

// Diffusion variance
const int sigma_diffusion = 8;

// Number of particles
const int N = 2000;

// Generate N random particles
struct Particle{
	int x,y;
	int sx,sy;
	double w;
};
Particle* particles = new Particle[N];

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* NAO_DETECTION_HPP */
