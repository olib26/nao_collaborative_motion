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

// Point in integral image
struct Point
{
	int x,y;
	Point(int x,int y) : x(x),y(y){}
};

// Detector frame size
const int sx = 30;
const int sy = 50;

// Object coordinate
int x;
int y;

// Diffusion variance
const int sigma_diffusion = 8;

// Number of particles
const int N = 1000;

// Generate N random particles
int Px [N];
int Py [N];
double W [N];

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* NAO_DETECTION_HPP */
