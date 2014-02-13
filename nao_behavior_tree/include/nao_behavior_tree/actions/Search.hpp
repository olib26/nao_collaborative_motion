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

// Update Filter
bool updateRequest;

// Image size
int height,width;

// Point in integral image
struct Point
{
	int x,y;
	Point(int x,int y) : x(x),y(y){}
};

// Detector frame size
const int sx_min = 50;
const int sy_min = 70;

// Diffusion variance
const int sigma_diffusion = 8;

// Number of particles
const int N = 1000;

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
const double Var_min = 100;

// Robot detection
bool robotDetected;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* SEARCH_HPP */
