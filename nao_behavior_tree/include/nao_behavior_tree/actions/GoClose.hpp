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
const int cutHeight = 150;

// Camera parameters
const double VFOV = 47.6*M_PI/180;
const double HFOV = 60.9*M_PI/180;

// Nao head spot
const double H = 0.043;

// Mean depth
double depth_temp;
int counter;
const int nbSamplesMean = 3;

// Detector frame size
const int sx_min = 20;
const int sy_min = 30;
const int sx_max = 200;
const int sy_max = 300;

// Diffusion standard deviation
const int sigma_diffusion = 50; // Position
const int s_diffusion = 10; // Size

// Object depth
double depth = INFINITY;

// Number of particles
const int N = 2500;

// Object
Object* objects = new Object;

// Particle Filter
particleFilter PF(N,1,MAP,objects,sx_min,sy_min,sx_max,sy_max,sigma_diffusion,s_diffusion);

// Variance Threshold
const double StD_max = 2000;

// Robot detection
bool robotDetected;

// Controller parameters
const double alpha = 0.001;
const double rho = 0.8;
const double distThreshold = 0.4;
const double yThreshold = 100;

// Sonar
bool sonar;
double right = INFINITY;
double left = INFINITY;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* GOCLOSE_HPP */
