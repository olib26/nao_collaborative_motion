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
const int cutHeight = 150;

// Detector frame size
const int sx_min = 20;
const int sy_min = 30;
const int sx_max = 200;
const int sy_max = 300;

// Diffusion standard deviation
const int sigma_diffusion = 50; // Position
const int s_diffusion = 10; // Size

// Number of particles
const int N = 2000;

// Object
Object* objects = new Object;

// Particle Filter
particleFilter PF(N,1,MAP,objects,sx_min,sy_min,sx_max,sy_max,sigma_diffusion,s_diffusion);

// Variance Threshold
const double StD_minx = 2000;
const double StD_miny = 2000;

// Robot detection
bool robotDetected;

// HSV Thresholds
int H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX;
CvScalar hsv_min;
CvScalar hsv_max;


#endif /* SEARCH_HPP */
