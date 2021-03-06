/*
 * HeadTracker.hpp
 *
 *  Created on: Mar 17, 2014
 *      Author: Olivier BALLAND
 */


#ifndef HEADTRACKER_HPP
#define HEADTRACKER_HPP


// Parameters
AL::ALValue name = "HeadYaw";
bool useSensors = true;
float fractionMaxSpeed = 0.02;
AL::ALValue angle;
double angleThreshold = 10*M_PI/180;

// Bearing
double relative;
bool robotDetected;


#endif /* HEADTRACKER_HPP */
