/*
 * nao_tracker_camera.hpp
 *
 *  Created on: Jan 22, 2014
 *      Author: Olivier BALLAND
 */

#ifndef NAO_TRACKER_CAMERA_HPP
#define NAO_TRACKER_CAMERA_HPP


// Robot parameters
std::string nao;

// Initialization
bool init = false;

// Controller parameters
const double alpha = 0.001;
const double rho = 0.8;
const double dist = 0.5;

// Sonar
double right = INFINITY;
double left = INFINITY;


#endif /* NAO_TRACKER_CAMERA_HPP */
