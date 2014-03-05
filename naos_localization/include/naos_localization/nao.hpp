/*
 * nao.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: Olivier BALLAND
 */

#ifndef NAO_HPP_
#define NAO_HPP_


// Robot parameters
std::string NAO_IP;
int NAO_PORT;

// Proxy
AL::ALVideoDeviceProxy* camera_proxy_ptr;
std::string clientName;

// Image
IplImage* imgHeader;

// Parameters
const double HFOV = 60.9*M_PI/180;
const double cameraHeight = 2; // meters


#endif /* NAO_HPP_ */
