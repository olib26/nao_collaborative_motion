/*
 * nao_tracker_odometry.hpp
 *
 *  Created on: Jan 22, 2014
 *      Author: Olivier BALLAND
 */

#ifndef NAO_TRACKER_ODOMETRY_HPP
#define NAO_TRACKER_ODOMETRY_HPP


using namespace nao_msgs;


// Publisher
ros::Publisher cmd1_pub,cmd2_pub;

// Initialization
double x_10,y_10,theta_10,x_20,y_20,theta_20;
bool init1,init2;

// Initial odometry of the two robots
double x_01,y_01,theta_01,x_02,y_02,theta_02;

// Odometry of the two robots
double x_1,y_1,theta_1,x_2,y_2,theta_2;

// Controller parameters
const double alpha = 0.3; //0.1
const double rho = 1;
const double dist_min = 0.4;


void receive_odometry1(const TorsoOdometry::ConstPtr &msg);
void receive_odometry2(const TorsoOdometry::ConstPtr &msg);


#endif /* NAO_TRACKER_ODOMETRY_HPP */
