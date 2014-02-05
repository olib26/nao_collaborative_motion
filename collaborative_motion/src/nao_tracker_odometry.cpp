/*
 * nao_tracker_odometry.cpp
 *
 *  Created on: Jan 24, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include <nao_msgs/TorsoOdometry.h>
#include <geometry_msgs/Twist.h>
#include "headers/nao_tracker_odometry.hpp"

using namespace nao_msgs;
using namespace geometry_msgs;


void receive_odometry1(const TorsoOdometry::ConstPtr &msg)
{
	// Estimate odometry
	x_1 = x_01 + msg->x*cos(theta_01) - msg->y*sin(theta_01);
	y_1 = y_01 + msg->x*sin(theta_01) + msg->y*cos(theta_01);
	theta_1 = theta_01 + msg->wz;

	// Controller
	Twist cmd;

	double dist = sqrt((x_2-x_1)*(x_2-x_1)+(y_2-y_1)*(y_2-y_1));
	if(dist > dist_min)
	{
		double dtheta;

		if(x_1 == x_2)
		{
			if((y_2-y_1) < 0) {dtheta = -M_PI;}
			if((y_2-y_1) > 0) {dtheta = M_PI;}
		}
		else
		{
			dtheta = atan((y_2-y_1)/(x_2-x_1));
			if((x_2-x_1) < 0) {dtheta += M_PI;}
			if((x_2-x_1) > 0) {dtheta -= M_PI;}
		}

		dtheta -= theta_1;
		while(dtheta >= M_PI) {dtheta -= 2*M_PI;}
		while(dtheta < -M_PI) {dtheta += 2*M_PI;}

		double linear = rho*dist;
		double angular = alpha*dtheta;
		// Thresholds
		if(linear > 1) {linear = 1;}
		if(angular > 1) {angular = 1;}
		if(angular < -1) {angular = -1;}

		cmd.linear.x = linear;
		cmd.angular.z = angular;
	}

	cmd_pub.publish(cmd);
}


void receive_odometry2(const TorsoOdometry::ConstPtr &msg)
{
	// Estimate odometry
	x_2 = x_02 + msg->x*cos(theta_02) - msg->y*sin(theta_02);
	y_2 = y_02 + msg->x*sin(theta_02) + msg->y*cos(theta_02);
	theta_2 = theta_02 + msg->wz;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_tracker_odometry");
	ros::NodeHandle nh;

	// Initial odometry of the two robots
	ros::NodeHandle pnh("~");
	pnh.param("x_01",x_01,double(0.0));
	pnh.param("y_01",y_01,double(0.0));
	pnh.param("theta_01",theta_01,double(0.0));
	pnh.param("x_02",x_02,double(0.0));
	pnh.param("y_02",y_02,double(0.0));
	pnh.param("theta_02",theta_02,double(0.0));

	x_1 = x_01;
	y_1 = y_01;
	theta_1 = theta_01;
	x_2 = x_02;
	y_2 = y_02;
	theta_2 = theta_02;

	if(argc != 1)
	{
		std::string nao1,nao2;

		// Robot selection
		if(atoi(argv[1]) == 1) {nao1 = "1"; nao2 = "2";}
		if(atoi(argv[1]) == 2) {nao1 = "2"; nao2 = "1";}

		// Subscribers
		ros::Subscriber odometry1_sub = nh.subscribe("/torso_odometry" + nao1,1000,receive_odometry1);
		ros::Subscriber odometry2_sub = nh.subscribe("/torso_odometry" + nao2,1000,receive_odometry2);

		// Publisher
		cmd_pub = nh.advertise<Twist>("/cmd_vel" + nao1,100);

		ros::Rate loop_rate(100);
		while(ros::ok())
		{
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
