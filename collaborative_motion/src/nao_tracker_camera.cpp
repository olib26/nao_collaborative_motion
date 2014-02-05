/*
 * nao_tracker_camera.cpp
 *
 *  Created on: Jan 24, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nao_custom/Sonar.h>
#include <collaborative_motion/PosImage.h>
#include "headers/nao_tracker_camera.hpp"

#include <std_srvs/Empty.h>

using namespace geometry_msgs;
using namespace collaborative_motion;
using namespace nao_custom;


class Controller
{
	ros::NodeHandle nh;

	// Subscribers
	ros::Subscriber robotPosition_sub;
	ros::Subscriber sonar_sub;

	// Publisher
	ros::Publisher cmd_pub;


public:
	Controller()
	{
		// Subscribers
		robotPosition_sub = nh.subscribe("/image_nao_pos" + nao,1000,&Controller::receive_robotPosition,this);
		sonar_sub = nh.subscribe("/sonar" + nao,1000,&Controller::receive_sonar,this);

		// Publisher
		cmd_pub = nh.advertise<Twist>("/cmd_vel" + nao,100);
	}

	~Controller()
	{
	}

	void receive_robotPosition(const PosImage::ConstPtr &msg)
	{
		// Controller
		Twist cmd;

		int y = msg->y - msg->width/2;
		double angular = -alpha*y;
		// Thresholds
		if(angular > 1) {angular = 1;}
		if(angular < -1) {angular = -1;}

		cmd.linear.x = rho;
		cmd.angular.z = angular;
		cmd_pub.publish(cmd);
	}


	void receive_sonar(const Sonar::ConstPtr &msg)
	{
		right = msg->right;
		left = msg->left;
	}
};


Controller* c;

bool startService(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
{
	if(!init)
	{
		// Start controller
		c = new Controller();
		init = true;
	}

	// Check sonar sensors
	bool res = !((right > dist) & (left > dist));
	if(res) {init = false; delete c;}

	return res;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_tracker_camera");
	ros::NodeHandle nh;

	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		// Service
		ros::ServiceServer service = nh.advertiseService("nao_controller_camera" + nao,startService);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
