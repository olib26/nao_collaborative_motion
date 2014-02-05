/*
 * nao_behavior.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include <alproxies/albehaviormanagerproxy.h>

#include <std_srvs/Empty.h>


AL::ALBehaviorManagerProxy* behavior_proxy_ptr;

// Robot parameters
std::string NAO_IP,nao;
int NAO_PORT;


bool startService(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
{
	// Run behavior
	behavior_proxy_ptr->runBehavior("behavior_hello");

	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_hello");
	ros::NodeHandle nh;

	ros::NodeHandle pnh("~");
	pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
	pnh.param("NAO_PORT",NAO_PORT,int(9559));

	if(argc != 1)
	{
		std::string nao;

		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		// Behavior manager proxy
		behavior_proxy_ptr = new AL::ALBehaviorManagerProxy(NAO_IP,NAO_PORT);

		// Install behavior
		behavior_proxy_ptr->installBehavior("/home/olivier/ros_workspace/collaborative_motion/behaviors/behavior_hello/behavior.xar");

		// Service
		ros::ServiceServer service = nh.advertiseService("nao_hello" + nao,startService);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
