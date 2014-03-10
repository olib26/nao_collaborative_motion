/*
 * nao_behavior.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include <alproxies/albehaviormanagerproxy.h>
#include <alproxies/almotionproxy.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_behavior");
	ros::NodeHandle nh;

	// Robot parameters
	ros::NodeHandle pnh("~");
	std::string NAO_IP;
	int NAO_PORT;
	pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
	pnh.param("NAO_PORT",NAO_PORT,int(9559));

	// Enable stiffness
	AL::ALMotionProxy* motion_proxy_ptr;
	motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	AL::ALValue stiffness_name("Body");
	AL::ALValue stiffness(1.0f);
	AL::ALValue stiffness_time(1.0f);
	motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

	// Behavior manager proxy
	AL::ALBehaviorManagerProxy* behavior_proxy_ptr;
	behavior_proxy_ptr = new AL::ALBehaviorManagerProxy(NAO_IP,NAO_PORT);

	// Install behavior
	bool installation = behavior_proxy_ptr->installBehavior("/home/olivier/ros_workspace/nao_custom/behavior_hello/behavior.xar");

	if(installation) {ROS_INFO("INSTALLED");}
	else {ROS_INFO("FAILURE");}

	// Behavior list
	std::vector<std::string> list = behavior_proxy_ptr->getInstalledBehaviors();
	printf("Behaviors list:\n");
	for(size_t i = 0; i < list.size(); i++)
	{
		std::cout << list.at(i) << std::endl;
	}

	// Run behavior
	behavior_proxy_ptr->runBehavior("behavior_hello");

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
