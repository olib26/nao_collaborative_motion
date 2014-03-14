/*
 * nao_leds.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_leds");
	ros::NodeHandle nh;

	// Robot parameters
	ros::NodeHandle pnh("~");
	std::string NAO_IP;
	int NAO_PORT;
	pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
	pnh.param("NAO_PORT",NAO_PORT,int(9559));

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
