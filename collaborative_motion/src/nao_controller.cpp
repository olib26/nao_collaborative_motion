/*
 * nao_controller.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: Olivier BALLAND
 */

#include <ros/ros.h>
#include <alproxies/almotionproxy.h>

#include <std_srvs/Empty.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_controller");
	ros::NodeHandle nh;

	// Robot parameters
	std::string NAO_IP;
	int NAO_PORT;

	ros::NodeHandle pnh("~");
	pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
	pnh.param("NAO_PORT",NAO_PORT,int(9559));

	// Enable stiffness
	AL::ALMotionProxy* motion_proxy_ptr;
	motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	AL::ALValue stiffness_name("Body");
	AL::ALValue stiffness(1.0f);
	AL::ALValue stiffness_time(1.0f);
	motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

	if(argc != 1)
	{
		std::string nao;

		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		// Clients
		ros::ServiceClient client_tracker = nh.serviceClient<std_srvs::Empty>("nao_tracker_camera" + nao);
		ros::ServiceClient client_hello = nh.serviceClient<std_srvs::Empty>("nao_hello" + nao);

		ros::Rate rate(100);
		while(ros::ok())
		{
			// Controller
			std_srvs::Empty srv;
			if(client_tracker.call(srv))
			{
				client_hello.call(srv);
				break;
			}

			rate.sleep();
			ros::spinOnce();
		}

		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();
		}
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
