/*
 * nao_sonar.cpp
 *
 *  Created on: Feb 2, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsonarproxy.h>
#include <first_approaches/Sonar.h>

using namespace first_approaches;


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_sonar");
	ros::NodeHandle nh;

	// Robot parameters
	ros::NodeHandle pnh("~");
	std::string NAO_IP;
	int NAO_PORT;
	pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
	pnh.param("NAO_PORT",NAO_PORT,int(9559));

	// Publisher
	ros::Publisher sonar_pub = nh.advertise<Sonar>("/sonar",100);

	// Data
	double right,left;
	ros::Time timestamp;
	Sonar sonar_msg;

	// Connect to ALSonar module.
	AL::ALSonarProxy sonarProxy = AL::ALSonarProxy(NAO_IP,NAO_PORT);

	// Subscribe to sonars, this will launch sonars (at hardware level) and start data acquisition.
	std::string name = "nao_sonar";
	sonarProxy.subscribe(name);

	// Memory proxy
	AL::ALMemoryProxy* memory_proxy_ptr;
	memory_proxy_ptr = new AL::ALMemoryProxy(NAO_IP,NAO_PORT);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		timestamp = ros::Time::now();
		right = memory_proxy_ptr->getData("Device/SubDeviceList/US/Right/Sensor/Value");
		left = memory_proxy_ptr->getData("Device/SubDeviceList/US/Left/Sensor/Value");

		sonar_msg.right = right;
		sonar_msg.left = left;
		sonar_msg.header.stamp = timestamp;
		sonar_pub.publish(sonar_msg);

		loop_rate.sleep();
		ros::spinOnce();
	}

	// Stopping sonar hardware
	sonarProxy.unsubscribe(name);

	return 0;
}
