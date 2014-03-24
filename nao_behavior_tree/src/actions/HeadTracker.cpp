/*
 * HeadTracker.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include "nao_behavior_tree/rosaction.h"

#include <alproxies/almotionproxy.h>

#include <nao_behavior_tree/Bearing.h>

#include <nao_behavior_tree/actions/HeadTracker.hpp>


void receive_bearing(const nao_behavior_tree::Bearing::ConstPtr &msg)
{
	relative = msg->relative;
	robotDetected = msg->robotDetected;

	//ROS_INFO("Angle = %f",relative);
}


class HeadTracker : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALMotionProxy* motion_proxy_ptr;

	HeadTracker(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	}

	~HeadTracker()
	{
		delete motion_proxy_ptr;
	}

	void initialize()
	{
		init_ = true;

		// Enable stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		//motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);
	}

	void finalize()
	{
		angle = 0;
		motion_proxy_ptr->setAngles(name,angle,fractionMaxSpeed);

		init_ = false;
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**HeadTracker -%- Executing Main Task, elapsed_time: "
		          << dt.toSec() << std::endl;
		std::cout << "**HeadTracker -%- execute_time: "
		          << execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			set_feedback(RUNNING);
			initialize();
		}

		// Tracker
		if(robotDetected)
		{
			angle = (float)(-relative-motion_proxy_ptr->getAngles(name,useSensors).front());
			if((fabs(motion_proxy_ptr->getAngles(name,useSensors).front()) < 2) & (fabs((double)angle) > angleThreshold)) {motion_proxy_ptr->changeAngles(name,angle,fractionMaxSpeed);}

			//angle = (float)(-relative);
			//if(fabs(relative) < 2) {motion_proxy_ptr->setAngles(name,angle,fractionMaxSpeed);}
		}

		else
		{
			angle = 0;
			motion_proxy_ptr->setAngles(name,angle,fractionMaxSpeed);
		}

		return 0;
	}

	void resetCB()
	{
		execute_time_ = (ros::Duration) 0;
	}
};


int main(int argc, char** argv)
{
	if(argc != 1)
	{
		// Robot selection
		std::string nao;
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		ros::init(argc, argv,"HeadTracker" + nao);
		ros::NodeHandle nh;

		// Robot parameters
		ros::NodeHandle pnh("~");
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		// Subscriber
		ros::Subscriber bearing_sub = nh.subscribe<nao_behavior_tree::Bearing>("/bearing" + nao,1,receive_bearing);

		// Launch Server
		HeadTracker server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}


