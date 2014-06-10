/*
 * remote.cpp
 *
 *  Created on: May 12, 2014
 *      Author: Olivier BALLAND
 */

/* Description:
 *  Allows to control a Nao with the keyboard.
 */

/* Topics used:
 *  /cmd_vel
 */


#include <ros/ros.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/albehaviormanagerproxy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_ENTER 0x0D
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

const double alpha = 0.5;
const double beta = 0.5;

int kfd = 0;
struct termios cooked, raw;

ros::Publisher cmd_pub;


void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
	if(argc != 1)
	{
		// Robot selection
		std::string id;
		if(atoi(argv[1]) == 1) {id = "3";}
		if(atoi(argv[1]) == 2) {id = "4";}

		ros::init(argc, argv,"remote" + id);
		ros::NodeHandle nh;

		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + id,1);

		ros::NodeHandle pnh("~");
		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		// Enable stiffness
		AL::ALMotionProxy* motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		char c;
		tcgetattr(kfd, &cooked);
		memcpy(&raw, &cooked, sizeof(struct termios));
		raw.c_lflag &=~ (ICANON | ECHO);
		raw.c_cc[VEOL] = 1;
		raw.c_cc[VEOF] = 2;
		tcsetattr(kfd, TCSANOW, &raw);

		puts("Reading from keyboard");
		puts("---------------------------");
		puts("Use arrow keys to move the robot.");

		ros::Rate r(10);
		for(;;)
		{
			if(read(kfd, &c, 1) < 0)
			{
				perror("read():");
				exit(-1);
			}

			geometry_msgs::Twist cmd;

			switch(c)
			{
			case KEYCODE_L:
				ROS_DEBUG("LEFT");
				cmd.angular.z = alpha;
				break;
			case KEYCODE_R:
				ROS_DEBUG("RIGHT");
				cmd.angular.z = -alpha;
				break;
			case KEYCODE_U:
				ROS_DEBUG("UP");
				cmd.linear.x = beta;
				break;
			case KEYCODE_D:
				ROS_DEBUG("DOWN");
				cmd.linear.x = -beta;
				break;
			case KEYCODE_ENTER:
				cmd.linear.x = 0;
				cmd.angular.z = 0;
				break;
			}

			cmd_pub.publish(cmd);
			ros::spinOnce();
			ros::Duration(0.02).sleep();
		}
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
