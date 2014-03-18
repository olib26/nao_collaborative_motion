#include "nao_behavior_tree/rosaction.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

#include <geometry_msgs/Twist.h>
#include <nao_behavior_tree/Odometry.h>

#include "nao_behavior_tree/actions/GoToPoint.hpp"


double modulo2Pi(double theta)
{
	// Angle between ]-pi,pi]
	while(theta > M_PI) {theta -= 2*M_PI;}
	while(theta <= -M_PI) {theta += 2*M_PI;}
	return theta;
}


double angle(double dx, double dy)
{
	double theta;
	if(dx == 0)
	{
		if(dy < 0) {theta = -M_PI/2;}
		if(dy > 0) {theta = M_PI/2;}
	}
	else
	{
		theta = atan(dy/dx);
		if(dx < 0) {theta += M_PI;}
	}

	theta = modulo2Pi(theta);

	return theta;
}


void estimateBearing()
{
	std::vector<float> odometry = motion_proxy_ptr->getRobotPosition(useSensorValues);

	if(sqrt(r.vx*r.vx + r.vy*r.vy) > velThreshold)
	{		
		r.theta = angle(r.vx,r.vy);
	}
	else
	{
		// Use MRE sensor
		dtheta = odometry.at(2) - theta_temp;
		r.theta += dtheta;

		//ROS_INFO("dtheta = %f",dtheta);
	}

	theta_temp = odometry.at(2);
}


geometry_msgs::Twist controller()
{
	geometry_msgs::Twist cmd;
	
	estimateBearing();
	
	double linear = rho;
	double angular = modulo2Pi(angle(p.x-r.x,p.y-r.y)-r.theta);
	
	// Robot not in the right direction
	if(fabs(angular)*180/M_PI > angleThreshold)
	{
		linear = 0;
	}
	
	//ROS_INFO("angular = %f, linear = %f",angular,linear);

	// Command threshold
	if(angular > 1) {angular = 1;}
	if(angular < -1) {angular = -1;}

	cmd.linear.x = linear;
	cmd.angular.z = angular;

	return cmd;
}


class GoToPoint : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALRobotPostureProxy* robotPosture;

	GoToPoint(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
	}

	~GoToPoint()
	{
		delete motion_proxy_ptr;
		delete robotPosture;
	}

	void initialize()
	{
		init_ = true;

		// Enable stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		// Stand
		robotPosture->goToPosture("Stand",0.5f);

		// Init moving
		motion_proxy_ptr->moveInit();

		// Init bearing
		std::vector<float> odometry = motion_proxy_ptr->getRobotPosition(useSensorValues);
		theta_temp = odometry.at(2);
		
		r.theta = angle(p.x-r.x,p.y-r.y);
	}

	void finalize()
	{
		// Stop moving
		motion_proxy_ptr->stopMove();

		init_ = false;
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**GoToPoint -%- Executing Main Task, elapsed_time: "
		          << dt.toSec() << std::endl;
		std::cout << "**GoToPoint -%- execute_time: "
		          << execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			set_feedback(RUNNING);
			initialize();
		}

		if(sqrt((r.x-p.x)*(r.x-p.x) + (r.y-p.y)*(r.y-p.y)) > distThreshold)
		{
			geometry_msgs::Twist cmd = controller();
			cmd_pub.publish(cmd);
		}
		else
		{
			set_feedback(SUCCESS);
			finalize();
			return 1;
		}

		//ROS_INFO("x = %f, y = %f, theta = %f",r.x,r.y,r.theta);

		return 0;
	}

	void resetCB()
	{
		execute_time_ = (ros::Duration) 0;
	}
};


void receive_odometry(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r.x = msg->x;
	r.y = msg->y;
	r.vx = msg->vx;
	r.vy = msg->vy;
}


int main(int argc, char** argv)
{
	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {r.id = "1";}
		if(atoi(argv[1]) == 2) {r.id = "2";}

		ros::init(argc, argv,"GoToPoint" + r.id);
		ros::NodeHandle nh;

		ros::NodeHandle pnh("~");
		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));
		// Point
		pnh.param("x",p.x,double(0));
		pnh.param("y",p.y,double(0));

		// Odometry subscribers
		ros::Subscriber odom_sub = nh.subscribe("/odometry" + r.id,1,receive_odometry);

		// Walker publisher
		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + r.id,1);

		// Launch Server
		GoToPoint server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
