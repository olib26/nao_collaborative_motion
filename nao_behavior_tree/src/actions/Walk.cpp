#include "nao_behavior_tree/rosaction.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

#include <geometry_msgs/Twist.h>


ros::Publisher cmd_pub;

class Walk : ROSAction
{
public:
	bool init_;
	double dist;
	ros::Duration execute_time_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* robotPosture;

	Walk(std::string name,std::string NAO_IP,int NAO_PORT,double dist) :
		ROSAction(name),
		init_(false),
		dist(dist),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
	}

	~Walk()
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
		//motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		// Stand
		//robotPosture->goToPosture("Stand",0.5f);

		// Init moving
		motion_proxy_ptr->moveInit();
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
		std::cout << "**Walk -%- Executing Main Task, elapsed_time: "
		          << dt.toSec() << std::endl;
		std::cout << "**Walk -%- execute_time: "
		          << execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			set_feedback(RUNNING);
			initialize();
		}

		// Walk forward
		geometry_msgs::Twist cmd;
		cmd.linear.x = 0.5;
		cmd_pub.publish(cmd);

		if(motion_proxy_ptr->getRobotPosition(true).front() > dist)
		{
			set_feedback(SUCCESS);
			finalize();
			return 1;
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

		ros::init(argc, argv,"Walk" + nao);
		ros::NodeHandle nh;

		ros::NodeHandle pnh("~");
		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		// Distance
		double dist;
		pnh.param("distance",dist,double(0));

		// Walker publisher
		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + nao,100);

		// Launch Server
		Walk server(ros::this_node::getName(),NAO_IP,NAO_PORT,dist);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
