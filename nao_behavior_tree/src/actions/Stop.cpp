/* Description:
 *  Makes the robot crouch and disable the stiffness.
 */

#include "nao_behavior_tree/rosaction.h"
#include <alproxies/almotionproxy.h>
#include <alproxies/albehaviormanagerproxy.h>


class Stop : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALBehaviorManagerProxy* behavior_proxy_ptr;

	Stop(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		behavior_proxy_ptr = new AL::ALBehaviorManagerProxy(NAO_IP,NAO_PORT);
		motion_proxy_ptr  = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	}

	void initialize()
	{
		init_ = true;

		// Enable stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		// Crouch
		behavior_proxy_ptr->runBehavior("crouch");

		// Check if it is finished
		while(behavior_proxy_ptr->isBehaviorRunning("crouch"))
		{
			sleep(0.5);
		}

		// Disable siffness
	    stiffness = 0.0f;
	    motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**Stop -%- Executing Main Task, elapsed_time: "
				<< dt.toSec() << std::endl;
		std::cout << "**Stop -%- execute_time: "
				<< execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		set_feedback(RUNNING);

		if (!init_)
		{
			initialize();
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

		ros::init(argc, argv,"Stop" + nao);

		// Robot parameters
		ros::NodeHandle pnh("~");
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		Stop server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
