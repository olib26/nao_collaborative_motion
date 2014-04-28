#include "nao_behavior_tree/rosaction.h"


class Stop : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALBehaviorManagerProxy* behavior_proxy_ptr;

	Stop(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
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


		// Disable siffness

	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**Stop -%- Executing Main Task, elapsed_time: "
				<< dt.toSec() << std::endl;
		std::cout << "**Stop -%- execute_time: "
				<< execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		set_feedback(RUNNING);
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

		Stop server(ros::this_node::getName());

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
