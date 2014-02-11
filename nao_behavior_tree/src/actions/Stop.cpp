#include "nao_behavior_tree/rosaction.h"


class Stop : ROSAction
{
public:
	ros::Duration execute_time_;

	Stop(std::string name) :
		ROSAction(name),
		execute_time_((ros::Duration) 0)
	{}

	// the action succeeds automatically after 5 seconds
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
