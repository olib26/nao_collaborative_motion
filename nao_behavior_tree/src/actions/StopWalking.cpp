#include "nao_behavior_tree/rosaction.h"
#include <alproxies/almotionproxy.h>


class StopWalking : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALMotionProxy* motion_proxy_ptr;

	StopWalking(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr  = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	}

	void initialize()
	{
		init_ = true;
		set_feedback(RUNNING);
	}

	void finalize()
	{
		// Stop walking
		// and send vel = 0 !!!!!!!!!!!!!
		motion_proxy_ptr->stopMove();

		init_ = false;
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**Stop -%- Executing Main Task, elapsed_time: "
				<< dt.toSec() << std::endl;
		std::cout << "**Stop -%- execute_time: "
				<< execute_time_.toSec() << std::endl;
		execute_time_ += dt;


		if (!init_)
		{
			initialize();

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

		ros::init(argc, argv,"StopWalking" + nao);

		// Robot parameters
		ros::NodeHandle pnh("~");
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		StopWalking server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
