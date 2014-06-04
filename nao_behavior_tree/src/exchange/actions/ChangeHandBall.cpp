#include <nao_behavior_tree/rosaction.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <nao_behavior_tree/BallPosChangeBT.h>
#include <nao_behavior_tree/BallPosGetBT.h>

std::string nao_id;
int nao;
ros::ServiceClient client1;
ros::ServiceClient client2;

//angle transcriptor
float dToR(float angle){
	return angle*M_PI/180.;
}
//////////

//Method to control the joint angles
void setAngles(int nao,std::string ip,AL::ALValue names,AL::ALValue angles,AL::ALMotionProxy* motion_proxy_ptr){

	float fractionMaxSpeed = 0.10f;

	//set arms angles
	motion_proxy_ptr->setAngles(names, angles, fractionMaxSpeed);
}
//////////

//Method to open hand
void openHand(int nao,std::string ip,int hand,AL::ALMotionProxy* motion_proxy_ptr){
	std::string effector;
	if (hand==1){effector="LHand";}
	if (hand==0){effector="RHand";}

	AL::ALValue hands = AL::ALValue::array(effector);
	AL::ALValue angles = AL::ALValue::array(1.0f);

	setAngles(nao,ip,hands,angles,motion_proxy_ptr);
}
//////////

//Method to close hand
void closeHand(int nao,std::string ip,int hand,AL::ALMotionProxy* motion_proxy_ptr){
	std::string effector;
	if (hand==1){effector="LHand";}
	if (hand==0){effector="RHand";}

	AL::ALValue hands = AL::ALValue::array(effector);
	AL::ALValue angles = AL::ALValue::array(0.0f);

	setAngles(nao,ip,hands,angles,motion_proxy_ptr);
}
//////////


//passes the ball to the other hand
void changeHand(int nao,std::string ip,int arm,AL::ALMotionProxy* motion_proxy_ptr){

	motion_proxy_ptr->setSmartStiffnessEnabled(false);

	if(arm==0){
		openHand(nao,ip,1,motion_proxy_ptr);

		AL::ALValue namesL = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
				,"LElbowYaw","LElbowRoll","LWristYaw");
		//AL::ALValue anglesL      = AL::ALValue::array(dToR(16.6),dToR(0.5),dToR(-40),dToR(-44),dToR(-48));
		AL::ALValue anglesL      = AL::ALValue::array(dToR(66),dToR(2),dToR(-40),dToR(-61),dToR(-48));
		setAngles(nao,ip,namesL,anglesL,motion_proxy_ptr);

		sleep(1);

		AL::ALValue namesR = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
						,"RElbowYaw","RElbowRoll","RWristYaw");
		//AL::ALValue anglesR      = AL::ALValue::array(dToR(16.6),dToR(-0.5),dToR(40),dToR(44),dToR(36));
		AL::ALValue anglesR      = AL::ALValue::array(dToR(64),dToR(-2),dToR(40),dToR(57),dToR(36));
		setAngles(nao,ip,namesR,anglesR,motion_proxy_ptr);

		sleep(2);

		motion_proxy_ptr->setSmartStiffnessEnabled(true);
		sleep(1);
		closeHand(nao,ip,1,motion_proxy_ptr);
		sleep(2);
		closeHand(nao,ip,1,motion_proxy_ptr);
		sleep(2);
		openHand(nao,ip,0,motion_proxy_ptr);
		sleep(2);
	}

	else{
		openHand(nao,ip,0,motion_proxy_ptr);

		AL::ALValue namesR = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
						,"RElbowYaw","RElbowRoll","RWristYaw");
		AL::ALValue anglesR      = AL::ALValue::array(dToR(64),dToR(-2),dToR(40),dToR(57),dToR(36));
		setAngles(nao,ip,namesR,anglesR,motion_proxy_ptr);

		sleep(1);

		AL::ALValue namesL = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
				,"LElbowYaw","LElbowRoll","LWristYaw");
		AL::ALValue anglesL      = AL::ALValue::array(dToR(66),dToR(2),dToR(-40),dToR(-61),dToR(-48));
		setAngles(nao,ip,namesL,anglesL,motion_proxy_ptr);

		sleep(2);

		motion_proxy_ptr->setSmartStiffnessEnabled(true);
		sleep(1);
		closeHand(nao,ip,0,motion_proxy_ptr);
		sleep(2);
		closeHand(nao,ip,0,motion_proxy_ptr);
		sleep(2);
		openHand(nao,ip,1,motion_proxy_ptr);
		sleep(2);
	}
}
//////////





class ChangeHandBall : ROSAction
{
public:
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* robotPosture;
	AL::ALTextToSpeechProxy* speechProxy;
	int hand;
	bool init_;
	int NAO_PORT_p;
	std::string NAO_IP_p;
	int nao_p;
	nao_behavior_tree::BallPosGetBT srv1;
	nao_behavior_tree::BallPosChangeBT srv2;
	ros::Duration time_to_complete_;

	ChangeHandBall(std::string name,int NAO_PORT,std::string NAO_IP,int nao) :
		ROSAction(name),
		init_(false),
		time_to_complete_((ros::Duration) 0)
		{	ROS_INFO("coucou1");
			motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
			robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
			speechProxy = new AL::ALTextToSpeechProxy(NAO_IP,NAO_PORT);
			NAO_PORT_p=NAO_PORT;
			NAO_IP_p=NAO_IP;
			nao_p=nao;
			ROS_INFO("coucou2");
		}

	~ChangeHandBall()
	{
		delete motion_proxy_ptr;
		delete robotPosture;
		delete speechProxy;
	}

	void initialize(){
		init_ = true;
		set_feedback(RUNNING);
		ROS_INFO("coucou4");

		//Stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		speechProxy->say("I WANNA CHANGE THE HAND HOLDING THE BALL.");
		robotPosture->goToPosture("StandInit",0.5f);
		ROS_INFO("coucou5");
	}
	
	void finalize(){
		init_ = false;
		deactivate();
	}

	
	int executeCB(ros::Duration dt)
		{
			ROS_INFO("executeCB");
			std::cout << "**ChangeHandBall -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**ChangeHandBall -%- elapsed_time: "
			          << time_to_complete_.toSec() << std::endl;

			time_to_complete_ += dt;


			if(!init_){
			ROS_INFO("coucou3");
			initialize();
			
			srv1.request.NAO=nao_p;
			if(client1.call(srv1)){
				hand=(long int)srv1.response.hand;
			}


			if (time_to_complete_.toSec() < 10){
			changeHand(NAO_PORT_p,NAO_IP_p,hand,motion_proxy_ptr);
			robotPosture->goToPosture("StandInit",0.5f);
			srv2.request.object=1;
			srv2.request.NAO=nao_p;
			if(hand==0){hand=1;} 
			else{hand=0;}
			srv2.request.hand=hand;
			if(client2.call(srv2)){
			}
			speechProxy->say("change has been done");
			set_feedback(SUCCESS);
			finalize();
			return 1;	
			}
			else if (time_to_complete_.toSec() >= 10)
			{
				set_feedback(FAILURE);
				finalize();
				return 1;		
			}
			}
			return 0;
		}

	void resetCB()
		{
			time_to_complete_ = (ros::Duration) 0;
		}


};

int main(int argc, char** argv)
{ROS_INFO("COUCOU bisou");
	if(argc != 1)
	{
		ROS_INFO("COUCOU");
		// Robot selection
		if(atoi(argv[1]) == 1) {nao_id = "1";nao=1;}
		if(atoi(argv[1]) == 2) {nao_id = "2";nao=2;}

		ros::init(argc, argv, "ChangeHandball"+nao_id);
		ros::NodeHandle pnh("~");
		ros::NodeHandle n;
		client1 = n.serviceClient<nao_behavior_tree::BallPosGetBT::Response>("ballPosGet");
		client2 = n.serviceClient<nao_behavior_tree::BallPosChangeBT::Response>("ballPosChange");

		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

	ChangeHandBall server(ros::this_node::getName(),NAO_PORT,NAO_IP,nao);
	ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
