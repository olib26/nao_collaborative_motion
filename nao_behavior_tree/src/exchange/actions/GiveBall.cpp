#include <nao_behavior_tree/rosaction.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <nao_behavior_tree/BallPosGetBT.h>
#include <nao_behavior_tree/ArmReadyBT.h>

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

//prepare arm to hold the ball
void holdArm(int nao,std::string ip,int arm,AL::ALMotionProxy* motion_proxy_ptr){
	if(arm==0){//RIGHT ARM
		//preplace arm
		AL::ALValue names = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
				,"RElbowYaw","RElbowRoll","RWristYaw");
		AL::ALValue angles      = AL::ALValue::array(dToR(80.2),dToR(-17.2),dToR(43.5),dToR(89),dToR(0));
		setAngles(nao,ip,names,angles,motion_proxy_ptr);
		sleep(2);
		angles      = AL::ALValue::array(dToR(-8.0),dToR(-3),dToR(43.5),dToR(89),dToR(0));
		setAngles(nao,ip,names,angles,motion_proxy_ptr);
		sleep(2);
		angles      = AL::ALValue::array(dToR(-20.0),dToR(-1),dToR(43.5),dToR(3),dToR(0));
		setAngles(nao,ip,names,angles,motion_proxy_ptr);
	}

	if (arm==1){//LEFT ARM
		//preplace arm
		AL::ALValue names = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
				,"LElbowYaw","LElbowRoll","LWristYaw");
		AL::ALValue angles      = AL::ALValue::array(dToR(80.2),dToR(17.2),dToR(-43.5),dToR(-89),dToR(0));
		setAngles(nao,ip,names,angles,motion_proxy_ptr);
		sleep(2);
		angles      = AL::ALValue::array(dToR(-8.0),dToR(3),dToR(-43.5),dToR(-89),dToR(0));
		setAngles(nao,ip,names,angles,motion_proxy_ptr);
		sleep(2);
		angles      = AL::ALValue::array(dToR(-20.0),dToR(1),dToR(-43.5),dToR(-3),dToR(0));
		setAngles(nao,ip,names,angles,motion_proxy_ptr);
	}
}
//////////




class GiveBall : ROSAction
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
	nao_behavior_tree::ArmReadyBT srv2;
	ros::Duration time_to_complete_;

	GiveBall(std::string name,int NAO_PORT,std::string NAO_IP,int nao) :
		ROSAction(name),
		init_(false),
		time_to_complete_((ros::Duration) 0) // used for this example
		{
			motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
			robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
			speechProxy = new AL::ALTextToSpeechProxy (NAO_IP,NAO_PORT);
			NAO_PORT_p=NAO_PORT;
			NAO_IP_p=NAO_IP;
			nao_p=nao;
		}

	~GiveBall()
		{
		delete motion_proxy_ptr;
		delete speechProxy;
		delete robotPosture;
		}



	void initialize(){
		init_ = true;
		set_feedback(RUNNING);

		//Stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		speechProxy->say("I WANNA GIVE MY BALL.");
		robotPosture->goToPosture("StandInit",0.5f);
	}
	
	void finalize(){
	init_ = false;
	deactivate();
	}
	
	int executeCB(ros::Duration dt)
		{
			std::cout << "**GiveBall -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**GiveBall -%- elapsed_time: "
			          << time_to_complete_.toSec() << std::endl;

			time_to_complete_ += dt;

			if(!init_){
			initialize();
			
			srv1.request.NAO=nao_p;
			if(client1.call(srv1)){
				hand=(long int)srv1.response.hand;
			}

		
			if (time_to_complete_.toSec() < 10)
			{
				holdArm(NAO_PORT_p,NAO_IP_p,hand,motion_proxy_ptr);
				speechProxy->say("YOU CAN TAKE THE BALL.");
				srv2.request.NAO=nao_p;
				srv2.request.arm=hand+1;
				if(client2.call(srv2)){
				}
				set_feedback(SUCCESS);
				finalize();
				return 1;		
			}
			else if (time_to_complete_.toSec() >= 10)
			{
				speechProxy->say("FINALLY, I WON'T GIVE ANYTHING.");
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
{
	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {nao_id = "1";nao=1;}
		if(atoi(argv[1]) == 2) {nao_id = "2";nao=2;}

		ros::init(argc, argv, "GiveBall"+nao_id);
		ros::NodeHandle pnh("~");
		ros::NodeHandle n;
		client1 = n.serviceClient<nao_behavior_tree::BallPosGetBT::Response>("ballPosGet");
		client2 = n.serviceClient<nao_behavior_tree::ArmReadyBT::Response>("armReady");

		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

	GiveBall server(ros::this_node::getName(),NAO_PORT,NAO_IP,nao);
	ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
