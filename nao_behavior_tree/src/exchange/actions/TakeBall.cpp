#include "nao_behavior_tree/rosaction.h"
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <nao_behavior_tree/BallPosForHead.h>
#include <nao_behavior_tree/BallPosForHand.h>
#include <nao_behavior_tree/ArmReadyBT.h>
#include <ros/ros.h>

#include <Eigen/Dense>


std::string nao_id;
int nao;
ros::ServiceClient client3;
ros::ServiceClient client4;
ros::ServiceClient client6;

//angle transcriptor
float dToR(float angle){
	return angle*M_PI/180.;
}
//////////


//Method to control the joint angles
void setAngles(int nao,std::string ip,AL::ALValue names,AL::ALValue angles,AL::ALMotionProxy* motion_proxy_ptr){

	//initilize proxies
	//AL::ALMotionProxy motionProxy("127.0.0.1",nao);
	
	float fractionMaxSpeed = 0.10f;

	//set arms angles
	motion_proxy_ptr->setAngles(names, angles, fractionMaxSpeed);
}
//////////

//Method to move an arm respectively to a given displacement vector
void moveArm(int nao,std::string ip,float x,float y,float z,float a,float b,float c,std::string effector,AL::ALMotionProxy* motion_proxy_ptr){
	//effector= "LArm" or "RArm"
	//initilize proxies
	

	int space = 0;//in frame_torso!!
	int axisMask = 63; //remettre à 63 ou 7 au cas où

	std::vector<float> positionChange(6,0.0f);
	positionChange[0]=x;
	positionChange[1]=y;
	positionChange[2]=z;
	positionChange[3]=a;
	positionChange[4]=b;
	positionChange[5]=c;
	float fractionMaxSpeed = 0.2f;

	//changePosition to move with a vector
	motion_proxy_ptr->changePosition(effector, space, positionChange,
			fractionMaxSpeed, axisMask);
}
//////////

//Method to turn head to try putting ball on the screen, we suppose that it will be accessible for the top camera
void turnHead(int nao,std::string ip,int position,AL::ALMotionProxy* motion_proxy_ptr){
	//1:ball at top,2right,3bot,4left,5center,0nowhere
	//AL::ALMotionProxy motionProxy("127.0.0.1",9559);
	float fractionMaxSpeed = 0.10f;

	//joints to manipulate
	AL::ALValue names       = AL::ALValue::array("HeadYaw","HeadPitch");
	//yaw=right-left //pitch=bot-top

	//control angle with the position of the ball
	AL::ALValue angles;
	if (position==1) angles      = AL::ALValue::array(0.0f,-0.10f);
	if (position==2) angles      = AL::ALValue::array(-0.10f,0.0f);
	if (position==3) angles      = AL::ALValue::array(0.0f,0.10f);
	if (position==4) angles      = AL::ALValue::array(+0.10f,0.0f);

	//action
	if (position!=5 && position!=0){motion_proxy_ptr->changeAngles(names, angles, fractionMaxSpeed);}

	if (position==0){

		AL::ALValue angleY;
		AL::ALValue angleP;
		std::vector<float> headYawAngle = motion_proxy_ptr->getAngles("HeadYaw", true);
		std::vector<float> headPitchAngle = motion_proxy_ptr->getAngles("HeadPitch", true);

		ROS_INFO("Y = %f",headYawAngle[0]);

		if (headYawAngle[0]<-0.3f) {angleY = AL::ALValue::array(+0.15f);}
		if (headYawAngle[0]<-0.15f) {angleY = AL::ALValue::array(+0.15f);}
		if (headYawAngle[0]<0.0f) {angleY = AL::ALValue::array(+0.15f);}
		if (headYawAngle[0]<0.15f) {angleY = AL::ALValue::array(+0.15f);}
		if (headYawAngle[0]<0.3f) {angleY = AL::ALValue::array(+0.15f);}
		else {angleY = AL::ALValue::array(-0.6f);
		if (headPitchAngle[0]<0){
			angleP=AL::ALValue::array(-0.20f);
			motion_proxy_ptr->changeAngles("HeadPitch", angleP, fractionMaxSpeed);
		}
		else{
			angleP=AL::ALValue::array(+0.20f);
			motion_proxy_ptr->changeAngles("HeadPitch", angleP, fractionMaxSpeed);
		}
		}
		motion_proxy_ptr->changeAngles("HeadYaw", angleY, fractionMaxSpeed);
	}
}
//////////


//Method that prepares the arm to approach the ball with a brutal function
//and that finishes putting it under the ball with the soft control
void placeArmFornao_behavior_tree(int nao,std::string ip,int arm,Eigen::Vector3f position,AL::ALMotionProxy* motion_proxy_ptr){

	if(arm==0){//RIGHT ARM
		//preplace arm
		AL::ALValue names = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
				,"RElbowYaw","RElbowRoll","RWristYaw","RHand");
		AL::ALValue angles      = AL::ALValue::array(dToR(59.7),dToR(-6.8),dToR(65.2),dToR(80),dToR(104),1.0f);
		setAngles(nao,ip,names,angles,motion_proxy_ptr);

		//sleep
		sleep(5);
		ros::spinOnce();

		std::string name = "RArm";
		int space = 0;
		bool useSensorValues = true;
		std::vector<float> result = motion_proxy_ptr->getPosition(name, space, useSensorValues);
		ROS_INFO("X = %f",position[0]-result[0]);
		ROS_INFO("Y = %f",position[1]-result[1]);
		ROS_INFO("Z = %f",position[2]-result[2]);


		//ser arm
		moveArm(nao,ip,position[0]-result[0],position[0]-result[0],position[0]-result[0]-0.02,0,0,0,"RArm",motion_proxy_ptr);
	}

	if (arm==1){//LEFT ARM
		//preplace arm
		AL::ALValue names = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
				,"LElbowYaw","LElbowRoll","LWristYaw","LHand");
		AL::ALValue angles      = AL::ALValue::array(dToR(49.7),dToR(6.8),dToR(-70),dToR(-80),dToR(-104),1.0f);
		setAngles(nao,ip,names,angles,motion_proxy_ptr);

		//sleep
		sleep(5);
		ros::spinOnce();

		std::string name = "LArm";
		int space = 0;
		bool useSensorValues = true;
		std::vector<float> result = motion_proxy_ptr->getPosition(name, space, useSensorValues);

		//set arm
		moveArm(nao,ip,position[0]-result[0],position[0]-result[0],position[0]-result[0]-0.02,0,0,0,"LArm",motion_proxy_ptr);
	}
}
//////////

//Method to decide which hand to use and if you are well placed
int whichHand(Eigen::Vector3f position){
	//0 not well placed
	//2 left hand
	//1 righthand
	double half_bust_length=0.06;
	double max_reachable_distance=0.35;
	double z_arm=0.05;

	if (position[1]<0){
		//balle côté droit
		if (position[1]<-half_bust_length){//balle pas devant le buste
			if (sqrt(pow(position[0],2.)+pow(position[1]+half_bust_length,2.)+pow	(position[2],2.))<max_reachable_distance) {ROS_INFO("main droite");return 1;}
			else {ROS_INFO("main indeterminee");return 0;}
		}

		else {//balle devant le buste
			if (sqrt(pow(position[0]-max_reachable_distance/2,2.)+pow(position[1]+half_bust_length,2.)+pow(position[2]-z_arm,2.))<max_reachable_distance/2) {
				ROS_INFO("main droite");return 1;}
			else {ROS_INFO("main indeterminee");return 0;}
		}
	}

	else {
		//balle côté gauche
		if (position[1]>half_bust_length){//balle pas devant le buste
			if (sqrt(pow(position[0],2.)+pow(position[1]-half_bust_length,2.)+pow(position[2],2.))<max_reachable_distance) {ROS_INFO("main gauche");return 2;}
			else {ROS_INFO("main indeterminee");return 0;}
		}
		else {//balle devant le buste
			if (sqrt(pow(position[0]-max_reachable_distance/2,2.)+pow(position[1]-half_bust_length,2.)+pow(position[2]-z_arm,2.))<max_reachable_distance/2)
			{ROS_INFO("main gauche");return 2;}
			else {ROS_INFO("main indeterminee");return 0;}
		}
	}
}
//////////





class TakeBall : ROSAction
{
public:
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* robotPosture;

	AL::ALTextToSpeechProxy* speechProxy;

	bool init_;
	int NAO_PORT_p;
	std::string NAO_IP_p;
	int nao_p;
	std::string nao_id_p;
	nao_behavior_tree::BallPosForHead srv3;
	nao_behavior_tree::BallPosForHand srv4;
	nao_behavior_tree::ArmReadyBT srv6;
	ros::Duration time_to_complete_;

	TakeBall(std::string name,int NAO_PORT,std::string NAO_IP,int nao,std::string nao_id) :
		ROSAction(name),
		init_(false),
		time_to_complete_((ros::Duration) 0) 
	{
		ROS_INFO("CREATING TAKEBALL");

		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);

		//AL::ALTextToSpeechProxy speechProxy(NAO_IP,NAO_PORT);
		speechProxy = new AL::ALTextToSpeechProxy(NAO_IP,NAO_PORT);

		NAO_PORT_p=NAO_PORT;
		NAO_IP_p=NAO_IP;
		nao_p=nao;
		nao_id_p=nao_id;

		ROS_INFO("CREATION OK TAKEBALL");
	}

	~TakeBall()
	{
		delete motion_proxy_ptr;
		delete robotPosture;
		delete speechProxy;
	}

	void initialize(){
		init_ = true;
		set_feedback(RUNNING);

		//Stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		speechProxy->say("I WANT A BALL.");

		ROS_INFO("INIT POSTURE");

		robotPosture->goToPosture("StandInit",0.5f);

		ROS_INFO("INITIALIZE");
	}

	void finalize(){
		init_ = false;
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**TakeBall -%- Executing Main Task, elapsed_time: "
				<< dt.toSec() << std::endl;
		std::cout << "**TakeBall -%- elapsed_time: "
				<< time_to_complete_.toSec() << std::endl;

		time_to_complete_ += dt;

		if(!init_){
			initialize();
			speechProxy->say("HE, I WANT THIS BALL.");




			//change here for the color
			srv3.request.min_h=150;
			srv3.request.max_h=180;
			srv4.request.min_h=150;
			srv4.request.max_h=180;


			Eigen::Vector3f position;
			int pos=0;
			ros::spinOnce();

			//search for ball
			ROS_INFO("SEARCH");
			speechProxy->say("WHERE IS THE BALL?");
			time_to_complete_ = (ros::Duration) 0;
			ros::spinOnce();
			while ((pos!=5)&&time_to_complete_.toSec()<60){
				turnHead(NAO_PORT_p,NAO_IP_p,pos,motion_proxy_ptr);
				sleep(1);
				ros::spinOnce();
				ROS_INFO("CHECK SERVER");
				if(client3.call(srv3)){
					pos=(long int)srv3.response.pos;
				}
			}
			if(time_to_complete_.toSec()>60) {
				set_feedback(FAILURE);
				speechProxy->say("I DON'T FIND THE BALL. I GIVE UP.");
				return 1;}

			else{
				speechProxy->say("AH AH. THERE IT IS.");
				//get ball position
				if(client4.call(srv4)){
					position[0]=(float)srv4.response.pos_x;
					position[1]=(float)srv4.response.pos_y;
					position[2]=(float)srv4.response.pos_z;
				}

				//initialization
				int hand=whichHand(position);
				int tries=0;
				ros::spinOnce();

				//wait for ball to come closer
				while ((hand==0) & (tries<5)){
					speechProxy->say("BRING THE BALL HERE.");
					ros::spinOnce();
					//attente
					sleep(3);
					if(client4.call(srv4)){
						position[0]=(float)srv4.response.pos_x-0.02;
						position[1]=(float)srv4.response.pos_y;
						position[2]=(float)srv4.response.pos_z;}
					hand=whichHand(position);
					speechProxy->say("COME ON.");
					tries++;
				}

				ros::spinOnce();

				//si la balle est proche et le choix de la main est fait
				if(tries<5){
					placeArmFornao_behavior_tree(NAO_PORT_p,NAO_IP_p,hand-1,position,motion_proxy_ptr);
					speechProxy->say("Ready?");
					sleep(1);
					speechProxy->say("DROP YOUR BALL.");
					srv6.request.NAO=nao_p;
					srv6.request.arm=hand;//because hand =1/2
					if(client6.call(srv6)){
					}
					speechProxy->say("I PLACED MY ARM");
					set_feedback(SUCCESS);

					finalize();
					return 1;
				}
				else{
					speechProxy->say("OK. YOU DON'T COME. I DON'T TAKE IT. GOOD BYE.");
					set_feedback(FAILURE);
					finalize();
					return 1;}
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

		ros::init(argc, argv, "TakeBall"+nao_id);
		ros::NodeHandle pnh("~");
		
		ros::NodeHandle n;
		std::string headsrv="ballposhead"+nao_id;
		std::string handsrv="ballposhand"+nao_id;
		client3 = n.serviceClient<nao_behavior_tree::BallPosForHead::Response>(headsrv);
		client4 = n.serviceClient<nao_behavior_tree::BallPosForHand::Response>(handsrv);
		client6 = n.serviceClient<nao_behavior_tree::ArmReadyBT::Response>("armReady");

		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		ROS_INFO("INIT TAKEBALL");

		TakeBall server(ros::this_node::getName(),NAO_PORT,NAO_IP,nao,nao_id);

		ROS_INFO("SPIN");

		ros::spin();
	}

	else{
		puts("Error, not enough arguments");
	}

	return 0;
}
