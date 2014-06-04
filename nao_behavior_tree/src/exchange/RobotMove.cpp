#include "RobotMove.h"
//TO UNDERSTAND THE CODE, WATCH THE REPORT

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "nao_behavior_tree/BallPosForHead.h"
#include "nao_behavior_tree/BallPosForHand.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <Eigen/Dense>


AL::ALMotionProxy* motion_proxy_ptr;
int handball;

RobotMove::RobotMove() {
	// TODO Auto-generated constructor stub

}

RobotMove::~RobotMove() {
	// TODO Auto-generated destructor stub
}




//Method to rotate around myself
	void rotate(){
		AL::ALMotionProxy motion("127.0.0.1");
		AL::ALRobotPostureProxy robotPosture("127.0.0.1");
		robotPosture.goToPosture("StandInit",0.5f);

		int space = 2;
		int axisMask = 63;
		bool isAbsolute = false;
		std::string effector = "Torso";

		AL::ALValue path     = AL::ALValue::array(0.0f, -0.07f, -0.03f, 0.0f, 0.0f, 0.0f);
		AL::ALValue timeList = 2.0f; // seconds
		motion.positionInterpolation(effector, space, path,
				axisMask, timeList, isAbsolute);

		// LLeg motion
		effector   = "LLeg";
		path       = AL::ALValue::array(0.0f,  0.06f,  0.00f, 0.0f, 0.0f, 0.8f);
		timeList   = 2.0f; // seconds
		motion.positionInterpolation(effector, space, path,
				axisMask, timeList, isAbsolute);
	}

	//////////


	//Method to initialize naos
	void initialize(int nao,std::string ip){
		//make proxies //127.0.0.1
		AL::ALRobotPostureProxy postureProxy(ip,nao);

		//go to posture
		postureProxy.goToPosture("StandInit",0.5f);
		}
	//////////


	//Method to move an arm respectively to a given displacement vector
	//huge problems with shitty IK that blocks the arm for nothing
	void moveArm(int nao,std::string ip,float x,float y,float z,float a,float b,float c,std::string effector){
		//effector= "LArm" or "RArm"

		//initilize proxies
		AL::ALMotionProxy motionProxy(ip,nao);

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
		motionProxy.changePosition(effector, space, positionChange,
				fractionMaxSpeed, axisMask);
	}
	//////////


	//Method to bring the arm to a certain point
	//huge problems with shitty IK that blocks the arm for nothing
	void setArm(int nao,std::string ip,float x,float y,float z,float a,float b,float c,std::string effector){
		//effector= "LArm" or "RArm"

		//initilize proxies
		//AL::ALMotionProxy motionProxy("127.0.0.1",nao);
		AL::ALMotionProxy motionProxy(ip,nao);

		int space = 0;//in frame_torso!!
		int axisMask = 63;

		std::vector<float> target(6,0.0f);
		target[0]=x;//0.21f;
		target[1]=y;//-0.5f;
		target[2]=z;//0.03f;
		target[3]=a;
		target[4]=b;
		target[5]=c;
		float fractionMaxSpeed = 0.2f;

		//ROS_INFO("Y = %f",5.);

		//changePosition to move with a vector
		motionProxy.setPosition(effector, space, target,
				fractionMaxSpeed, axisMask);

	}
	//////////


	//Method to control the joint angles
	void setAngles(int nao,std::string ip,AL::ALValue names,AL::ALValue angles){

		//initilize proxies
		//AL::ALMotionProxy motionProxy("127.0.0.1",nao);
		AL::ALMotionProxy motionProxy(ip,nao);


		float fractionMaxSpeed = 0.10f;

		//set arms angles
		motionProxy.setAngles(names, angles, fractionMaxSpeed);

	}
	//////////


	//Method to change the joint angles
	void changeAngles(int nao,std::string ip,AL::ALValue names,AL::ALValue angles){

		//initilize proxies
		AL::ALMotionProxy motionProxy(ip,nao);

		float fractionMaxSpeed = 0.15f;

		//move
		motionProxy.changeAngles(names, angles, fractionMaxSpeed);
	}
	//////////


	//Method to turn head to try putting ball on the screen, we suppose that it will be accessible for the top camera
	void turnHead(int nao,std::string ip,int position){
		//1:ball at top,2right,3bot,4left,5center,0nowhere
		//AL::ALMotionProxy motionProxy("127.0.0.1",9559);
		AL::ALMotionProxy motionProxy(ip,nao);
		float fractionMaxSpeed = 0.10f;

		//joints to manipulate
		AL::ALValue names       = AL::ALValue::array(
				"HeadYaw","HeadPitch");
		//yaw=right-left //pitch=bot-top

		//control angle with the position of the ball
		AL::ALValue angles;
		if (position==1) angles      = AL::ALValue::array(0.0f,-0.10f);
		if (position==2) angles      = AL::ALValue::array(-0.10f,0.0f);
		if (position==3) angles      = AL::ALValue::array(0.0f,0.10f);
		if (position==4) angles      = AL::ALValue::array(+0.10f,0.0f);
		//this control is certainly improvable by putting a proportionnal controller
		//mettre si j'ai le temps un ball tracker

		//action
		if (position!=5 && position!=0){motionProxy.changeAngles(names, angles, fractionMaxSpeed);}

		if (position==0){

			//ROS_INFO("Y = %f",5.);

			AL::ALValue angleY;
			AL::ALValue angleP;
			std::vector<float> headYawAngle = motionProxy.getAngles("HeadYaw", true);
			std::vector<float> headPitchAngle = motionProxy.getAngles("HeadPitch", true);

			ROS_INFO("Y = %f",headYawAngle[0]);

			if (headYawAngle[0]<-0.3f) {angleY = AL::ALValue::array(+0.15f);}
			if (headYawAngle[0]<-0.15f) {angleY = AL::ALValue::array(+0.15f);}
			if (headYawAngle[0]<0.0f) {angleY = AL::ALValue::array(+0.15f);}
			if (headYawAngle[0]<0.15f) {angleY = AL::ALValue::array(+0.15f);}
			if (headYawAngle[0]<0.3f) {angleY = AL::ALValue::array(+0.15f);}
			else {angleY = AL::ALValue::array(-0.6f);
				if (headPitchAngle[0]<0){
					angleP=AL::ALValue::array(-0.20f);
					motionProxy.changeAngles("HeadPitch", angleP, fractionMaxSpeed);
				}
				else{
					angleP=AL::ALValue::array(+0.20f);
					motionProxy.changeAngles("HeadPitch", angleP, fractionMaxSpeed);
				}
			}
			motionProxy.changeAngles("HeadYaw", angleY, fractionMaxSpeed);
		}
	}
	//////////


	//Method that prepares the arm to approach the ball with a brutal function
	//and that finishes putting it under the ball with the soft control
	void setArmPosition(int nao,std::string ip,int effector,float a,float b,float c,float d,float e,float f){
		//effector:1/left 0/rigth

		//top left arm robot
		//		AL::ALValue names1 = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
		//							,"LElbowYaw","LElbowRoll","LWristYaw","LHand");
		//		AL::ALValue angles1 = AL::ALValue::array(-0.15f,0.009f,0.0f,-0.5f,0.0f,0.0f);
		//		setAngles(9560,names1,angles1);


		//bot right arm robot
		if(effector==0){//RIGHT
			AL::ALValue names = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
					,"RElbowYaw","RElbowRoll","RWristYaw","RHand");
			//AL::ALValue angles2      = AL::ALValue::array(0.72f,-0.03f,0.76f,0.77f,1.8f,1.0f);
			AL::ALValue angles      = AL::ALValue::array(a,b,c,d,e,f);


			setAngles(nao,ip,names,angles);
		}

		if(effector==1){//LEFT
			AL::ALValue names = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
					,"LElbowYaw","LElbowRoll","LWristYaw","LHand");
			//AL::ALValue angles2      = AL::ALValue::array(0.72f,-0.03f,0.76f,0.77f,1.8f,1.0f);
			AL::ALValue angles      = AL::ALValue::array(a,b,c,d,e,f);


			setAngles(nao,ip,names,angles);}
	}
	//////////

	//

	//Method to open hand
	void openHand(int nao,std::string ip,int hand){
		std::string effector;
		if (hand==1){effector="LHand";}
		if (hand==0){effector="RHand";}

		AL::ALValue hands = AL::ALValue::array(effector);
		AL::ALValue angles = AL::ALValue::array(1.0f);

		setAngles(nao,ip,hands,angles);
	}
	//////////

	//Method to close hand
	void closeHand(int nao,std::string ip,int hand){
		std::string effector;
		if (hand==1){effector="LHand";}
		if (hand==0){effector="RHand";}

		AL::ALValue hands = AL::ALValue::array(effector);
		AL::ALValue angles = AL::ALValue::array(0.0f);

		setAngles(nao,ip,hands,angles);
	}
	//////////

	//Method to make an exchange from nao1 to nao2
	void exchangeA(int nao1,std::string ip1,int hand1,int nao2, std::string ip2,int hand2){
		openHand(nao1,ip1,hand1);
		sleep(1.);
		closeHand(nao2,ip2,hand2);
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
				if (sqrt(pow(position[0],2.)+pow(position[1]+half_bust_length,2.)+pow(position[2],2.))<max_reachable_distance) {ROS_INFO("main droite");return 1;}
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

	//angle transcriptor
		float dToR(float angle){
			return angle*M_PI/180.;
		}
		//////////

		//Method that prepares the arm to approach the ball with a brutal function
		//and that finishes putting it under the ball with the soft control
		void placeArmForExchange(int nao,std::string ip,int arm,Eigen::Vector3f position){
			//a modifier selon quel bras possède la balle

			if(arm==0){//RIGHT ARM
				//preplace arm
				AL::ALValue names = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
						,"RElbowYaw","RElbowRoll","RWristYaw","RHand");
				AL::ALValue angles      = AL::ALValue::array(dToR(59.7),dToR(-6.8),dToR(65.2),dToR(80),dToR(104),1.0f);
				setAngles(nao,ip,names,angles);

				//sleep
				sleep(5);
				ros::spinOnce();

				AL::ALMotionProxy motionProxy(ip,nao);
				std::string name = "RArm";
				int space = 0;
				bool useSensorValues = true;
				std::vector<float> result = motionProxy.getPosition(name, space, useSensorValues);
				ROS_INFO("X = %f",position[0]-result[0]);
				ROS_INFO("Y = %f",position[1]-result[1]);
				ROS_INFO("Z = %f",position[2]-result[2]);


				//placer le bras
				//setArm(9559,0.25f,-0.07f,0.0f,effector);
				//setArm(nao,ip,position[0],position[1],position[2],0.0f,0.0f,0.0f,"RArm");
				//NOTE: set arm fout un angle de merde à 0;0;0 il faut donc faire un move arm ou bien comprendre comment marchent les angles
				moveArm(nao,ip,position[0]-result[0],position[0]-result[0],position[0]-result[0]-0.02,0,0,0,"RArm");
			}
			if (arm==1){//LEFT ARM
				//preplace arm
				AL::ALValue names = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
						,"LElbowYaw","LElbowRoll","LWristYaw","LHand");
				AL::ALValue angles      = AL::ALValue::array(dToR(49.7),dToR(6.8),dToR(-70),dToR(-80),dToR(-104),1.0f);
				setAngles(nao,ip,names,angles);

				//sleep
				sleep(5);
				ros::spinOnce();

				AL::ALMotionProxy motionProxy(ip,nao);
				std::string name = "LArm";
				int space = 0;
				bool useSensorValues = true;
				std::vector<float> result = motionProxy.getPosition(name, space, useSensorValues);

				//placer le bras
				//setArm(9559,0.25f,-0.07f,0.0f,effector);
				moveArm(nao,ip,position[0]-result[0],position[0]-result[0],position[0]-result[0]-0.02,0,0,0,"LArm");
			}
		}
	//////////

		//prepare arm to hold the ball
		void holdArm(int nao,std::string ip,int arm){
			if(arm==0){//RIGHT ARM
							//preplace arm
							AL::ALValue names = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
									,"RElbowYaw","RElbowRoll","RWristYaw");
							AL::ALValue angles      = AL::ALValue::array(dToR(80.2),dToR(-17.2),dToR(43.5),dToR(89),dToR(0));
							setAngles(nao,ip,names,angles);
							sleep(2);
							angles      = AL::ALValue::array(dToR(-8.0),dToR(-3),dToR(43.5),dToR(89),dToR(0));
							setAngles(nao,ip,names,angles);
							sleep(2);
							angles      = AL::ALValue::array(dToR(-20.0),dToR(-1),dToR(43.5),dToR(3),dToR(0));
							setAngles(nao,ip,names,angles);
			}
			if (arm==1){//LEFT ARM
							//preplace arm
							AL::ALValue names = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
									,"LElbowYaw","LElbowRoll","LWristYaw");
							AL::ALValue angles      = AL::ALValue::array(dToR(80.2),dToR(17.2),dToR(-43.5),dToR(-89),dToR(0));
							setAngles(nao,ip,names,angles);
							sleep(2);
							angles      = AL::ALValue::array(dToR(-8.0),dToR(3),dToR(-43.5),dToR(-89),dToR(0));
							setAngles(nao,ip,names,angles);
							sleep(2);
							angles      = AL::ALValue::array(dToR(-20.0),dToR(1),dToR(-43.5),dToR(-3),dToR(0));
							setAngles(nao,ip,names,angles);
			}

		}


		//////////

		//passes the ball to the other hand
		void changeHand(int nao,std::string ip,int arm){

			AL::ALMotionProxy motionProxy(ip,nao);
			motionProxy.setSmartStiffnessEnabled(false);

				if(arm==0){
				openHand(nao,ip,1);

				AL::ALValue namesL = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
						,"LElbowYaw","LElbowRoll","LWristYaw");
				//AL::ALValue anglesL      = AL::ALValue::array(dToR(16.6),dToR(0.5),dToR(-40),dToR(-44),dToR(-48));
				AL::ALValue anglesL      = AL::ALValue::array(dToR(66),dToR(2),dToR(-40),dToR(-61),dToR(-48));
				setAngles(nao,ip,namesL,anglesL);

				sleep(1);

				AL::ALValue namesR = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
						,"RElbowYaw","RElbowRoll","RWristYaw");
				//AL::ALValue anglesR      = AL::ALValue::array(dToR(16.6),dToR(-0.5),dToR(40),dToR(44),dToR(36));
				AL::ALValue anglesR      = AL::ALValue::array(dToR(64),dToR(-2),dToR(40),dToR(57),dToR(36));
				setAngles(nao,ip,namesR,anglesR);

				sleep(2);

				motionProxy.setSmartStiffnessEnabled(true);
				sleep(1);
				closeHand(nao,ip,1);
				sleep(2);
				closeHand(nao,ip,1);
				sleep(2);
				openHand(nao,ip,0);
				sleep(2);
				handball=1;
				}

				else{
					openHand(nao,ip,0);

				AL::ALValue namesR = AL::ALValue::array("RShoulderPitch", "RShoulderRoll"
						,"RElbowYaw","RElbowRoll","RWristYaw");
				//AL::ALValue anglesR      = AL::ALValue::array(dToR(16.6),dToR(-0.5),dToR(40),dToR(44),dToR(36));
				AL::ALValue anglesR      = AL::ALValue::array(dToR(64),dToR(-2),dToR(40),dToR(57),dToR(36));
				setAngles(nao,ip,namesR,anglesR);

				sleep(1);

				AL::ALValue namesL = AL::ALValue::array("LShoulderPitch", "LShoulderRoll"
						,"LElbowYaw","LElbowRoll","LWristYaw");
				//AL::ALValue anglesL      = AL::ALValue::array(dToR(16.6),dToR(0.5),dToR(-40),dToR(-44),dToR(-48));
				AL::ALValue anglesL      = AL::ALValue::array(dToR(66),dToR(2),dToR(-40),dToR(-61),dToR(-48));
				setAngles(nao,ip,namesL,anglesL);

				sleep(2);

				motionProxy.setSmartStiffnessEnabled(true);
				sleep(1);
				closeHand(nao,ip,0);
				sleep(2);
				closeHand(nao,ip,0);
				sleep(2);
				openHand(nao,ip,1);
				sleep(2);
				handball=0;
				}
				initialize(nao,ip);
		}
		//////////



	//prefixed exchange for top-bottom from nao1 to nao2
	void test1(int nao1,std::string ip1,int hand1,int nao2,std::string ip2,int hand2){

		initialize(nao1,ip1);
		openHand(nao1,ip1,hand1);
		sleep(3);
		closeHand(nao1,ip1,hand1);
		initialize(nao2,ip2);


		setArmPosition(nao1,ip1,0,-0.35f,-0.009f,0.0f,+0.5f,0.0f,0.0f);

		sleep(4);
		Eigen::Vector3f position; position[0]=0.16f;position[1]=-0.08f;position[2]=0.01f;
		placeArmForExchange(nao2,ip2,1,position);

		exchangeA(nao1,ip1,hand1,nao2,ip2,hand2);
	}
	///////////

	//prefixed exchange for top bottom from human to nao2 without grasping
	void test2(int nao2,std::string ip2,int hand2){

			initialize(nao2,ip2);

			//choisir position de imageconverter
			Eigen::Vector3f position; position[0]=0.16f;position[1]=-0.08f;position[2]=0.01f;
			placeArmForExchange(nao2,ip2,1,position);
			sleep(3);
			closeHand(nao2,ip2,hand2);
		}
	//////////

	//Method with client-server exchange with 1 nao
	//RACKET D UN HUMAIN PAR UN NAO
	//void test3(int nao1,std::string ip1,int min_h,int max_h,AL::ALTextToSpeechProxy speechProxy){
	void test3(int nao,std::string ip, std::string name,int min_h,int max_h,AL::ALTextToSpeechProxy speechProxy){
		//initialization
		initialize(nao,ip);
		ROS_INFO("YOU");
		speechProxy.say("EH YOU");
		sleep(2);
		ros::NodeHandle n;
//======>ici j'ai rajouté les name
		ros::ServiceClient client1 = n.serviceClient<nao_behavior_tree::BallPosForHead::Response>("ballposhead"+name);
		ros::ServiceClient client2 = n.serviceClient<nao_behavior_tree::BallPosForHand::Response>("ballposhand"+name);

		ros::spinOnce();

		//sends info on H for HSV to imageConverter
		nao_behavior_tree::BallPosForHead srv1;
		nao_behavior_tree::BallPosForHand srv2;
		srv1.request.min_h=min_h;
		srv1.request.max_h=max_h;
		srv2.request.min_h=min_h;
		srv2.request.max_h=max_h;

		Eigen::Vector3f position;
		int pos=0;
		ROS_INFO("GIVE ME YOUR BALL");
		speechProxy.say("EH YOU GIMME YOUR BALL");

		//chercher la balle
		while (pos!=5){
			turnHead(nao,ip,pos);
			sleep(1);
			if(client1.call(srv1)){
				pos=(long int)srv1.response.pos;
				ROS_INFO("ball pos on screen=%i",pos);
			}
		}
		//on suppose la balle trouvée


		//obtenir la position de la balle
		if(client2.call(srv2)){
			position[0]=(float)srv2.response.pos_x;
			position[1]=(float)srv2.response.pos_y;
			position[2]=(float)srv2.response.pos_z;
			ROS_INFO("x: %f",position[0]);
			ROS_INFO("y: %f",position[1]);
			ROS_INFO("z: %f",position[2]);
		}

		//initialisation
		int hand=whichHand(position);
		int tries=0;

		//attente du rapprochement de la balle pour choisir la main
		while ((hand==0) & (tries<5)){
			ROS_INFO("COME CLOSER");
			ros::spinOnce();
			//attente
			sleep(3);
			if(client2.call(srv2)){
				position[0]=(float)srv2.response.pos_x-0.02;
				position[1]=(float)srv2.response.pos_y;
				position[2]=(float)srv2.response.pos_z;}
			hand=whichHand(position);
			tries++;
		}
		//to memorize the hand of the ball
		handball=hand-1;


		//si la balle est proche et le choix de la main est fait
		if(tries<5){
		ros::spinOnce();
		placeArmForExchange(nao,ip,hand-1,position);
		ROS_INFO("DROP THE BALL");
		speechProxy.say("DROP YOUR BALL");
		sleep(3);
		closeHand(nao,ip,hand-1);
		//update the hand holding the ball
		handball=hand-1;
		}
		ROS_INFO("HAHAHA BOLOS");
		speechProxy.say("HA HA HA I GOT THE BALL");
		ros::spinOnce();
	}

	//Method with client-server exchange with 2 naos
	//nao1 rackets nao2
	void test4(int nao1,std::string ip1,std::string name1,AL::ALTextToSpeechProxy speechProxy1,  int nao2,std::string ip2,AL::ALTextToSpeechProxy speechProxy2,  int min_h,int max_h){
			//initialization
		speechProxy2.say("SECOND PART");
			initialize(nao1,ip1);
			ROS_INFO("EH TYOU");
			ros::spinOnce();
			speechProxy2.say("Dear");
			sleep(2);
			speechProxy1.say("YES?");
			sleep(2);

			ros::NodeHandle n;
//======>idem, ici j'ai rajouté les name1 ppour préciser le serveur attaché
			std::string headsrv="ballposhead"+name1;
			std::string handsrv="ballposhand"+name1;
			ros::ServiceClient client3 = n.serviceClient<nao_behavior_tree::BallPosForHead::Response>(headsrv);
			ros::ServiceClient client4 = n.serviceClient<nao_behavior_tree::BallPosForHand::Response>(handsrv);

			ros::spinOnce();

			//sends info on H for HSV to imageConverter
			nao_behavior_tree::BallPosForHead srv3;
			nao_behavior_tree::BallPosForHand srv4;
			srv3.request.min_h=min_h;
			srv3.request.max_h=max_h;
			srv4.request.min_h=min_h;
			srv4.request.max_h=max_h;

			Eigen::Vector3f position;
			int pos=0;
			ROS_INFO("PREPARATION");
			ros::spinOnce();
			speechProxy2.say("Would you like to marry me");
			sleep(2);
			speechProxy1.say("Oh yes, this is the most wonderful day of my life.");
			sleep(2);
			//setArmPosition(nao2,ip2,handball,-0.35f,-0.009f,0.0f,+0.5f,0.0f,0.0f);
			holdArm(nao2,ip2,handball);
			sleep(4);

			ros::spinOnce();

			//chercher la balle
			ROS_INFO("second robot search for the ball");
			ros::spinOnce();
			while (pos!=5){
				ROS_INFO("cherche");
				turnHead(nao1,ip1,pos);
				sleep(1);
				ros::spinOnce();
				if(client3.call(srv3)){
					pos=(long int)srv3.response.pos;
					ROS_INFO("ball pos on screen=%i",pos);
				}
			}
			//on suppose la balle trouvée
			//cette condition sera peut être à modifier au cas où le nao n'arrive pas à centrer la balle

			ros::spinOnce();

			//obtenir la position de la balle
			if(client4.call(srv4)){
				position[0]=(float)srv4.response.pos_x;
				position[1]=(float)srv4.response.pos_y;
				position[2]=(float)srv4.response.pos_z;
				ROS_INFO("x: %f",position[0]);
				ROS_INFO("y: %f",position[1]);
				ROS_INFO("z: %f",position[2]);
			}

			//initialisation
			int hand=whichHand(position);
			int tries=0;
			ros::spinOnce();

			//attente du rapprochement de la balle pour choisir la main
			while ((hand==0) & (tries<5)){
				ROS_INFO("COME CLOSER");
				speechProxy1.say("Come here");
				ros::spinOnce();
				//attente
				sleep(3);
				if(client4.call(srv4)){
					position[0]=(float)srv4.response.pos_x-0.02;
					position[1]=(float)srv4.response.pos_y;
					position[2]=(float)srv4.response.pos_z;}
				hand=whichHand(position);
				speechProxy2.say("Come on");
				tries++;
			}

			ros::spinOnce();

			//si la balle est proche et le choix de la main est fait
			if(tries<5){
			placeArmForExchange(nao1,ip1,hand-1,position);
			ROS_INFO("DROP THE BALL");
			speechProxy2.say("Ready?");
			sleep(1);
			speechProxy1.say("yes my love");
			openHand(nao2,ip2,handball);
			sleep(0.2);
			closeHand(nao1,ip1,hand-1);
			//update the hand holding the ball
			handball=hand-1;
			}

			speechProxy2.say("HA HA HA HA HA HA Now you have to do all that I desire! Clean my pants");
			speechProxy1.say("For some reasons, I feel I have been manipulated.");
			sleep(5);
			initialize(nao1,ip1);
			initialize(nao2,ip2);
			ROS_INFO("END OF THE MARIAGE");
		}

int main(int argc, char** argv) {
	ros::init(argc, argv, "RobotMove");

	AL::ALTextToSpeechProxy speechProxy1("192.168.1.19",9559);
	AL::ALTextToSpeechProxy speechProxy2("192.168.1.127",9559);
	speechProxy1.setLanguage("English");
	speechProxy1.setVolume(1.0);
	speechProxy2.setLanguage("English");
	speechProxy2.setVolume(1.0);
	speechProxy1.say("Let's rock, dude");

	//motion_proxy_ptr = new AL::ALMotionProxy("192.168.1.19",9559);
	//motion_proxy_ptr = new AL::ALMotionProxy("127.0.0.1",9560);

	//test1(9559,"192.168.1.127","LHand",9559,"192.168.1.19","RHand");
	//test1(9559,"127.0.0.1","LHand",9560,"127.0.0.1","RHand");

//	initialize(9559,"192.168.1.19");
//	openHand(9559,"192.168.1.19",0);
//	sleep(2);
//	openHand(9559,"192.168.1.19",1);
//	sleep(2);
//	closeHand(9559,"192.168.1.19",0);
//	sleep(2);
//	closeHand(9559,"192.168.1.19",1);
	test3(9559,"192.168.1.19","1",150,180,speechProxy1);
	changeHand(9559,"192.168.1.19",handball);
	test4(9559,"192.168.1.127","2",speechProxy2,  9559,"192.168.1.19",speechProxy1,  150,175);

	ros::spin();
	return 0;
}
