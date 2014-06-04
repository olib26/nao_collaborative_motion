#include "BTObjectServer.h"
#include <ros/ros.h>
#include <vector>
#include "nao_behavior_tree/ArmReadyBT.h"
#include "nao_behavior_tree/GetArmReadyBT.h"
#include "nao_behavior_tree/BallPosChangeBT.h"
#include "nao_behavior_tree/BallPosGetBT.h"

using namespace std;
int N;

//au cas ou autre version dans le collaborative_motion_nao
//class BTObjectServer {
	
	//number of robots at the beginning

//note: if multiple objects have to be managed =>
//the "actions.cpp" (in src) can be extended to the case of multiple objects
//currently in the "actions.cpp" files, the only object is given the number 1
//one would have to modify these files to ask the object number and update it when taking an object. it should be fine to use a ros topic.


//public:
	BTObjectServer :: BTObjectServer() {
	
	srv1=nh_.advertiseService("ballPosGet", &BTObjectServer::ballPosGet,this);
	srv2=nh_.advertiseService("ballPosChange", &BTObjectServer::ballPosChange,this);
	srv3=nh_.advertiseService("armReady", &BTObjectServer::armReady,this);
	srv4=nh_.advertiseService("getArmReady", &BTObjectServer::getArmReady,this);
	//N=n;
	vector<int> initVector (3,0);
	ROS_INFO("N = %i",N);
	for(int i=0;i<N;i++){Naos.push_back(initVector);}
	}

	BTObjectServer :: ~BTObjectServer() {
		}

	//SERVERS
	//this server updates the position of the objects
	bool BTObjectServer::ballPosChange(nao_behavior_tree::BallPosChangeBT::Request &req,
			nao_behavior_tree::BallPosChangeBT::Response &res){

		int obj= req.object;ROS_INFO("obj = %i",obj);
		int NAO= req.NAO-1;ROS_INFO("NAO = %i",NAO);
		int hand= req.hand;ROS_INFO("hand = %i",hand);
		Naos.at(NAO).at(0)=obj;
		Naos.at(NAO).at(1)=hand;
		return true;
	}
	
	//this server gives the NAO's hand holding the object
	bool BTObjectServer::ballPosGet(nao_behavior_tree::BallPosGetBT::Request &req,
			nao_behavior_tree::BallPosGetBT::Response &res){

		int NAO= req.NAO-1;
		res.hand=Naos.at(NAO).at(1);
		return true;
	}

	//this server updates the risen arms
	bool BTObjectServer::armReady(nao_behavior_tree::ArmReadyBT::Request &req,
			nao_behavior_tree::ArmReadyBT::Response &res){

		ROS_INFO("arm ready");
		int NAO= req.NAO-1;ROS_INFO("NAO = %i",NAO);
		int arm=req.arm;ROS_INFO("arm = %i",arm);
		Naos.at(NAO).at(2)=arm;
		return true;
	}

	//this server gives the NAO's risen arm
	bool BTObjectServer::getArmReady(nao_behavior_tree::GetArmReadyBT::Request &req,
			nao_behavior_tree::GetArmReadyBT::Response &res){
		int NAO= req.NAO-1;
		res.arm=Naos.at(NAO).at(2);
		return true;
	}
//
//};

int main(int argc, char** argv) {
	if(argc != 1)
{
	N=atoi(argv[1]);
	ros::init(argc, argv, "nao_info_server");
	ROS_INFO("OBJECT SERVER BEGIN");
	BTObjectServer os;//initial number of NAO
	ros::spin();
}
	return 0;
}
