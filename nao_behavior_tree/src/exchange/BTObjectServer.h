/*
 * BTObjectServer.h
 *
 *  Created on: May 20, 2014
 *      Author: geoffray
 */

#ifndef BTOBJECTSERVER_H_
#define BTOBJECTSERVER_H_

#include "BTObjectServer.h"
#include <ros/ros.h>
#include <vector>
#include "nao_behavior_tree/ArmReadyBT.h"
#include "nao_behavior_tree/GetArmReadyBT.h"
#include "nao_behavior_tree/BallPosChangeBT.h"
#include "nao_behavior_tree/BallPosGetBT.h"
using namespace std;

class BTObjectServer {
public:

	ros::NodeHandle nh_;
	ros::ServiceServer srv1;
	ros::ServiceServer srv2;
	ros::ServiceServer srv3;
	ros::ServiceServer srv4;
	vector<vector<int> > Naos;

	BTObjectServer();
	virtual ~BTObjectServer();

	bool ballPosChange(nao_behavior_tree::BallPosChangeBT::Request &req,
			nao_behavior_tree::BallPosChangeBT::Response &res);
	bool ballPosGet(nao_behavior_tree::BallPosGetBT::Request &req,
			nao_behavior_tree::BallPosGetBT::Response &res);
	bool armReady(nao_behavior_tree::ArmReadyBT::Request &req,
			nao_behavior_tree::ArmReadyBT::Response &res);
	bool getArmReady(nao_behavior_tree::GetArmReadyBT::Request &req,
			nao_behavior_tree::GetArmReadyBT::Response &res);

};

#endif /* BTOBJECTSERVER_H_ */
