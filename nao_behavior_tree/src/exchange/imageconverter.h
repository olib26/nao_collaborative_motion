/*
 * imageconverter.h
 *
 *  Created on: Feb 4, 2014
 *      Author: geoffray
 */

#ifndef IMAGECONVERTER_H_
#define IMAGECONVERTER_H_


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
#include "WQUPC.h"
#include "imageconverter.h"
#include "nao_behavior_tree/BallPosForHead.h"
#include "nao_behavior_tree/BallPosForHand.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <Eigen/Dense>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;


class image_converter {
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	Mat imgcam;
	vector<vector<int> >meltingList;

	image_converter();
	virtual ~image_converter();

	void publishImage(const cv::Mat img,const char* WINDOW);
	void receiveImage(const sensor_msgs::ImageConstPtr& msg);
	Mat test1(Mat img_cam);
	Mat test2(Mat img_cam);
	Mat test3(Mat img_cam);
	Mat test4(Mat img_cam);
	Mat test5(Mat img_cam);
	void test6();
	void test7(Mat img_cam);

	Mat invertImage(Mat img_cam);
	Mat subsampleOnce(Mat img_cam);
	Mat subsampleMulti(Mat img_cam, int nbsub);
	Mat colorDetectionRGB(Mat img_cam, double R, double G, double B,double thres);
	Mat colorDetectionHSV(Mat img_cam,double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX);
	int thresCondOKRGB(Mat img_cam,int i,int j,double R, double G, double B,double thres);
	int thresCondOKHSV(Mat img_cam,int i, int j, double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX);
	Mat blurImage(Mat img_cam,int size);

	Mat getContourBall(Mat img_cam,double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX);
	Mat getContourBall2(Mat img_cam,double R, double G, double B,double thres);
	Mat getContourBall3(Mat img_cam,double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX);

	vector<vector<vector<int> > > getChains(Mat contoursImage);
	Mat drawChains(vector<vector<vector<int> > > chains,Mat contoursImage);
	vector<vector<vector<int> > > meltChains(vector<vector<vector<int> > > chains,vector<vector<int> > meltingList);
	vector<vector<int> > chooseChain(vector<vector<vector<int> > > chains,int kindOfLongestChain,Mat contoursImage);
	vector<vector<int> > getLongestChain(Mat contoursImage,int kindOfLongestChain);
	vector <double> ballInfo (vector<vector<int> > contours);
	Eigen::Vector3f get3DcoordBall (vector <double> ballinfo);
	Mat lowResBallFindingRGB(Mat img_cam,double R, double G, double B,double thresColor,int nbSubsampling);
	Mat lowResBallFindingHSV(Mat img_cam,int HMIN,int HMAX,int SMIN,int SMAX,int VMIN,int VMAX,int nbSubsampling);
	int isBallInScreen(Mat lowRes,double thresBallPresence);

	Mat drawMatFromList (vector<vector<int> > contours,Mat contoursImage);
	Mat zoom2(Mat img);

	bool ballPosForHead(nao_behavior_tree::BallPosForHead::Response &req,
			nao_behavior_tree::BallPosForHead::Response &res);
	bool ballPosForHand(nao_behavior_tree::BallPosForHand::Response &req,
			nao_behavior_tree::BallPosForHand::Response &res);

};

#endif /* IMAGECONVERTER_H_ */
