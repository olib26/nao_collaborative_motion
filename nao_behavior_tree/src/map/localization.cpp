/*
 * localization.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Olivier BALLAND
 */

/* Description:
 *  Receives the image from the webcam.
 *  Publishes the robots’ position.
 *  Receives the robots’ bearing.
 *  Receives the optimal velocity.
 */

/* Topics used:
 *  /odometry
 *  /bearing
 *  /vel
 */

#include <ros/ros.h>
#include "nao_behavior_tree/rosaction.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

#include "nao_behavior_tree/filters/particleFilter.hpp"

#include <nao_behavior_tree/Odometry.h>
#include <nao_behavior_tree/Bearing.h>
#include <nao_behavior_tree/Velocity.h>

#include <nao_behavior_tree/map/localization.hpp>
#include <nao_behavior_tree/map/webcam.hpp>
#include <nao_behavior_tree/map/nao.hpp>

using namespace std;


bool initRobots = false;
bool webcam;


void updateCoordinate(Object object, Robot* r)
{
	// Save previous values
	r->x_temp = r->x;
	r->y_temp = r->y;

	// Update
	r->x = object.x;
	r->y = object.y;
	r->sx = object.sx;
	r->sy = object.sy;

	// Init temp position
	if(!r->initTempPosition)
	{
		r->x_temp = r->x;
		r->y_temp = r->y;
		r->initTempPosition = true;
	}
}


void drawOdometry(IplImage* img, nao_behavior_tree::Odometry odom, Robot r, double k)
{
	CvPoint p1,p2;
	CvScalar color = cvScalar(0,0,255);
	int thickness = 2;

	p1.x = r.y;
	p1.y = r.x;

	double ratio = 1;
	p2.x = (odom.x + odom.vx*ratio)/k + width/2;
	p2.y = -(odom.y + odom.vy*ratio)/k + height/2;

	cvLine(img,p1,p2,color,thickness,CV_AA,0);
}


void drawBearing(IplImage* img, nao_behavior_tree::Odometry odom, Robot r, double k, double bearing)
{
	CvPoint p1,p2;
	CvScalar color = cvScalar(0,255,0);
	int thickness = 2;
	double length = 1;

	p1.x = r.y;
	p1.y = r.x;

	p2.x = (odom.x + length*cos(bearing))/k + width/2;
	p2.y = -(odom.y + length*sin(bearing))/k + height/2;

	cvLine(img,p1,p2,color,thickness,CV_AA,0);
}


void drawVelocity(IplImage* img, nao_behavior_tree::Velocity vel, Robot r, double k)
{
	CvPoint p1,p2;
	CvScalar color = cvScalar(255,0,0);
	int thickness = 2;
	double length = 0.2;

	p1.x = r.y;
	p1.y = r.x;

	p2.x = length*cos(vel.theta)/k + r.y;
	p2.y = -length*sin(vel.theta)/k + r.x;

	cvLine(img,p1,p2,color,thickness,CV_AA,0);
}


void drawPath(IplImage* img, Robot r, int* x_temp, int* y_temp)
{
	CvPoint p1,p2;
	CvScalar color = cvScalar(0,0,255);
	int thickness = 1;

	p1 = cvPoint(*y_temp,*x_temp);
	p2 = cvPoint(r.y,r.x);
	cvLine(img,p1,p2,color,thickness,CV_AA,0);

	*x_temp = r.x;
	*y_temp = r.y;
}


void drawPathEst(IplImage* img, double x, double y, double* x_temp, double* y_temp)
{
	CvPoint p1,p2;
	CvScalar color = cvScalar(0,255,0);
	int thickness = 1;

	p1.x = *x_temp/k + width/2;
	p1.y = -*y_temp/k + height/2;

	p2.x = x/k + width/2;
	p2.y = -y/k + height/2;

	cvLine(img,p1,p2,color,thickness,CV_AA,0);

	*x_temp = x;
	*y_temp = y;
}


double cameraCoef(bool webcam)
{
	double k;
	if(webcam)
	{
		k = webcamHeight/focalLength;
	}
	else
	{
		float f = (float)width/2/tan(HFOV/2.);
		k = cameraHeight/f;
	}
	return k;
}


void computeOdometry(Robot* r, nao_behavior_tree::Odometry* odom, double* timestamp, int* counter , double k)
{
	// Position
	odom->x = (r->y-width/2)*k;
	odom->y = -(r->x-height/2)*k;

	// Velocity
	double dt = ros::Time::now().toSec() - *timestamp;
	*timestamp = ros::Time::now().toSec();
	r->vx_temp += (odom->x - (r->y_temp-width/2)*k)/dt;
	r->vy_temp += (odom->y + (r->x_temp-height/2)*k)/dt;

	if(*counter < nbSamples) {(*counter)++; return;}
	else
	{
		odom->vx = r->vx_temp/nbSamples;
		odom->vy = r->vy_temp/nbSamples;

		r->vx_temp = 0;
		r->vy_temp = 0;

		*counter = 0;
	}

	return;
}


void imageProcessing(IplImage* img)
{
	// Create temporary images
	CvSize sz = cvGetSize(img);
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);
	IplImage* hsv_mask_int = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);

	if(H_MIN > H_MAX)
	{
		CvScalar hsv_min_int = cvScalar(0,S_MIN,V_MIN,0);
		CvScalar hsv_max_int = cvScalar(180,S_MAX,V_MAX,0);

		cvInRangeS(hsv_image,hsv_min_int,hsv_max,hsv_mask);
		cvInRangeS(hsv_image,hsv_min,hsv_max_int,hsv_mask_int);

		cvAdd(hsv_mask,hsv_mask_int,hsv_mask);
	}
	else {cvInRangeS(hsv_image,hsv_min,hsv_max,hsv_mask);}

	// Particle Filter
	PF.imageProcessing(hsv_mask);

	// Update robot coordinate
	updateCoordinate(PF.getObject(0),&r1);
	updateCoordinate(PF.getObject(1),&r2);

	// Show hsv_mask
	//cvNamedWindow("Localization_HSV",1); cvShowImage("Localization_HSV",hsv_mask);
}


IplImage* getImage(bool webcam)
{
	if(!webcam)
	{
		AL::ALValue img = camera_proxy_ptr->getImageRemote(clientName);
		imgHeader->imageData = (char*) img[6].GetBinary();
		camera_proxy_ptr->releaseImage(clientName);
		return cvCloneImage(imgHeader);
	}

	else
	{
		return cvQueryFrame(capture);
	}
}


void on_mouse(int event, int x, int y, int d, void *ptr)
{
	if(event == cv::EVENT_LBUTTONDOWN)
	{
		cv::Point* p = (cv::Point*)ptr;
		p->x = y;
		p->y = x;
	}
}


void receive_bearing1(const nao_behavior_tree::Bearing::ConstPtr &msg)
{
	r1.absoluteBearing = msg->absolute;
	r1.relativeBearing = msg->relative;
	robot2Detected = msg->robotDetected;
}


void receive_bearing2(const nao_behavior_tree::Bearing::ConstPtr &msg)
{
	r2.absoluteBearing = msg->absolute;
	r2.relativeBearing = msg->relative;
	robot1Detected = msg->robotDetected;
}


void receive_vel1(const nao_behavior_tree::Velocity::ConstPtr &msg)
{
	vel1.norm = msg->norm;
	vel1.theta = msg->theta;
	vel1.area = msg->area;
}


void updateEstimation1(AL::ALMotionProxy* motion)
{
	// Init odometry
	if(!init1)
	{
		x_10 = motion->getRobotPosition(true).at(0);
		y_10 = motion->getRobotPosition(true).at(1);
		theta_10 = motion->getRobotPosition(true).at(2);

		init1 = true;
	}

	// Estimate odometry
	x_1 = x_01 + (motion->getRobotPosition(true).at(0)-x_10)*cos(theta_10+theta_01) + (motion->getRobotPosition(true).at(1)-y_10)*sin(theta_10+theta_01);
	y_1 = y_01 - (motion->getRobotPosition(true).at(0)-x_10)*sin(theta_10+theta_01) + (motion->getRobotPosition(true).at(1)-y_10)*cos(theta_10+theta_01);
}


void updateEstimation2(AL::ALMotionProxy* motion)
{
	// Init odometry
	if(!init2)
	{
		x_20 = motion->getRobotPosition(true).at(0);
		y_20 = motion->getRobotPosition(true).at(1);
		theta_20 = motion->getRobotPosition(true).at(2);

		init2 = true;
	}

	// Estimate odometry
	x_2 = x_02 + (motion->getRobotPosition(true).at(0)-x_20)*cos(theta_20+theta_02) + (motion->getRobotPosition(true).at(1)-y_20)*sin(theta_20+theta_02);
	y_2 = y_02 - (motion->getRobotPosition(true).at(0)-x_20)*sin(theta_20+theta_02) + (motion->getRobotPosition(true).at(1)-y_20)*cos(theta_20+theta_02);
}


class Localization : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;

	Localization(std::string name) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0){}

	void finalize()
	{
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**Localization -%- Executing Main Task, elapsed_time: "
		          << dt.toSec() << std::endl;
		std::cout << "**Localization -%- execute_time: "
		          << execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			init_ = true;
			set_feedback(RUNNING);

			// Image
			IplImage* img;

			// Init robots positions
			cv::Point p;
			cv::setMouseCallback("Localization",on_mouse,&p);

			while(((r1.x == 0) | (r2.x == 0)) & ros::ok())
			{
				img = getImage(webcam);
				cvShowImage("Localization",img);

				if((p.x != 0) & (p.y !=0))
				{
					if(r1.x == 0)
					{
						r1.x = p.x;
						r1.y = p.y;
						ROS_INFO("Robot 1 initialized: x = %i, y =%i",r1.x,r1.y);
					}
					else
					{
						r2.x = p.x;
						r2.y = p.y;
						ROS_INFO("Robot 2 initialized: x = %i, y =%i",r2.x,r2.y);
					}

					p.x = p.y = 0;
				}

				cvWaitKey(10);
			}

			// Init objects
			Object* objects = new Object [2];
			objects[0].x = r1.x;
			objects[0].y = r1.y;
			objects[1].x = r2.x;
			objects[1].y = r2.y;

			PF = particleFilter(N,2,MMSE,objects,sx_min,sy_min,sx_max,sy_max,sigma_diffusion,s_diffusion);

			initRobots = true;
		}

		if(initRobots)
		{
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
	ros::init(argc, argv,"Localization");
	ros::NodeHandle nh;

	// Odometry publishers
	ros::Publisher odom1_pub = nh.advertise<nao_behavior_tree::Odometry>("/odometry1",1);
	ros::Publisher odom2_pub = nh.advertise<nao_behavior_tree::Odometry>("/odometry2",1);

	// Bearing subscribers
	ros::Subscriber bearing1_sub = nh.subscribe("/bearing1",1,receive_bearing1);
	ros::Subscriber bearing2_sub = nh.subscribe("/bearing2",1,receive_bearing2);

	// Optimal velocity subscriber
	ros::Subscriber vel1_sub = nh.subscribe("/vel1",1,receive_vel1);

	ros::NodeHandle pnh("~");
	if(argc != 1)
	{
		// Camera selection
		if(atoi(argv[1]) == 0) {webcam = true;}
		if(atoi(argv[1]) == 1) {webcam = false;}

		// Robot parameters
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		// Camera id
		pnh.param("camera",camera,int(0));
	}
	else
	{
		puts("Error, not enough arguments");
		return 0;
	}

	// HSV parameters
	pnh.param("H_MIN",H_MIN,int(0));
	pnh.param("H_MAX",H_MAX,int(0));
	pnh.param("S_MIN",S_MIN,int(0));
	pnh.param("S_MAX",S_MAX,int(0));
	pnh.param("V_MIN",V_MIN,int(0));
	pnh.param("V_MAX",V_MAX,int(0));
	hsv_min = cvScalar(H_MIN,S_MIN,V_MIN,0);
	hsv_max = cvScalar(H_MAX,S_MAX,V_MAX,0);

	// Estimted position
	pnh.param("estimatedPosition",estimatedPosition,bool(false));
	if(estimatedPosition)
	{
		pnh.param("NAO_IP1",NAO_IP1,std::string("127.0.0.1"));
		pnh.param("NAO_PORT1",NAO_PORT1,int(9559));
		pnh.param("NAO_IP2",NAO_IP2,std::string("127.0.0.1"));
		pnh.param("NAO_PORT2",NAO_PORT2,int(9560));

		// Initial odometry of the two robots
		pnh.param("theta_01",theta_01,double(0.0));
		pnh.param("theta_02",theta_02,double(0.0));

		init1 = init2 = false;

		motion_proxy_ptr1 = new AL::ALMotionProxy(NAO_IP1,NAO_PORT1);
		motion_proxy_ptr2 = new AL::ALMotionProxy(NAO_IP2,NAO_PORT2);
	}

	// Init proxy
	if(!webcam)
	{
		camera_proxy_ptr = new AL::ALVideoDeviceProxy(NAO_IP,NAO_PORT);
		// Select top camera
		camera_proxy_ptr->setParam(AL::kCameraSelectID,0);
		// Set Resolution, Color space and FPS
		// Resolution = 640x480
		// Color space = BGR
		// FPS = 30
		clientName = camera_proxy_ptr->subscribe("localization",AL::kVGA,AL::kBGRColorSpace,30);
		// Init image
		imgHeader = cvCreateImageHeader(cvSize(640,480),8,3);
	}

	// Init webcam
	if(webcam)
	{
		capture = cvCaptureFromCAM(camera);
	}

	sleep(1); // Wait for proxy init

	// Images
	IplImage* img;

	// Window
	cvNamedWindow("Localization",1);

	// Launch Server
	Localization server(ros::this_node::getName());

	// Wait for init
	ros::Rate r(100);
	while(!initRobots)
	{
		ros::spinOnce();
		r.sleep();
	}

	img = getImage(webcam);
	CvSize sz = cvGetSize(img);
	// Image size
	height = sz.height;
	width = sz.width;
	// Camera coefficient
	k = cameraCoef(webcam);
	// Init image path
	IplImage* paths = cvCreateImage(sz,8,3);
	cvSet(paths,cvScalar(255,255,255));
	cvNamedWindow("Paths",1);

	// Init timers
	timestamp1 = timestamp2 = ros::Time::now().toSec();

	while(ros::ok()){
		img = getImage(webcam);

		// Image processing
		imageProcessing(img);

		// Compute and Publish odometry
		computeOdometry(&r1,&odom1,&timestamp1,&counter1,k);
		computeOdometry(&r2,&odom2,&timestamp2,&counter2,k);

		odom1_pub.publish(odom1);
		odom2_pub.publish(odom2);

		// Show results
		drawOdometry(img,odom1,r1,k);
		drawOdometry(img,odom2,r2,k);
		if(robot2Detected) {drawBearing(img,odom1,r1,k,r1.absoluteBearing); drawBearing(img,odom1,r1,k,r1.absoluteBearing + r1.relativeBearing);}
		if(robot1Detected) {drawBearing(img,odom2,r2,k,r2.absoluteBearing); drawBearing(img,odom2,r2,k,r2.absoluteBearing + r2.relativeBearing);}
		if(robot2Detected) {drawVelocity(img,vel1,r1,k);}

		// Areas information
		CvFont font;
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,1.0,1.0,0,2,8);
		std::string text = "Area: " + std::to_string(vel1.area);
		if(robot2Detected) {cvPutText(img,text.c_str(),cvPoint(30,30),&font,cvScalar(0,0,255));}

		cvShowImage("Localization",img);


		// Draw paths
		if(counter >= nb)
		{
			if(x_temp1 == 0)
			{
				x_temp1 = r1.x; y_temp1 = r1.y;
				x_temp2 = r2.x; y_temp2 = r2.y;

				if(estimatedPosition)
				{
					x_temp_est1 = x_01 = odom1.x; y_temp_est1 = y_01 = odom1.y;
					x_temp_est2 = x_02 = odom2.x; y_temp_est2 = y_02 = odom2.y;
				}
			}

			drawPath(paths,r1,&x_temp1,&y_temp1);
			drawPath(paths,r2,&x_temp2,&y_temp2);

			if(estimatedPosition)
			{
				updateEstimation1(motion_proxy_ptr1);
				updateEstimation2(motion_proxy_ptr2);

				drawPathEst(paths,x_1,y_1,&x_temp_est1,&y_temp_est1);
				drawPathEst(paths,x_2,y_2,&x_temp_est2,&y_temp_est2);
			}

			counter = 0;
		}
		else {counter++;}

		cvShowImage("Paths",paths);


		cvWaitKey(50);
		ros::spinOnce();
	}


	// Cleanup
	if(!webcam)
	{
		camera_proxy_ptr->unsubscribe(clientName);
		cvReleaseImageHeader(&imgHeader);
	}
	else
	{
		cvReleaseCapture(&capture); // Release capture
	}
	cvDestroyWindow("Localization");
	cvDestroyWindow("Paths");

	return 0;
}
