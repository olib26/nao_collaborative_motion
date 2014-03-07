/*
 * obstacles.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Olivier BALLAND
 */

#include <ros/ros.h>

#include <iostream>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

#include <nao_behavior_tree/Odometry.h>
#include <nao_behavior_tree/Bearing.h>

#include <nao_behavior_tree/map/obstacles.hpp>
#include <nao_behavior_tree/map/webcam.hpp>
#include <nao_behavior_tree/map/nao.hpp>


void showObstacles(IplImage* img)
{
	cv::Point p1,p2;
	CvScalar color = cvScalar(0,0,255);
	int thickness = 2;

	for(int i = 0; i < obstacles.size(); i++)
	{
		std::vector<cv::Point> points = obstacles.at(i).pointsImage;
		for(int j = 0; j < points.size()-1; j++)
		{
			p1 = points.at(j);
			p2 = points.at(j+1);
			cvLine(img,p1,p2,color,thickness,CV_AA,0);
		}
		p1 = points.back();
		p2 = points.at(0);
		cvLine(img,p1,p2,color,thickness,CV_AA,0);
	}
}


void receive_odometry1(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r1.x = msg->x;
	r1.y = msg->y;
	r1.vx = msg->vx;
	r1.vy = msg->vy;
}


void receive_odometry2(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r2.x = msg->x;
	r2.y = msg->y;
	r2.vx = msg->vx;
	r2.vy = msg->vy;
}


void receive_bearing1(const nao_behavior_tree::Bearing::ConstPtr &msg)
{
	r1.theta = msg->absolute;
	robot2Detected = msg->robotDetected;
}


void receive_bearing2(const nao_behavior_tree::Bearing::ConstPtr &msg)
{
	r2.theta = msg->absolute;
	robot1Detected = msg->robotDetected;
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


Point pixelToPoint(cv::Point pixel)
{
	Point p;
	return p;
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
		// New point
		cv::Point pixel(y,x);
		currentObstacle.pointsImage.push_back(pixel);
		
		Point p = pixelToPoint(pixel);
		currentObstacle.pointsWorld.push_back(p);
	}
	
	if(event == cv::EVENT_RBUTTONDOWN)
	{	
		obstacles.push_back(currentObstacle);
		
		// New obstacle
		currentObstacle.pointsWorld.clear();
		currentObstacle.pointsImage.clear();
	}
}


void creation(IplImage* img)
{
	cvShowImage("Camera_Output",img);

	// Init mouse callback
	cv::Point p;
	cv::setMouseCallback("Camera_Output",on_mouse,&p);
	
	int key;
	while(ros::ok())
	{
		showObstacles(img);
		key = cvWaitKey(100);
		if(key == 27) {break;} // Esc key
	}
	
	// Save obstacles
	obstacles.resize(maxObstacles);
	std::ofstream os("/home/olivier/ros_workspace/map/obstacles.dat",std::ios::binary);
	os.write(reinterpret_cast<const char*>(&(obstacles[0])),obstacles.size()*sizeof(Obstacle));
	os.close();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"obstacles");
	ros::NodeHandle nh;

	// Odometry subscribers
	ros::Subscriber odom1_sub = nh.subscribe("/odometry1",1,receive_odometry1);
	ros::Subscriber odom2_sub = nh.subscribe("/odometry2",1,receive_odometry2);

	// Bearing subscribers
	ros::Subscriber bearing1_sub = nh.subscribe("/bearing1",1,receive_bearing1);
	ros::Subscriber bearing2_sub = nh.subscribe("/bearing2",1,receive_bearing2);

	// Mode
	int mode = NORMAL;

	ros::NodeHandle pnh("~");
	if(argc != 1)
	{
		// Mode selection
		if(atoi(argv[1]) == 0) {mode = CREATION;}
		if(atoi(argv[1]) == 1) {mode = NORMAL;}

		// Robot parameters
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));

		// Camera selection
		pnh.param("webcam",webcam,bool(true));
			
		// Camera id
		pnh.param("cameraId",cameraId,int(0));
	}
	else
	{
		puts("Error, not enough arguments");
		return 0;
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
		clientName = camera_proxy_ptr->subscribe("obstacles",AL::kVGA,AL::kBGRColorSpace,30);
		// Init image
		imgHeader = cvCreateImageHeader(cvSize(640,480),8,3);
	}

	// Init webcam
	if(webcam)
	{
		capture = cvCaptureFromCAM(cameraId);
	}

	// Window
	cvNamedWindow("Camera_Output",1);

	// Image
	IplImage* img;

	 // Wait for proxy init
	sleep(1);

	// Init image size
	img = getImage(webcam);
	CvSize sz = cvGetSize(img);
	height = sz.height;
	width = sz.width;

	// Camera Coef
	k = cameraCoef(webcam);

	
	if(mode == CREATION) {creation(img);}
	if(mode == NORMAL)
	{
		// Load obstacles
		obstacles.resize(maxObstacles);
		std::ifstream is("/home/olivier/ros_workspace/map/obstacles.dat",std::ios::binary);
		is.read(reinterpret_cast<char*>(&(obstacles[0])),obstacles.size()*sizeof(Obstacle));
		is.close();

		// Test
		cvShowImage("Camera_Output",img);
		showObstacles(img);

		ros::Rate r(10);
		while(ros::ok())
		{
			ros::spinOnce();
			r.sleep();
		}
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
	cvDestroyWindow("Camera_Output"); // Destroy Window

	return 0;
}
