/*
 * localization.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Olivier BALLAND
 */

#include <ros/ros.h>
#include "nao_behavior_tree/rosaction.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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


bool initRobots;
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


/*
void particleFilter(IplImage* img, Particle* particles, Robot* robot, Robot* other)
{
	// Diffusion
	for(int i = 0; i < N; i++)
	{
		// Position
		particles[i].x += sigma_diffusion*normalRandom();
		if(particles[i].x < 0) {particles[i].x = 0;}
		if(particles[i].x > height-1) {particles[i].x = height-1;}

		particles[i].y += sigma_diffusion*normalRandom();
		if(particles[i].y < 0) {particles[i].y = 0;}
		if(particles[i].y > width-1) {particles[i].y = width-1;}

		// Size
		particles[i].sx += s_diffusion*normalRandom();
		if(particles[i].sx < sx_min) {particles[i].sx = sx_min;}
		if(particles[i].sx > sx_max) {particles[i].sx = sx_max;}

		particles[i].sy += s_diffusion*normalRandom();
		if(particles[i].sy < sy_min) {particles[i].sy = sy_min;}
		if(particles[i].sy > sy_max) {particles[i].sy = sy_max;}
	}

	// Weighting
	double norm = 0;
	int** integral = integralImage(img);
	for(int i = 0; i < N; i++)
	{
		particles[i].w = evaluate(particles[i].x,particles[i].y,particles[i].sx,particles[i].sy,integral,robot,other);
		norm += particles[i].w;
	}
	for(int i = 0; i < N; i++)
	{
		particles[i].w = particles[i].w/norm;
	}

	// Resampling
	double cdf[N];
	cdf[0] = particles[0].w;
	for(int i = 1; i < N; i++)
	{
		cdf[i] = cdf[i-1] + particles[i].w;
	}

	double r = uniformRandom()/N;
	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			if(cdf[j] >= r)
			{
				particles[i] = particles[j];
				break;
			}
		}
		particles[i].w = (double)1/N;
		r += (double)1/N;
	}

	// Update object coordinate
	robotCoordinate(particles,robot);
}
*/


void imageProcessing(IplImage* img)
{
	// Create temporary images
	CvSize sz = cvGetSize(img);
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	cvInRangeS(hsv_image,hsv_min,hsv_max,hsv_mask);

	// Particle Filter
	PF.imageProcessing(hsv_mask);

	// Update robot coordinate
	updateCoordinate(PF.getObject(0),&r1);
	updateCoordinate(PF.getObject(1),&r2);
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
			set_feedback(RUNNING);
			init_ = true;

			// Image
			IplImage* img;

			// Init robots positions
			cv::Point p;
			cv::setMouseCallback("Odometry",on_mouse,&p);

			while(((r1.x == 0) | (r2.x == 0)) & ros::ok())
			{
				img = getImage(webcam);
				cvShowImage("Odometry",img);

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
	cvNamedWindow("Odometry",1);

	// Launch Server
	Localization server(ros::this_node::getName());

	// Wait for init
	ros::Rate r(100);
	while(!initRobots)
	{
		ros::spinOnce();
		r.sleep();
	}

	// Init timers
	timestamp1 = timestamp2 = ros::Time::now().toSec();

	while(ros::ok()){
		img = getImage(webcam);

		CvSize sz = cvGetSize(img);

		// Image size
		height = sz.height;
		width = sz.width;

		// Camera coefficient
		k = cameraCoef(webcam);

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
		if(robot1Detected) {drawBearing(img,odom2,r2,k,r2.absoluteBearing); drawBearing(img,odom1,r1,k,r2.absoluteBearing + r2.relativeBearing);}
		if(robot2Detected) {drawVelocity(img,vel1,r1,k);}
		cvShowImage("Odometry",img);

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
	cvDestroyWindow("Odometry");

	return 0;
}
