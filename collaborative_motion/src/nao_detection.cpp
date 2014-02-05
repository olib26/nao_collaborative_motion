/*
 * nao_detection.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_srvs/Empty.h>

#include <collaborative_motion/PosImage.h>
#include "headers/nao_detection.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace collaborative_motion;


double uniformRandom()
{
	return (double)(rand())/(double)(RAND_MAX);
}


double normalRandom()
{
	// Box-Muller transform
	double u1 = uniformRandom();
	double u2 = uniformRandom();
	return cos(2*M_PI*u2)*sqrt(-2.*log(u1));
}


void robotCoordinate()
{
	x = 0;
	y = 0;
	for(int i = 0; i < N; i++)
	{
		x += Px[i];
		y += Py[i];
	}
	x = x/N+sx/2;
	y = y/N+sy/2;
}


void showRobot(IplImage* img)
{
	CvPoint c1,c2;

	c1 = cvPoint(y-5,x-5);
	if((x-5) < 0) {c1.y = 0;}
	if((y-5) < 0) {c1.x = 0;}
	if((x-5) >= height) {c1.y = height-1;}
	if((y-5) >= width) {c1.x = width-1;}

	c2 = cvPoint(y+5,x+5);
	if((x+5) < 0) {c2.y = 0;}
	if((y+5) < 0) {c2.x = 0;}
	if((x+5) >= height) {c2.y = height-1;}
	if((y+5) >= width) {c2.x = width-1;}

	cvRectangle(img,c1,c2,cvScalar(130),CV_FILLED);
}


void showParticles(IplImage* img)
{
	for(int i = 0; i < N; i++)
	{
		if((((Px[i]+sx/2) < height) & ((Py[i]+sy/2) < width)) & (((Px[i]+sx/2) >= 0) & ((Py[i]+sy/2) >= 0)))
		{
			img->imageData[((Px[i]+sx/2)*img->widthStep)+(Py[i]+sy/2)] = 130;
		}
	}
}


int** integralImage(IplImage* img)
{
	// Init
	int** integral = 0;
	integral = new int*[height];
	for(int i = 0; i < height; i++) {integral[i] = new int[width];};
	int cumSum = cvGet2D(img,0,0).val[0];
	integral[0][0] = cumSum;

	for(int i = 1; i < height; i++)
	{
		cumSum += cvGet2D(img,i,0).val[0];
		integral[i][0] = cumSum;
	}

	cumSum = cvGet2D(img,0,0).val[0];
	for(int j = 1; j < width; j++)
	{
		cumSum += cvGet2D(img,0,j).val[0];
		integral[0][j] = cumSum;
	}

	for(int i = 1; i < height; i++)
	{
		for(int j = 1; j < width; j++)
		{
			integral[i][j] = integral[i-1][j] + integral[i][j-1] - integral[i-1][j-1] + cvGet2D(img,i,j).val[0];
		}
	}

	return integral;
}


double evaluate(int x, int y, int** integral)
{
	Point p(x+sx-1,y+sy-1);

	if((x+sx-1) < 0) {p.x = 0;}
	if((y+sy-1) < 0) {p.y = 0;}
	if((x+sx-1) >= height) {p.x = height-1;}
	if((y+sy-1) >= width) {p.y = width-1;}

	if(x < 1) {x = 1;}
	if(y < 1) {y = 1;}
	if(x > height) {x = height;}
	if(y > width) {y = width;}

	int weight = integral[p.x][p.y] - integral[x-1][p.y] - integral[p.x][y-1] + integral[x-1][y-1];
	return weight;
}


void initParticles()
{
	for(int i = 0; i < N; i++)
	{
		Px[i] = rand() % height;
		Py[i] = rand() % width;
		W[i] = 1;
	}
}


void particleFilter(IplImage* img)
{
	// Diffusion
	double diffusion_x [N];
	double diffusion_y [N];

	for(int i = 0; i < N; i++)
	{
		diffusion_x[i] = sigma_diffusion*normalRandom();
		Px[i] += diffusion_x[i];
		diffusion_y[i] = sigma_diffusion*normalRandom();
		Py[i] += diffusion_y[i];
	}

	// Weighting
	double norm = 0;
	int** integral = integralImage(img);
	for(int i = 0; i < N; i++)
	{
		W[i] = evaluate(Px[i],Py[i],integral);
		norm += W[i];
	}
	for(int i = 0; i < N; i++)
	{
		W[i] = W[i]/norm;
	}

	// Resampling
	double cdf[N];
	cdf[0] = W[0];
	for(int i = 1; i < N; i++)
	{
		cdf[i] = cdf[i-1] + W[i];
	}

	double r = uniformRandom()/N;
	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			if(cdf[j] >= r)
			{
				Px[i] = Px[j];
				Py[i] = Py[j];
				break;
			}
		}
		W[i] = (double)1/N;
		r += (double)1/N;
	}

	// Update object coordinate
	robotCoordinate();
}


class ImageConverter
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;

public:
	IplImage* img;
	IplImage* hsv_image;
	IplImage* hsv_mask;

	ImageConverter()
	: it(nh)
	{
		image_sub = it.subscribe("/image_raw" + nao,1,&ImageConverter::imageConv,this);
	}

	~ImageConverter()
	{
	}

	void imageConv(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		img = new IplImage(cv_ptr->image);
		CvSize sz = cvGetSize(img);
		hsv_image = cvCreateImage(sz,8,3);
		hsv_mask = cvCreateImage(sz,8,1);
		cvCvtColor(img,hsv_image,CV_BGR2HSV);
		cvInRangeS(hsv_image,hsv_min,hsv_max, hsv_mask);

		// Init
		if((height != sz.height) | (width != sz.width))
		{
			height = sz.height;
			width = sz.width;

			initParticles();
			robotCoordinate();
		}

		// Filter
		particleFilter(hsv_mask);

		// Publish robot position
		PosImage pos;
		pos.x = x;
		pos.y = y;
		pos.height = height;
		pos.width = width;
		robotPosition_pub.publish(pos);

		// Draw particles and robot
		showParticles(hsv_mask);
		showRobot(hsv_mask);

		cvNamedWindow("hsv-msk",1); cvShowImage("hsv-msk",hsv_mask);

		cvWaitKey(10);
	}
};


ImageConverter* ic;

bool startService(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
{
	if(!init)
	{
		// Start image processing
		ic = new ImageConverter();
		init = true;
	}

	std_srvs::Empty srv;
	bool res = client.call(srv);
	if(res) {init = false; delete ic; cvDestroyAllWindows();}

	return res;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"nao_detection");
	ros::NodeHandle nh;

	// HSV parameters
	ros::NodeHandle pnh("~");
	pnh.param("H_MIN",H_MIN,int(0));
	pnh.param("H_MAX",H_MAX,int(0));
	pnh.param("S_MIN",S_MIN,int(0));
	pnh.param("S_MAX",S_MAX,int(0));
	pnh.param("V_MIN",V_MIN,int(0));
	pnh.param("V_MAX",V_MAX,int(0));

	hsv_min = cvScalar(H_MIN,S_MIN,V_MIN,0);
	hsv_max = cvScalar(H_MAX,S_MAX,V_MAX,0);

	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		// Publisher
		robotPosition_pub = nh.advertise<PosImage>("/image_nao_pos" + nao,100);

		// Service
		ros::ServiceServer service = nh.advertiseService("nao_tracker_camera" + nao,startService);

		// Client
		client = nh.serviceClient<std_srvs::Empty>("nao_controller_camera" + nao);

		ros::spin();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
