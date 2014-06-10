/*
 * nao_detection.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: Olivier BALLAND
 */

/* Description:
 *  Receives the image from the camera’s proxy.
 *  Publishes the robot’s position in the image.
 */

/* Topics used:
 *  /image_raw
 *  /image_nao_pos
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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <std_srvs/Empty.h>

#include <first_approaches/PosImage.h>
#include "first_approaches/nao_detection.hpp"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace first_approaches;


double uniformRandom()
{
	return (double)(rand())/(double)(RAND_MAX);
}


double normalRandom()
{
	// Box-Muller transform
	double u1,u2;
	u1 = u2 = 0;
	while(u1 == 0) {u1 = uniformRandom();}
	while(u2 == 0) {u2 = uniformRandom();}
	return cos(2*M_PI*u2)*sqrt(-2.*log(u1));
}


void robotCoordinate()
{
	x = y = sx = sy = 0;
	double maxWeight = 0;
	for(int i = 0; i < N; i++)
	{
		if(particles[i].w > maxWeight)
		{
			x = particles[i].x + particles[i].sx/2;
			y = particles[i].y + particles[i].sy/2;
			sx = particles[i].sx;
			sy = particles[i].sy;
			maxWeight = particles[i].w;
		}
	}

	// Derive distance between the two robots
	// Two interesting points
	float p_u1 = -(x + (sx-height)/2);
	float p_u2 = -(x - (sx+height)/2);
	float p_v = -(y - width/2);
	float f = (float)width/2/tan(HFOV/2.);

	// Vectors
	Eigen::Vector3f v1(f,p_v,p_u1);
	Eigen::Vector3f v2(f,p_v,p_u2);

	// Normalization
	v1 = v1/v1.norm();
	v2 = v2/v2.norm();

	// Center
	Eigen::Vector3f c = (v1+v2)/2.;
	float c_norm = c.norm();

	// Projection
	Eigen::MatrixXf proj_mat = c.transpose()*v1;
	float proj = proj_mat(0,0);

	// Orthogonal part in v1
	Eigen::Vector3f orth = v1 - proj/c_norm*c;

	// Norm
	float orth_norm = orth.norm();

	// Approximate depth
	depth = H/2.*proj/orth_norm;
}


void showRobot(IplImage* img)
{
	CvPoint c1,c2;

	c1 = cvPoint(y-sy/2,x-sx/2);
	if((x-sx/2) < 0) {c1.y = 0;}
	if((y-sy/2) < 0) {c1.x = 0;}
	if((x-sx/2) >= height) {c1.y = height-1;}
	if((y-sy/2) >= width) {c1.x = width-1;}

	c2 = cvPoint(y+sy/2,x+sx/2);
	if((x+sx/2) < 0) {c2.y = 0;}
	if((y+sy/2) < 0) {c2.x = 0;}
	if((x+sx/2) >= height) {c2.y = height-1;}
	if((y+sy/2) >= width) {c2.x = width-1;}

	cvRectangle(img,c1,c2,cvScalar(130),2);
}


void showParticles(IplImage* img)
{
	for(int i = 0; i < N; i++)
	{
		if((((particles[i].x+particles[i].sx/2) < height) & ((particles[i].y+particles[i].sy/2) < width)) & (((particles[i].x+particles[i].sx/2) >= 0) & ((particles[i].y+particles[i].sy/2) >= 0)))
		{
			img->imageData[((particles[i].x+particles[i].sx/2)*img->widthStep)+(particles[i].y+particles[i].sy/2)] = 130;
		}
	}
}


int getPixelValue(IplImage* img,int i,int j)
{
	if(cvGet2D(img,i,j).val[0] == 255)
	{
		return 1;
	}
	return -1;
}


int** integralImage(IplImage* img)
{
	// Init integral image
	int** integral = 0;
	integral = new int*[height];
	for(int i = 0; i < height; i++) {integral[i] = new int[width];};
	int cumSum = getPixelValue(img,0,0);
	integral[0][0] = cumSum;

	for(int i = 1; i < height; i++)
	{
		cumSum += getPixelValue(img,i,0);
		integral[i][0] = cumSum;
	}

	cumSum = getPixelValue(img,0,0);
	for(int j = 1; j < width; j++)
	{
		cumSum += getPixelValue(img,0,j);
		integral[0][j] = cumSum;
	}

	for(int i = 1; i < height; i++)
	{
		for(int j = 1; j < width; j++)
		{
			integral[i][j] = integral[i-1][j] + integral[i][j-1] - integral[i-1][j-1] + getPixelValue(img,i,j);
		}
	}

	return integral;
}


double evaluate(int x, int y, int sx, int sy, int** integral)
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
	if(weight <= 0) {weight = 1E-5;}
	return weight;
}


void initParticles()
{
	for(int i = 0; i < N; i++)
	{
		// Position
		particles[i].x = rand() % height;
		particles[i].y = rand() % width;
		
		// Weight
		particles[i].w = 1;
		
		// Size
		particles[i].sx = sx_min;
		particles[i].sy = sy_min;
	}
}


void particleFilter(IplImage* img)
{
	// Diffusion
	double diffusion_x [N];
	double diffusion_y [N];
	double diffusion_sx [N];
	double diffusion_sy [N];

	for(int i = 0; i < N; i++)
	{
		// Position
		diffusion_x[i] = sigma_diffusion*normalRandom();
		particles[i].x += diffusion_x[i];
		diffusion_y[i] = sigma_diffusion*normalRandom();
		particles[i].y += diffusion_y[i];
		
		// Size
		diffusion_sx[i] = s_diffusion*normalRandom();
		particles[i].sx += diffusion_sx[i];
		if(particles[i].sx < sx_min) {particles[i].sx = sx_min;}
		diffusion_sy[i] = s_diffusion*normalRandom();
		particles[i].sy += diffusion_sy[i];
		if(particles[i].sy < sy_min) {particles[i].sy = sy_min;}
	}

	// Weighting
	double norm = 0;
	int** integral = integralImage(img);
	for(int i = 0; i < N; i++)
	{
		particles[i].w = evaluate(particles[i].x,particles[i].y,particles[i].sx,particles[i].sy,integral);
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
	int index = 0;
	Particle* temp = new Particle [N];
	for(int i = 0; i < N; i++)
	{
		for(int j = index; j < N; j++)
		{
			if(cdf[j] >= r)
			{
				temp[i] = particles[j];
				index = j;
				break;
			}
		}
		r += (double)1/N;
	}

	for(int i = 0; i < N; i++)
	{
		particles[i] = temp[i];
	}

	// Update object coordinate
	robotCoordinate();

	for(int i = 0; i < N; i++)
	{
		particles[i].w = (double)1/N;
	}
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
		pos.depth = depth;
		robotPosition_pub.publish(pos);

		// Draw particles and robot
		//showParticles(hsv_mask);
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
