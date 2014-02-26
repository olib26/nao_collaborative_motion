/*
 * main.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Olivier BALLAND
 */

#include <ros/ros.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <naos_localization/PF.hpp>

using namespace std;


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


void robotCoordinate(Particle* particles, Robot* r)
{
	r->x = r->y = r->sx = r->sy = 0;
	for(int i = 0; i < N; i++)
	{
		r->x += particles[i].x + particles[i].sx/2;
		r->y += particles[i].y + particles[i].sy/2;
		r->sx += particles[i].sx;
		r->sy += particles[i].sy;
	}
	r->x = r->x/N;
	r->y = r->y/N;
	r->sx = r->sx/N;
	r->sy = r->sy/N;
}


void showRobot(IplImage* img, Robot r)
{
	CvPoint c1,c2;

	c1 = cvPoint(r.y-r.sy/2,r.x-r.sx/2);
	if((r.x-r.sx/2) < 0) {c1.y = 0;}
	if((r.y-r.sy/2) < 0) {c1.x = 0;}
	if((r.x-r.sx/2) >= height) {c1.y = height-1;}
	if((r.y-r.sy/2) >= width) {c1.x = width-1;}

	c2 = cvPoint(r.y+r.sy/2,r.x+r.sx/2);
	if((r.x+r.sx/2) < 0) {c2.y = 0;}
	if((r.y+r.sy/2) < 0) {c2.x = 0;}
	if((r.x+r.sx/2) >= height) {c2.y = height-1;}
	if((r.y+r.sy/2) >= width) {c2.x = width-1;}

	cvRectangle(img,c1,c2,cvScalar(130),2);
}


void showParticles(IplImage* img, Particle* particles)
{
	for(int i = 0; i < N; i++)
	{
		if((((particles[i].x+particles[i].sx/2) < height) & ((particles[i].y+particles[i].sy/2) < width)) & (((particles[i].x+particles[i].sx/2) >= 0) & ((particles[i].y+particles[i].sy/2) >= 0)))
		{
			img->imageData[((particles[i].x+particles[i].sx/2)*img->widthStep)+(particles[i].y+particles[i].sy/2)] = 130;
		}
	}
}


Odometry computeOdometry(Robot r)
{
	Odometry odom;
	return odom;
}


std::pair<double,double> particlesVariance(Particle* particles)
{
	std::pair<double,double> M; // Mean
	std::pair<double,double> V;	// Variance

	for(int i = 0; i < N; i++)
	{
		M.first += particles[i].x;
		M.second += particles[i].y;

		V.first += particles[i].x*particles[i].x;
		V.second += particles[i].y*particles[i].y;
	}

	M.first = M.first/N;
	M.second = M.second/N;

	V.first = V.first/N - M.first*M.first;
	V.second = V.second/N - M.second*M.second;

	return V;
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


double evaluate(int x, int y, int sx, int sy, int** integral, Robot* other)
{
	// Distance between the particle and the other robot
	double distance = sqrt((x-other->x)*(x-other->x) + (y-other->y)*(y-other->y));
	if(distance < distanceThreshold) {return eps;}

	// Number of pixels
	CvPoint p = cvPoint(x+sx-1,y+sy-1);

	if((x+sx-1) < 0) {p.x = 0;}
	if((y+sy-1) < 0) {p.y = 0;}
	if((x+sx-1) >= height) {p.x = height-1;}
	if((y+sy-1) >= width) {p.y = width-1;}

	if(x < 1) {x = 1;}
	if(y < 1) {y = 1;}
	if(x > height) {x = height;}
	if(y > width) {y = width;}

	int weight = integral[p.x][p.y] - integral[x-1][p.y] - integral[p.x][y-1] + integral[x-1][y-1];
	if(weight <= 0) {weight = eps;}
	return weight;
}


void initParticles(Particle* particles)
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
		particles[i].w = evaluate(particles[i].x,particles[i].y,particles[i].sx,particles[i].sy,integral,other);
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


void imageProcessing(IplImage* img)
{
	// Create temporary images
	CvSize sz = cvGetSize(img);
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	cvInRangeS(hsv_image,hsv_min,hsv_max,hsv_mask);

	// Init
	if((height != sz.height) | (width != sz.width))
	{
		height = sz.height;
		width = sz.width;

		initParticles(particles1);
		initParticles(particles2);
	}

	// Filter
	particleFilter(hsv_mask,particles1,&r1,&r2);
	particleFilter(hsv_mask,particles2,&r2,&r1);

	// Draw particles
	showParticles(hsv_mask,particles1);
	showParticles(hsv_mask,particles2);

	// Draw robots
	showRobot(hsv_mask,r1);
	showRobot(hsv_mask,r2);

	// Show result
	cvShowImage("Camera_Output",hsv_mask);
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


int main(int argc, char** argv)
{
	ros::init(argc, argv,"naos_localization");
	ros::NodeHandle nh;

	ros::NodeHandle pnh("~");
	// Camera id
	pnh.param("camera",camera,int(0));
	// HSV parameters
	pnh.param("H_MIN",H_MIN,int(0));
	pnh.param("H_MAX",H_MAX,int(0));
	pnh.param("S_MIN",S_MIN,int(0));
	pnh.param("S_MAX",S_MAX,int(0));
	pnh.param("V_MIN",V_MIN,int(0));
	pnh.param("V_MAX",V_MAX,int(0));
	hsv_min = cvScalar(H_MIN,S_MIN,V_MIN,0);
	hsv_max = cvScalar(H_MAX,S_MAX,V_MAX,0);

	// Capture from camera
	IplImage* img;
	cvNamedWindow("Camera_Output",1);
	CvCapture* capture = cvCaptureFromCAM(0);

	// Init robots positions
	cv::Point p;
	cv::setMouseCallback("Camera_Output",on_mouse,&p);

	while(((r1.x == 0) | (r2.x == 0)) & ros::ok())
	{
		img = cvQueryFrame(capture); // Capture one image
		cvShowImage("Camera_Output",img);

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

		cvWaitKey(100);
	}

	while(ros::ok()){
		img = cvQueryFrame(capture); // Capture one image

		// Image processing
		imageProcessing(img);

		cvWaitKey(100);
	}

	cvReleaseCapture(&capture); // Release capture.
	cvDestroyWindow("Camera_Output"); // Destroy Window

	return 0;
}
