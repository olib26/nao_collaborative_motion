/*
 * localization.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Olivier BALLAND
 */

#include <ros/ros.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

#include <nao_behavior_tree/Odometry.h>
#include <nao_behavior_tree/Bearing.h>

#include <nao_behavior_tree/map/PF.hpp>
#include <nao_behavior_tree/map/webcam.hpp>
#include <nao_behavior_tree/map/nao.hpp>

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
	// Save previous values
	r->x_temp = r->x;
	r->y_temp = r->y;

	r->x = r->y = r->sx = r->sy = 0;
	for(int i = 0; i < N; i++)
	{
		r->x += particles[i].x + particles[i].sx/2;
		r->y += particles[i].y + particles[i].sy/2;
		r->sx += particles[i].sx;
		r->sy += particles[i].sy;
	}

	// Update
	r->x = r->x/N;
	r->y = r->y/N;
	r->sx = r->sx/N;
	r->sy = r->sy/N;

	// Init temp position
	if(!r->initTempPosition)
	{
		r->x_temp = r->x;
		r->y_temp = r->y;
		r->initTempPosition = true;
	}
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


void showOdometry(IplImage* img, nao_behavior_tree::Odometry odom, Robot r, double k)
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


void showBearings(IplImage* img, nao_behavior_tree::Odometry odom, Robot r, double k)
{
	CvPoint p1,p2,p3;
	int thickness = 2;
	double length = 1;

	p1.x = r.y;
	p1.y = r.x;

	double relative = r.relativeBearing;
	double absolute = r.absoluteBearing;

	ROS_INFO("Relative bearing = %f",relative);
	ROS_INFO("Absolute bearing = %f",absolute);

	// Relative
	p2.x = (odom.x + length*cos(relative+absolute))/k + width/2;
	p2.y = -(odom.y + length*sin(relative+absolute))/k + height/2;
	CvScalar color1 = cvScalar(0,255,0);
	cvLine(img,p1,p2,color1,thickness,CV_AA,0);

	// Absolute
	p3.x = (odom.x + length*cos(absolute))/k + width/2;
	p3.y = -(odom.y + length*sin(absolute))/k + height/2;
	CvScalar color2 = cvScalar(255,0,0);
	cvLine(img,p1,p3,color2,thickness,CV_AA,0);
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


double evaluate(int x, int y, int sx, int sy, int** integral, Robot* robot, Robot* other)
{
	// Distance between the particle and the other robot
	double distanceOther = sqrt((x+sx/2-other->x)*(x+sx/2-other->x) + (y+sy/2-other->y)*(y+sy/2-other->y));
	if(distanceOther < sqrt(other->sx*other->sx + other->sy*other->sy)) {return eps;}

	// Distance between the particle and the robot
	double distanceRobot = (x+sx/2-robot->x)*(x+sx/2-robot->x) + (y+sy/2-robot->y)*(y+sy/2-robot->y);
	double sigma = robot->sx*robot->sx + robot->sy*robot->sy;

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

	// Data association
	weight = weight*exp(-distanceRobot/sigma/2);

	if(weight <= 0) {weight = eps;}
	return weight;
}


void initParticles(Particle* particles, Robot r)
{
	for(int i = 0; i < N; i++)
	{
		// Position
		particles[i].x = r.x + sigma_diffusion*normalRandom();
		particles[i].y = r.y + sigma_diffusion*normalRandom();

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


void imageProcessing(IplImage* img)
{
	// Create temporary images
	CvSize sz = cvGetSize(img);
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	cvInRangeS(hsv_image,hsv_min,hsv_max,hsv_mask);

	// Filter
	particleFilter(hsv_mask,particles1,&r1,&r2);
	particleFilter(hsv_mask,particles2,&r2,&r1);

	// Draw particles
	//showParticles(hsv_mask,particles1);
	//showParticles(hsv_mask,particles2);

	// Draw robots
	showRobot(hsv_mask,r1);
	showRobot(hsv_mask,r2);

	// Show result
	cvShowImage("Camera_Output",hsv_mask);
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


int main(int argc, char** argv)
{
	ros::init(argc, argv,"localization");
	ros::NodeHandle nh;

	// Odometry publishers
	ros::Publisher odom1_pub = nh.advertise<nao_behavior_tree::Odometry>("/odometry1",1);
	ros::Publisher odom2_pub = nh.advertise<nao_behavior_tree::Odometry>("/odometry2",1);

	// Bearing subscribers
	ros::Subscriber bearing1_sub = nh.subscribe("/bearing1",1,receive_bearing1);
	ros::Subscriber bearing2_sub = nh.subscribe("/bearing2",1,receive_bearing2);

	// Camera selection
	bool webcam;

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

	// Window
	cvNamedWindow("Camera_Output",1);
	cvNamedWindow("Odometry",1);

	// Image
	IplImage* img;

	sleep(1); // Wait for proxy init

	// Init robots positions
	cv::Point p;
	cv::setMouseCallback("Camera_Output",on_mouse,&p);

	while(((r1.x == 0) | (r2.x == 0)) & ros::ok())
	{
		img = getImage(webcam);
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

	// Init timers
	timestamp1 = timestamp2 = ros::Time::now().toSec();

	while(ros::ok()){
		img = getImage(webcam);

		CvSize sz = cvGetSize(img);
		// Init
		if((height != sz.height) | (width != sz.width))
		{
			height = sz.height;
			width = sz.width;

			k = cameraCoef(webcam);

			initParticles(particles1,r1);
			initParticles(particles2,r2);
		}

		// Image processing
		imageProcessing(img);

		// Publish odometry
		computeOdometry(&r1,&odom1,&timestamp1,&counter1,k);
		computeOdometry(&r2,&odom2,&timestamp2,&counter2,k);

		odom1_pub.publish(odom1);
		odom2_pub.publish(odom2);

		//ROS_INFO("x1 = %f; y1 = %f",odom1.x,odom1.y);
		//ROS_INFO("x2 = %f; y2 = %f",odom2.x,odom2.y);

		// Show results
		showOdometry(img,odom1,r1,k);
		showOdometry(img,odom2,r2,k);
		if(robot2Detected) {showBearings(img,odom1,r1,k);}
		if(robot1Detected) {showBearings(img,odom2,r2,k);}
		cvShowImage("Odometry",img);

		cvWaitKey(100);
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
	cvDestroyWindow("Camera_Output"); // Destroy Window
	cvDestroyWindow("Odometry");

	return 0;
}
