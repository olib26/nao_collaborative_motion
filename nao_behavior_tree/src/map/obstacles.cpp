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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <nao_behavior_tree/Odometry.h>
#include <nao_behavior_tree/Bearing.h>

#include <nao_behavior_tree/map/obstacles.hpp>
#include <nao_behavior_tree/map/webcam.hpp>
#include <nao_behavior_tree/map/nao.hpp>


void drawObstacle(IplImage* img, Obstacle obstacle)
{
	cv::Point p1,p2;
	CvScalar color = cvScalar(0,0,255);
	int thickness = 2;

	std::vector<cv::Point> points = obstacle.pointsImage;

	for(unsigned  int j = 0; j < points.size()-1; j++)
	{
		p1 = points.at(j);
		p2 = points.at(j+1);
		cvLine(img,p1,p2,color,thickness,CV_AA,0);
	}
	p1 = points.back();
	p2 = points.at(0);
	cvLine(img,p1,p2,color,thickness,CV_AA,0);
}


void drawCurrentObstacle(IplImage* img)
{
	cv::Point p1,p2;
	CvScalar color = cvScalar(0,0,255);
	int thickness = 2;

	std::vector<cv::Point> points = currentObstacle.pointsImage;
	for(int j = 0; j < (int)(points.size()-1); j++)
	{
		p1 = points.at(j);
		p2 = points.at(j+1);
		cvLine(img,p1,p2,color,thickness,CV_AA,0);
	}
}


void showObstacles(IplImage* img)
{
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		drawObstacle(img,obstacles.at(i));
	}

	drawCurrentObstacle(img);

	cvShowImage("Camera_Output",img);
}


void receive_odometry1(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r1.pos.x = msg->x;
	r1.pos.y = msg->y;
	r1.vel.x = msg->vx;
	r1.vel.y = msg->vy;
}


void receive_odometry2(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r2.pos.x = msg->x;
	r2.pos.y = msg->y;
	r2.vel.x = msg->vx;
	r2.vel.y = msg->vy;
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
	p.x = (pixel.x-width/2)*k;
	p.y = -(pixel.y-height/2)*k;
	return p;
}


cv::Point pointToPixel(Point p)
{
	cv::Point point;
	point.x = p.x/k + width/2;
	point.y = -p.y/k + height/2;
	return point;
}


double modulo2Pi(double theta)
{
	// Angle between ]-pi,pi]
	while(theta > M_PI) {theta -= 2*M_PI;}
	while(theta <= -M_PI) {theta += 2*M_PI;}
	return theta;
}


double angle(Point p1, Point p2)
{
	double theta;

	if(p1.x == p2.x)
	{
		if((p2.y-p1.y) < 0) {theta = -M_PI/2;}
		if((p2.y-p1.y) > 0) {theta = M_PI/2;}
	}
	else
	{
		theta = atan((p2.y-p1.y)/(p2.x-p1.x));
		if((p2.x-p1.x) < 0) {theta += M_PI;}
	}

	modulo2Pi(theta);

	return theta;
}


obstacleEdges computeEdges(Obstacle obstacle, Robot r)
{
	obstacleEdges edges;

	// Angles
	double minAngle,maxAngle;
	double deltaAngle,currentAngle;

	Point pMin,pMax;

	Point p1 = obstacle.pointsWorld.back();

	minAngle = maxAngle = currentAngle = angle(r.pos,p1);
	pMin = pMax = p1;

	for(unsigned int i = 0; i < obstacle.pointsWorld.size(); i++)
	{
		Point p2 = obstacle.pointsWorld.at(i);

		// Compute angle between 2 points
		deltaAngle = modulo2Pi(angle(r.pos,p2)-angle(r.pos,p1));

		// Current angle
		currentAngle += deltaAngle;

		// Update min and max
		if(minAngle > currentAngle) {minAngle = currentAngle; pMin = p2;}
		if(maxAngle < currentAngle) {maxAngle = currentAngle; pMax = p2;}

		// Update point
		p1 = p2;
	}

	// Edges coefs
	edges.first.p = pMin;
	edges.first.theta = modulo2Pi(minAngle);

	edges.second.p = pMax;
	edges.second.theta = modulo2Pi(maxAngle);

	return edges;
}


allEdges computeAllEdges(std::vector<Obstacle> obstacles, Robot r)
{
	allEdges edges;

	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		obstacleEdges pair = computeEdges(obstacles.at(i),r);
		edges.push_back(pair.first);
		edges.push_back(pair.second);
	}

	return edges;
}


bool intersectEdge(Robot r, Edge edge)
{
	Point p0;
	double V = angle(p0,r.vel);
	double alpha = angle(r.pos,edge.p);

	if((edge.theta-alpha) >= 0) {return (((V-alpha) >= 0) & ((V-alpha) <= (edge.theta-alpha)));}
	else {return (((V-alpha) <= 0) & ((V-alpha) >= (edge.theta-alpha)));}
}


double distanceToEdge(Point p, Edge edge)
{
	Eigen::Vector2d vec(p.x-edge.p.x,p.y-edge.p.y);
	Eigen::Vector2d d(cos(edge.theta),sin(edge.theta));
	Eigen::MatrixXd proj = vec.transpose()*d;

	if(proj(0,0) > 0)
	{
		Eigen::Vector2d orth = vec - proj(0,0)*d;
		return orth.norm();
	}
	else
	{
		return vec.norm();
	}
}


Edge closestEdge(Robot r, allEdges edges)
{
	Edge closest;
	double dist;
	double minDist = INFINITY;

	for(unsigned int i = 0; i < edges.size(); i++)
	{
		Edge edge = edges.at(i);
		dist = distanceToEdge(r.pos,edge);

		if(dist < minDist) {closest = edge;}
	}

	return closest;
}


void showEdge(Edge edge, IplImage* img)
{
	cv::Point p1,p2;
	CvScalar color = cvScalar(0,0,0);
	int thickness = 1;

	p1 = pointToPixel(edge.p);

	Point p;
	double length = 10;
	p.x = edge.p.x + length*cos(edge.theta);
	p.y = edge.p.y + length*sin(edge.theta);
	p2 = pointToPixel(p);

	cvLine(img,p1,p2,color,thickness,CV_AA,0);
}


void showAllEdges(std::vector<Edge> allEdges, IplImage* img)
{
	Edge edge;
	for(unsigned int i = 0; i < allEdges.size(); i++)
	{
		edge = allEdges.at(i);
		showEdge(edge,img);
	}
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
		cv::Point pixel(x,y);
		currentObstacle.pointsImage.push_back(pixel);
		
		Point p = pixelToPoint(pixel);
		currentObstacle.pointsWorld.push_back(p);

		ROS_INFO("Pixel added: x = %i, y = %i",y,x);
	}
	
	if(event == cv::EVENT_RBUTTONDOWN)
	{	
		if(!currentObstacle.pointsImage.empty())
		{
			obstacles.push_back(currentObstacle);

			ROS_INFO("Create obstacle %i",obstacles.size());

			// New obstacle
			currentObstacle.pointsWorld.clear();
			currentObstacle.pointsImage.clear();
		}
	}
}


void creation(IplImage* img)
{
	cvShowImage("Camera_Output",img);

	// Init mouse callback
	cv::Point p;
	cv::setMouseCallback("Camera_Output",on_mouse,&p);
	
	char key;
	while(ros::ok())
	{
		showObstacles(img);
		key = cvWaitKey(100);
		if(key == 27) {break;} // Esc key
	}
	
	// Save obstacles
	// World
	std::vector<Point> saveWorld;
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		Obstacle obstacle = obstacles.at(i);
		for(unsigned int j = 0; j < obstacle.pointsWorld.size(); j++)
		{
			saveWorld.push_back(obstacle.pointsWorld.at(j));
		}

		Point space;
		space.x = 0;
		space.y = 0;
		saveWorld.push_back(space);
	}

	saveWorld.resize(maxPoints);
	std::ofstream os_world("/home/olivier/ros_workspace/map/world.dat",std::ios::binary);
	os_world.write(reinterpret_cast<const char*>(&(saveWorld[0])),saveWorld.size()*sizeof(Point));
	os_world.close();


	// Image
	std::vector<cv::Point> saveImage;
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		Obstacle obstacle = obstacles.at(i);
		for(unsigned int j = 0; j < obstacle.pointsImage.size(); j++)
		{
			saveImage.push_back(obstacle.pointsImage.at(j));
		}

		cv::Point space;
		space.x = -1;
		space.y = -1;
		saveImage.push_back(space);
	}

	saveImage.resize(maxPoints);
	std::ofstream os_image("/home/olivier/ros_workspace/map/image.dat",std::ios::binary);
	os_image.write(reinterpret_cast<const char*>(&(saveImage[0])),saveImage.size()*sizeof(cv::Point));
	os_image.close();
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
		// Image
		std::vector<cv::Point> saveImage;
		saveImage.resize(maxPoints);
		std::ifstream is_image("/home/olivier/ros_workspace/map/image.dat",std::ios::binary);
		is_image.read(reinterpret_cast<char*>(&(saveImage[0])),saveImage.size()*sizeof(cv::Point));
		is_image.close();

		// World
		std::vector<Point> saveWorld;
		saveWorld.resize(maxPoints);
		std::ifstream is_world("/home/olivier/ros_workspace/map/world.dat",std::ios::binary);
		is_world.read(reinterpret_cast<char*>(&(saveWorld[0])),saveWorld.size()*sizeof(Point));
		is_world.close();

		for(int i = 0; i < maxPoints; i++)
		{
			if(saveImage.at(i).x == 0)
			{
				break;
			}

			if(saveImage.at(i).x != -1)
			{
				currentObstacle.pointsImage.push_back(saveImage.at(i));
				currentObstacle.pointsWorld.push_back(saveWorld.at(i));
			}
			else
			{
				obstacles.push_back(currentObstacle);
				currentObstacle.pointsImage.clear();
				currentObstacle.pointsWorld.clear();
			}
		}


		// Test
		while(ros::ok())
		{
			showObstacles(img);
			allEdges edges = computeAllEdges(obstacles,r1);
			showAllEdges(edges,img);
			ros::spinOnce();
			cvWaitKey(100);
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
