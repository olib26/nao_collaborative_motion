/*
 * obstacles.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Olivier Balland
 */

#ifndef OBSTACLES_HPP_
#define OBSTACLES_HPP_


// Modes
enum MODES {
	CREATION = 0,
	NORMAL,
};

// Camera used
int cameraId;
bool webcam;

// Image size
int height,width;

// Camera coefficient
double k;

// Points
struct Point
{
	double x,y;
};

// Obstacles
struct Obstacle
{
	std::vector<Point> pointsWorld;
	std::vector<cv::Point> pointsImage;
};
Obstacle currentObstacle;
std::vector<Obstacle> obstacles;
const int maxPoints = 100;

// Obstacle gap edges
struct Edge
{
	Point p;
	double theta;
};
typedef std::pair<Edge,Edge> obstacleEdges;
typedef std::vector<Edge> allEdges;

// Robot odometry
struct Robot
{
	Point pos,vel;
	double theta;
};
Robot r1,r2;

bool robot1Detected,robot2Detected;

// Velocity threshold
double velThreshold = 0.03;

// Optimal velocity publishers
ros::Publisher vel1_pub,vel2_pub;


#endif /* OBSTACLES_HPP_ */
