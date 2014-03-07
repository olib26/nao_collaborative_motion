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
const int maxObstacles = 100;

// Robots
struct Robot
{
	double x,y,theta;
	double vx,vy;
};
Robot r1,r2;

bool robot1Detected,robot2Detected;


#endif /* OBSTACLES_HPP_ */
