/*
 * particleFilter.cpp
 *
 *  Created on: Mar 22, 2014
 *      Author: Olivier BALLAND
 */

/* Description:
 *  Particle Filter for multiple target tracking.
 */

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "nao_behavior_tree/filters/particleFilter.hpp"


particleFilter::particleFilter(int N0, int T0, ESTIMATOR estimator0, Object* initObjects, int sx_min0, int sy_min0, int sx_max0, int sy_max0, int sigma_diffusion0, int s_diffusion0)
{
	N = N0;
	T = T0;
	estimator = estimator0;
	objects = initObjects;
	sx_min = sx_min0;
	sy_min = sy_min0;
	sx_max = sx_max0;
	sy_max = sy_max0;
	sigma_diffusion = sigma_diffusion0;
	s_diffusion = s_diffusion0;
}
	

double particleFilter::uniformRandom()
{
	return (double)(rand())/(double)(RAND_MAX);
}


double particleFilter::normalRandom()
{
	// Box-Muller transform
	double u1,u2;
	u1 = u2 = 0;
	while(u1 == 0) {u1 = uniformRandom();}
	while(u2 == 0) {u2 = uniformRandom();}
	return cos(2*M_PI*u2)*sqrt(-2.*log(u1));
}


void particleFilter::updateObject(int idObject)
{
	if(estimator == MMSE)
	{
		double x,y,sx,sy;
		x = y = sx = sy = 0;

		for(int i = 0; i < N; i++)
		{
			x += objects[idObject].particles[i].x + objects[idObject].particles[i].sx/2;
			y += objects[idObject].particles[i].y + objects[idObject].particles[i].sy/2;
			sx += objects[idObject].particles[i].sx;
			sy += objects[idObject].particles[i].sy;
		}
		objects[idObject].x = x/N;
		objects[idObject].y = y/N;
		objects[idObject].sx = sx/N;
		objects[idObject].sy = sy/N;
	}
	
	if(estimator == MAP)
	{
		double x,y,sx,sy; 
		double maxWeight;
		for(int i = 0; i < N; i++)
		{
			if(objects[idObject].particles[i].w > maxWeight)
			{
				x = objects[idObject].particles[i].x + objects[idObject].particles[i].sx/2;
				y = objects[idObject].particles[i].y + objects[idObject].particles[i].sy/2;
				sx = objects[idObject].particles[i].sx;
				sy = objects[idObject].particles[i].sy;
				maxWeight = objects[idObject].particles[i].w;
			}
		}
		objects[idObject].x = x;
		objects[idObject].y = y;
		objects[idObject].sx = sx;
		objects[idObject].sy = sy;
	}
}


IplImage* particleFilter::drawObject(IplImage* img, int idObject)
{
	IplImage* copy = cvCloneImage(img);
	CvPoint c1,c2;

	c1 = cvPoint(objects[idObject].y-objects[idObject].sy/2,objects[idObject].x-objects[idObject].sx/2);
	if((objects[idObject].x-objects[idObject].sx/2) < 0) {c1.y = 0;}
	if((objects[idObject].y-objects[idObject].sy/2) < 0) {c1.x = 0;}
	if((objects[idObject].x-objects[idObject].sx/2) >= height) {c1.y = height-1;}
	if((objects[idObject].y-objects[idObject].sy/2) >= width) {c1.x = width-1;}

	c2 = cvPoint(objects[idObject].y+objects[idObject].sy/2,objects[idObject].x+objects[idObject].sx/2);
	if((objects[idObject].x+objects[idObject].sx/2) < 0) {c2.y = 0;}
	if((objects[idObject].y+objects[idObject].sy/2) < 0) {c2.x = 0;}
	if((objects[idObject].x+objects[idObject].sx/2) >= height) {c2.y = height-1;}
	if((objects[idObject].y+objects[idObject].sy/2) >= width) {c2.x = width-1;}

	cvRectangle(copy,c1,c2,cvScalar(130),2);
	return copy;
}


IplImage* particleFilter::drawParticles(IplImage* img, int idObject)
{
	IplImage* copy = cvCloneImage(img);

	for(int i = 0; i < N; i++)
	{
		if((((objects[idObject].particles[i].x+objects[idObject].particles[i].sx/2) < height) & ((objects[idObject].particles[i].y+objects[idObject].particles[i].sy/2) < width)) & (((objects[idObject].particles[i].x+objects[idObject].particles[i].sx/2) >= 0) & ((objects[idObject].particles[i].y+objects[idObject].particles[i].sy/2) >= 0)))
		{
			copy->imageData[((objects[idObject].particles[i].x+objects[idObject].particles[i].sx/2)*img->widthStep)+(objects[idObject].particles[i].y+objects[idObject].particles[i].sy/2)] = 130;
		}
	}

	return copy;
}


Object particleFilter::getObject(int idObject)
{
	return objects[idObject];
}


std::pair<double,double> particleFilter::particlesStD(int idObject)
{
	std::pair<double,double> M; // Mean
	std::pair<double,double> V;	// Standard deviation

	for(int i = 0; i < N; i++)
	{
		M.first += objects[idObject].particles[i].x;
		M.second += objects[idObject].particles[i].y;

		V.first += objects[idObject].particles[i].x*objects[idObject].particles[i].x;
		V.second += objects[idObject].particles[i].y*objects[idObject].particles[i].y;
	}

	M.first = M.first/N;
	M.second = M.second/N;

	V.first = V.first/N - M.first*M.first;
	V.second = V.second/N - M.second*M.second;

	return V;
}


int particleFilter::getPixelValue(IplImage* img, int i, int j)
{
	if(cvGet2D(img,i,j).val[0] == 255)
	{
		return 1;
	}
	return -1;
}


int** particleFilter::integralImage(IplImage* img)
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


double particleFilter::evaluate(int x, int y, int sx, int sy, int** integral)
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

	double weight = integral[p.x][p.y] - integral[x-1][p.y] - integral[p.x][y-1] + integral[x-1][y-1];
	if(weight <= 0) {weight = eps;}
	return weight;
}


double particleFilter::evaluateMultiple(int x, int y, int sx, int sy, int** integral, int idObject)
{
	double weight;

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

	weight = integral[p.x][p.y] - integral[x-1][p.y] - integral[p.x][y-1] + integral[x-1][y-1];

	// Data association
	for(int t = 0; t < T; t++)
	{
		if(t != idObject)
		{
			// Distance between the particle and the other object
			double distanceOther = sqrt((x+sx/2-objects[t].x)*(x+sx/2-objects[t].x) + (y+sy/2-objects[t].y)*(y+sy/2-objects[t].y));
			if(distanceOther < sqrt(objects[t].sx*objects[t].sx + objects[t].sy*objects[t].sy)) {weight = 0; break;}
		}
	}
	
	// Distance between the particle and the object
	double distanceRobot_x = (x+sx/2-objects[idObject].x)*(x+sx/2-objects[idObject].x);
	double distanceRobot_y = (y+sy/2-objects[idObject].y)*(y+sy/2-objects[idObject].y);
	double sigma_x = objects[idObject].sx*objects[idObject].sx;
	double sigma_y = objects[idObject].sy*objects[idObject].sy;
	
	weight = weight*exp(-distanceRobot_x/sigma_x/2-distanceRobot_y/sigma_y/2);

	if(weight <= 0) {weight = eps;}
	return weight;
}


void particleFilter::initParticles()
{
	for(int t = 0; t < T; t++)
	{
		objects[t].particles = new Particle [N];

		for(int i = 0; i < N; i++)
		{
			// Position
			objects[t].particles[i].x = rand() % height;
			objects[t].particles[i].y = rand() % width;

			// Weight
			objects[t].particles[i].w = 1;

			// Size
			objects[t].particles[i].sx = sx_min;
			objects[t].particles[i].sy = sy_min;
			
			// Object size
			objects[t].sx = sx_min;
			objects[t].sy = sy_min;
		}
	}
}


void particleFilter::PF(int** integral, int idObject)
{
	// Diffusion
	for(int i = 0; i < N; i++)
	{
		// Position
		objects[idObject].particles[i].x += sigma_diffusion*normalRandom();
		if(objects[idObject].particles[i].x < 0) {objects[idObject].particles[i].x = 0;}
		if(objects[idObject].particles[i].x > height-1) {objects[idObject].particles[i].x = height-1;}

		objects[idObject].particles[i].y += sigma_diffusion*normalRandom();
		if(objects[idObject].particles[i].y < 0) {objects[idObject].particles[i].y = 0;}
		if(objects[idObject].particles[i].y > width-1) {objects[idObject].particles[i].y = width-1;}

		// Size
		objects[idObject].particles[i].sx += s_diffusion*normalRandom();
		if(objects[idObject].particles[i].sx < sx_min) {objects[idObject].particles[i].sx = sx_min;}
		if(objects[idObject].particles[i].sx > sx_max) {objects[idObject].particles[i].sx = sx_max;}

		objects[idObject].particles[i].sy += s_diffusion*normalRandom();
		if(objects[idObject].particles[i].sy < sy_min) {objects[idObject].particles[i].sy = sy_min;}
		if(objects[idObject].particles[i].sy > sy_max) {objects[idObject].particles[i].sy = sy_max;}
	}

	// Weighting
	double norm = 0;
	for(int i = 0; i < N; i++)
	{
		if(T > 1) {objects[idObject].particles[i].w = evaluateMultiple(objects[idObject].particles[i].x,objects[idObject].particles[i].y,objects[idObject].particles[i].sx,objects[idObject].particles[i].sy,integral,idObject);}
		else {objects[idObject].particles[i].w = evaluate(objects[idObject].particles[i].x,objects[idObject].particles[i].y,objects[idObject].particles[i].sx,objects[idObject].particles[i].sy,integral);}
		//ROS_INFO("weight %i = %f",i,objects[idObject].particles[i].w);
		norm += objects[idObject].particles[i].w;
	}
	for(int i = 0; i < N; i++)
	{
		objects[idObject].particles[i].w = objects[idObject].particles[i].w/norm;
	}
		
	// Resampling
	double cdf[N];
	cdf[0] = objects[idObject].particles[0].w;
	for(int i = 1; i < N; i++)
	{
		cdf[i] = cdf[i-1] + objects[idObject].particles[i].w;
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
				temp[i] = objects[idObject].particles[j];
				index = j;
				break;
			}
		}

		r += (double)1/N;
	}
	for(int i = 0; i < N; i++)
	{
		objects[idObject].particles[i] = temp[i];
	}
	
	// Update object
	updateObject(idObject);

	// Change weights
	for(int i = 1; i < N; i++)
	{
		objects[idObject].particles[i].w = (double)1/N;	
	}
}


void particleFilter::imageProcessing(IplImage* img)
{
	// Create temporary images
	CvSize sz = cvGetSize(img);

	// Init
	if((height != sz.height) | (width != sz.width))
	{
		height = sz.height;
		width = sz.width;

		initParticles();
	}

	// Compute the integral image
	int** integral = integralImage(img);

	// Particle Filter
	for(int i = 0; i < T; i++)
	{
		PF(integral,i);
	}
}

