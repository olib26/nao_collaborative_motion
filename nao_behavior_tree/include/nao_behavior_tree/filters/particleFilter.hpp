/*
 * particleFilter.hpp
 *
 *  Created on: Mar 22, 2014
 *      Author: Olivier BALLAND
 */


#ifndef PARTICLEFILTER_HPP
#define PARTICLEFILTER_HPP


// Minimum weight
const double eps = 1E-3;


// Estimators
enum ESTIMATOR {
	MAP = 0,
	MMSE,
};


// Particle attributes
struct Particle{
	int x,y;
	int sx,sy;
	double w;
};


// Object attributes
struct Object
{
	int x,y;
	int sx,sy;
	Particle* particles;
};


class particleFilter
{
private:
	// Image size
	int height,width;
	
	// Objects properties
	Object* objects;
	
	// Object size
	int sx_min,sy_min,sx_max,sy_max;
	
	// Diffusion standard deviation
	int sigma_diffusion,s_diffusion;
	
	// Number of targets
	int T;
	
	// Number of particles per target
	int N;
	
	// Estimator
	ESTIMATOR estimator;
	
	// Point in integral image
	struct Point
	{
		int x,y;
		Point(int x,int y) : x(x),y(y){}
	};
	
	// Distributions
	double uniformRandom();
	double normalRandom();
	
	// Image processing
	int getPixelValue(IplImage* img, int i, int j);
	int** integralImage(IplImage* img);
	double evaluate(int x, int y, int sx, int sy, int** integral);
	double evaluateMultiple(int x, int y, int sx, int sy, int** integral, int idObject);
	void PF(int** integral, int idObject);
	void updateObject(int idObject);
	
	
public:
	// Creator
	particleFilter(int N0, int T0, ESTIMATOR estimator, Object* initObjects, int sx_min0, int sy_min0, int sx_max0, int sy_max0, int sigma_diffusion0, int s_diffusion0);
	
	// Functions
	void imageProcessing(IplImage* img);
	IplImage* drawObject(IplImage* img, Object object);
	IplImage* drawParticles(IplImage* img, Object object);
	std::pair<double,double> particlesStD(Object object);
	void initParticles();
};


#endif /* PARTICLEFILTER_HPP */
