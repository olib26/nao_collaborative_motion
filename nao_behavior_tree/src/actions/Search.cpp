#include "nao_behavior_tree/rosaction.h"

#include <alproxies/almotionproxy.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "nao_behavior_tree/actions/Search.hpp"

namespace enc = sensor_msgs::image_encodings;


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


std::pair<double,double> particlesVariance()
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
	return cvGet2D(img,i,j).val[0];
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
	if(weight <= 0) {weight = eps;}
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

	for(int i = 0; i < N; i++)
	{
		// Position
		diffusion_x[i] = sigma_diffusion*normalRandom();
		particles[i].x += diffusion_x[i];
		diffusion_y[i] = sigma_diffusion*normalRandom();
		particles[i].y += diffusion_y[i];
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
}


void imageProcessing(IplImage* img)
{
	// Create temporary images
	CvSize sz = cvGetSize(img);
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	cvInRangeS(hsv_image,hsv_min,hsv_max, hsv_mask);

	// Init
	if((height != sz.height) | (width != sz.width))
	{
		height = sz.height;
		width = sz.width;

		initParticles();
	}

	// Filter
	particleFilter(hsv_mask);

	// Compute variance
	std::pair<double,double> V = particlesVariance();
	//ROS_INFO("Variances:  Vx = %f, Vy = %f",V.first,V.second);
	if((V.first < Var_min) & (V.second < Var_min))
	{
		robotDetected = true;
	}

	// Draw particles
	showParticles(hsv_mask);

	// Show result
	cvNamedWindow("Search",1); cvShowImage("Search",hsv_mask);

	cvWaitKey(10);
}


class ImageConverter
{
	IplImage* img;

	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;

public:
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
		imageProcessing(img);
	}
};


class Search : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	ImageConverter* ic;
	AL::ALMotionProxy* motion_proxy_ptr;

	Search(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	}

	~Search()
	{
		delete motion_proxy_ptr;
		delete ic;
	}

	void initialize()
	{
		sleep(1.0);

		// Enable stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		// Foot Contact Protection
		motion_proxy_ptr->setMotionConfig(AL::ALValue::array(AL::ALValue::array("ENABLE_FOOT_CONTACT_PROTECTION",true)));

		// Init moving
		motion_proxy_ptr->moveInit();

		// Robot not detected
		robotDetected = false;

		init_ = true;
	}

	void finalize()
	{
		// Stop rotating
		motion_proxy_ptr->stopMove();

		// Delete Filter
		delete ic;

		init_ = false;
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**Search -%- Executing Main Task, elapsed_time: "
				<< dt.toSec() << std::endl;
		std::cout << "**Search -%- execute_time: "
				<< execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			set_feedback(RUNNING);
			initialize();

			// Launch Particle Filter
			ic = new ImageConverter();

			// Start rotating
			motion_proxy_ptr->setWalkTargetVelocity(0,0,1,0.5);
		}

		if(robotDetected)
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
	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		ros::init(argc, argv,"Search" + nao);

		ros::NodeHandle pnh("~");
		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));
		// HSV parameters
		pnh.param("H_MIN",H_MIN,int(0));
		pnh.param("H_MAX",H_MAX,int(0));
		pnh.param("S_MIN",S_MIN,int(0));
		pnh.param("S_MAX",S_MAX,int(0));
		pnh.param("V_MIN",V_MIN,int(0));
		pnh.param("V_MAX",V_MAX,int(0));
		hsv_min = cvScalar(H_MIN,S_MIN,V_MIN,0);
		hsv_max = cvScalar(H_MAX,S_MAX,V_MAX,0);

		// Launch Server
		Search server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();

		cvDestroyAllWindows();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
