#include "nao_behavior_tree/rosaction.h"

#include <alproxies/almotionproxy.h>
#include <nao_behavior_tree/Sonar.h>

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

#include <geometry_msgs/Twist.h>
#include <nao_behavior_tree/Odometry.h>

#include "nao_behavior_tree/actions/TrackingAmongObstacles.hpp"

namespace enc = sensor_msgs::image_encodings;


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


float robotDepth()
{
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
	float d = H/2.*proj/orth_norm;
	return d;
}


void robotCoordinate()
{
	x = y = sx = sy = 0;
	for(int i = 0; i < N; i++)
	{
		x += particles[i].x + particles[i].sx/2;
		y += particles[i].y + particles[i].sy/2;
		sx += particles[i].sx;
		sy += particles[i].sy;
	}
	x = x/N;
	y = y/N;
	sx = sx/N;
	sy = sy/N;

	// Estimate depth
	depth = robotDepth();
	//ROS_INFO("Depth = %f",depth);
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

	// Update object coordinate
	robotCoordinate();
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
	if((V.first > Var_max) & (V.second > Var_max))
	{
		robotDetected = false;
	}

	// Draw robot
	showRobot(hsv_mask);

	// Show result
	//cvNamedWindow("GoClose",1); cvShowImage("GoClose",hsv_mask);

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
		image_sub = it.subscribe("/image_raw" + r1.id,1,&ImageConverter::imageConv,this);
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


float computeDistance()
{
	return sqrt((r1.x-r2.x)*(r1.x-r2.x) + (r1.y-r2.y)*(r1.y-r2.y));
}


double relativeBearing()
{
	// Other robot parameters
	float p_x = -(y - width/2);
	float p_y = -(x - height/2);
	float f = (float)width/2/tan(HFOV/2.);
	float d = computeDistance();

	// Vector pointing to the other robot
	Eigen::Vector3f v(f,p_x,p_y);
	v.normalize();
	v = d*v;

	// Transformation to FRAME_ROBOT
	std::string cameraName = "CameraTop";
	int space = 2; //FRAME_ROBOT
	bool useSensorValues = true;
	std::vector<float> transVec = motion_proxy_ptr->getTransform(cameraName, space, useSensorValues);
	Eigen::Matrix4f transMat;
	transMat <<
		transVec[0] , transVec[1] , transVec[2] , transVec[3] ,
		transVec[4] , transVec[5] , transVec[6] , transVec[7] ,
		transVec[8] , transVec[9] , transVec[10], transVec[11],
		transVec[12], transVec[13], transVec[14], transVec[15];

	Eigen::Vector4f vec(v(0),v(1),v(2),1);
	vec = transMat*vec;

	// Projection on the floor
	// -HFOV/2 <= alpha <= HFOV/2
	double alpha = atan((double)(vec(1)/vec(0)));

	return alpha;
}

double computeBearing()
{
	double alpha = relativeBearing();

	double theta;
	if(r1.x == r2.x)
	{
		if((r2.y-r1.y) < 0) {theta = -M_PI/2;}
		if((r2.y-r1.y) > 0) {theta = M_PI/2;}
	}
	else
	{
		theta = atan((r2.y-r1.y)/(r2.x-r1.x));
		if((r2.x-r1.x) < 0) {theta += M_PI;}
	}

	// Angle between ]-pi,pi]
	while(theta > M_PI) {theta -= 2*M_PI;}
	while(theta <= -M_PI) {theta += 2*M_PI;}

	return theta-alpha;
}


class TrackingAmongObstacles : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	ImageConverter* ic;

	TrackingAmongObstacles(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
	}

	~TrackingAmongObstacles()
	{
		delete motion_proxy_ptr;
		delete ic;
	}

	void initialize()
	{
		init_ = true;

		// Enable stiffness
		AL::ALValue stiffness_name("Body");
		AL::ALValue stiffness(1.0f);
		AL::ALValue stiffness_time(1.0f);
		motion_proxy_ptr->stiffnessInterpolation(stiffness_name,stiffness,stiffness_time);

		// Init moving
		motion_proxy_ptr->moveInit();

        // Robot detected
        robotDetected = true;
	}

	void finalize()
	{
		// Stop moving
		motion_proxy_ptr->stopMove();

		// Delete Filter
		delete ic;

		init_ = false;
		deactivate();
	}

	int executeCB(ros::Duration dt)
	{
		std::cout << "**TrackingAmongObstacles -%- Executing Main Task, elapsed_time: "
		          << dt.toSec() << std::endl;
		std::cout << "**TrackingAmongObstacles -%- execute_time: "
		          << execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			set_feedback(RUNNING);
			initialize();

			// Launch Particle Filter
			ic = new ImageConverter();
		}

		// Robot not detected
		if(!robotDetected)
		{
			set_feedback(FAILURE);
			finalize();
			return 1;
		}

		// Close to the other robot
		//ROS_INFO("Depth = %f, r = %f, l = %f",depth,right,left);
		if(depth < dist_threshold)
		{
			set_feedback(SUCCESS);
			finalize();
			return 1;
		}

		// Controller
		int y_rel = y - width/2;
		double angular = -alpha*y_rel;
		// Thresholds
		if(angular > 1) {angular = 1;}
		if(angular < -1) {angular = -1;}

		// Publish
		geometry_msgs::Twist cmd;
		cmd.linear.x = rho;
		cmd.angular.z = angular;
		cmd_pub.publish(cmd);

		return 0;
	}

	void resetCB()
	{
		execute_time_ = (ros::Duration) 0;
	}
};


void receive_odometry1(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r1.x = msg->x;
	r1.y = msg->y;
	r1.vx = msg->vx;
	r1.vy = msg->vy;
}


void receive_odometry2(const nao_behavior_tree::Odometry::ConstPtr &msg)
{
	r2.x = msg->x;
	r2.y = msg->y;
	r2.vx = msg->vx;
	r2.vy = msg->vy;
}


int main(int argc, char** argv)
{
	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {r1.id = "1"; r2.id = "2";}
		if(atoi(argv[1]) == 2) {r1.id = "2"; r2.id = "1";}

		ros::init(argc, argv,"TrackingAmongObstacles" + r1.id);
		ros::NodeHandle nh;

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

		// Odometry subscribers
		ros::Subscriber odom1_sub = nh.subscribe("/odometry" + r1.id,1000,receive_odometry1);
		ros::Subscriber odom2_sub = nh.subscribe("/odometry" + r2.id,1000,receive_odometry2);

		// Walker publisher
		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + r1.id,100);

		// Launch Server
		TrackingAmongObstacles server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();

		cvDestroyAllWindows();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}