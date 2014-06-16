#include "nao_behavior_tree/rosaction.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

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

#include "nao_behavior_tree/filters/particleFilter.hpp"

#include <geometry_msgs/Twist.h>
#include <nao_behavior_tree/Sonar.h>

#include "nao_behavior_tree/actions/GoClose.hpp"

namespace enc = sensor_msgs::image_encodings;


float objectDepth(Object object)
{
	// Two interesting points
	float p_u1 = -(object.x + (object.sx-height)/2);
	float p_u2 = -(object.x - (object.sx+height)/2);
	float p_v = -(object.y - width/2);
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


void imageProcessing(IplImage* img)
{
	// Remove top
	CvSize sz = cvGetSize(img);
	CvPoint p1 = cvPoint(0,0);
	CvPoint p2 = cvPoint(sz.width-1,cutHeight-1);
	CvScalar color = cvScalar(0,0,0);
	cvRectangle(img,p1,p2,color,CV_FILLED);

	// Image size
	height = sz.height;
	width = sz.width;

	// Create temporary images
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);
	IplImage* hsv_mask_int = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);

	if(H_MIN > H_MAX)
	{
		CvScalar hsv_min_int = cvScalar(0,S_MIN,V_MIN,0);
		CvScalar hsv_max_int = cvScalar(180,S_MAX,V_MAX,0);

		cvInRangeS(hsv_image,hsv_min_int,hsv_max,hsv_mask);
		cvInRangeS(hsv_image,hsv_min,hsv_max_int,hsv_mask_int);

		cvAdd(hsv_mask,hsv_mask_int,hsv_mask);
	}
	else {cvInRangeS(hsv_image,hsv_min,hsv_max,hsv_mask);}

	// Particle Filter
	PF.imageProcessing(hsv_mask);

	// Compute standard deviation
	std::pair<double,double> V = PF.particlesStD(0);
	ROS_INFO("Standard deviation:  Vx = %f, Vy = %f",V.first,V.second);
	if((V.first > StD_max) & (V.second > StD_max))
	{
		robotDetected = false;
	}

	if(robotDetected)
	{
		// Draw robot
		hsv_mask = PF.drawObject(hsv_mask,0);

		// Compute depth
		depth = objectDepth(PF.getObject(0));
	}

	// Draw particles
	hsv_mask = PF.drawParticles(hsv_mask,0);

	// Show result
	cvNamedWindow("GoClose",1); cvShowImage("GoClose",hsv_mask);

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


class GoClose : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* robotPosture;
	ImageConverter* ic;

	GoClose(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
	}

	~GoClose()
	{
		delete motion_proxy_ptr;
		delete robotPosture;
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

		// Stand
		//robotPosture->goToPosture("Stand",0.5f);

		// Init moving
		motion_proxy_ptr->moveInit();

		// Head pitch
		AL::ALValue name = "HeadPitch";
		float angle = 0;
		float fractionMaxSpeed = 0.05;
		motion_proxy_ptr->setAngles(name,angle,fractionMaxSpeed);

        // Robot detected
        robotDetected = true;

		// Init particles
		//PF.initParticles();
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
		std::cout << "**GoClose -%- Executing Main Task, elapsed_time: "
		          << dt.toSec() << std::endl;
		std::cout << "**GoClose -%- execute_time: "
		          << execute_time_.toSec() << std::endl;
		execute_time_ += dt;

		if(!init_)
		{
			set_feedback(RUNNING);
			initialize();

			// Launch Image Converter
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
		ROS_INFO("Depth = %f, r = %f, l = %f",depth,right,left);
		bool sonarCond;
		if(sonar)
		{
			sonarCond = ((right < distThreshold) | (left < distThreshold));
		}
		else
		{
			sonarCond = true;
		}

		if(sonarCond & (depth < distThreshold))
		{
			set_feedback(SUCCESS);
			finalize();
			return 1;
		}

		// Object coordinates
		Object object = PF.getObject(0);

		// Controller
		int y_rel = object.y - width/2;

		double linear = rho;
		if((right < 0.8) | (left < 0.8)) {linear = rho/4;}
		if(fabs(y_rel) > yThreshold) {linear = 0;}

		double angular = -alpha*y_rel;
		// Thresholds
		if(angular > 1) {angular = 1;}
		if(angular < -1) {angular = -1;}

		// Publish
		geometry_msgs::Twist cmd;
		cmd.linear.x = linear;
		cmd.angular.z = angular;
		cmd_pub.publish(cmd);

		return 0;
	}

	void resetCB()
	{
		execute_time_ = (ros::Duration) 0;
	}
};


void receive_sonar(const nao_behavior_tree::Sonar::ConstPtr &msg)
{
	right = msg->right;
	left = msg->left;
}


int main(int argc, char** argv)
{
	if(argc != 1)
	{
		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		ros::init(argc, argv,"GoClose" + nao);
		ros::NodeHandle nh;

		ros::NodeHandle pnh("~");
		// Robot parameters
		std::string NAO_IP;
		int NAO_PORT;
		pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
		pnh.param("NAO_PORT",NAO_PORT,int(9559));
		// Sonar ON/OFF
		pnh.param("sonar",sonar,bool(false));
		// HSV parameters
		pnh.param("H_MIN",H_MIN,int(0));
		pnh.param("H_MAX",H_MAX,int(0));
		pnh.param("S_MIN",S_MIN,int(0));
		pnh.param("S_MAX",S_MAX,int(0));
		pnh.param("V_MIN",V_MIN,int(0));
		pnh.param("V_MAX",V_MAX,int(0));
		hsv_min = cvScalar(H_MIN,S_MIN,V_MIN,0);
		hsv_max = cvScalar(H_MAX,S_MAX,V_MAX,0);

		// Sonar subscriber
		ros::Subscriber sonar_sub = nh.subscribe("/sonar" + nao,1000,receive_sonar);

		// Walker publisher
		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + nao,100);

		// Launch Server
		GoClose server(ros::this_node::getName(),NAO_IP,NAO_PORT);

		ros::spin();

		cvDestroyAllWindows();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
