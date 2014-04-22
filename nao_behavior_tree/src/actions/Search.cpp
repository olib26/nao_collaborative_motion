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

#include "nao_behavior_tree/filters/particleFilter.hpp"

#include <geometry_msgs/Twist.h>

#include "nao_behavior_tree/actions/Search.hpp"

namespace enc = sensor_msgs::image_encodings;


void imageProcessing(IplImage* img)
{
	// Remove top
	CvSize sz = cvGetSize(img);
	CvPoint p1 = cvPoint(0,0);
	CvPoint p2 = cvPoint(sz.width-1,cutHeight-1);
	CvScalar color = cvScalar(0,0,0);
	cvRectangle(img,p1,p2,color,CV_FILLED);

	// Create temporary images
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	cvInRangeS(hsv_image,hsv_min,hsv_max,hsv_mask);

	// Particle Filter
	PF.imageProcessing(hsv_mask);

	// Compute standard deviation
	std::pair<double,double> V = PF.particlesStD(0);
	ROS_INFO("Standard deviation:  Vx = %f, Vy = %f",V.first,V.second);
	if((V.first < StD_minx) & (V.second < StD_miny))
	{
		robotDetected = true;
	}

	// Draw particles
	hsv_mask = PF.drawParticles(hsv_mask,0);

	// Show result
	//cvNamedWindow("Search",1); cvShowImage("Search",hsv_mask);

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
	AL::ALRobotPostureProxy* robotPosture;

	Search(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
	}

	~Search()
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
		//motion_proxy_ptr->moveInit();

		// Head pitch
		AL::ALValue name = "HeadPitch";
		float angle = 0;
		float fractionMaxSpeed = 0.05;
		motion_proxy_ptr->setAngles(name,angle,fractionMaxSpeed);

		// Robot not detected
		robotDetected = false;

		// Init particles
		//PF.initParticles();
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

			// Launch Image Converter
			ic = new ImageConverter();
		}

		// Rotate
		geometry_msgs::Twist cmd;
		cmd.angular.z = 0.1;
		cmd_pub.publish(cmd);

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

		// Walker publisher
		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + nao,100);

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
