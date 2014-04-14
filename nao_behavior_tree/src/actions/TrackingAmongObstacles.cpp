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
#include <nao_behavior_tree/Odometry.h>
#include <nao_behavior_tree/Bearing.h>
#include <nao_behavior_tree/Velocity.h>
#include <nao_behavior_tree/Sonar.h>

#include "nao_behavior_tree/actions/TrackingAmongObstacles.hpp"

namespace enc = sensor_msgs::image_encodings;


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

	// HSV Conversion and Thresholding
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	cvInRangeS(hsv_image,hsv_min,hsv_max, hsv_mask);

	// Particle Filter
	PF.imageProcessing(hsv_mask);

	// Compute standard deviation
	std::pair<double,double> V = PF.particlesStD(0);
	robotDetected = !((V.first > StD_max) & (V.second > StD_max));

	// Draw robot
	if(robotDetected) {hsv_mask = PF.drawObject(hsv_mask,0);}

	// Show result
	cvNamedWindow("TAO",1); cvShowImage("TAO",hsv_mask);

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


double modulo2Pi(double theta)
{
	// Angle between ]-pi,pi]
	while(theta > M_PI) {theta -= 2*M_PI;}
	while(theta <= -M_PI) {theta += 2*M_PI;}
	return theta;
}


double angle(double dx, double dy)
{
	double theta;

	if(dx == 0)
	{
		if(dy < 0) {theta = -M_PI/2;}
		if(dy > 0) {theta = M_PI/2;}
	}
	else
	{
		theta = atan(dy/dx);
		if(dx < 0) {theta += M_PI;}
	}

	modulo2Pi(theta);

	return theta;
}


float computeDistance()
{
	return sqrt((r1.x-r2.x)*(r1.x-r2.x) + (r1.y-r2.y)*(r1.y-r2.y));
}


double relativeBearing(Object object)
{
	// Other robot parameters
	float p_x = -(object.y - width/2);
	float p_y = -(object.x - height/2);
	float f = (float)width/2/tan(HFOV/2.);

	// Vector pointing to the other robot
	Eigen::Vector3f v(f,p_x,p_y);

	// Transformation to FRAME_ROBOT
	std::string cameraName = "CameraTop";
	int space = 2; //FRAME_ROBOT
	bool useSensorValues = true;
	std::vector<float> transVec = motion_proxy_ptr->getTransform(cameraName,space,useSensorValues);
	Eigen::Matrix3f transMat;
	transMat <<
		transVec[0] , transVec[1] , transVec[2] ,
		transVec[4] , transVec[5] , transVec[6] ,
		transVec[8] , transVec[9] , transVec[10];

	// Change frame
	v = transMat*v;

	// Projection on the floor
	// -HFOV/2 <= alpha <= HFOV/2
	double alpha = atan((double)(v(1)/v(0)));

	return -alpha;
}


double absoluteBearing()
{
	return angle(r2.x-r1.x,r2.y-r1.y);
}


class TrackingAmongObstacles : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	AL::ALRobotPostureProxy* robotPosture;
	ImageConverter* ic;

	TrackingAmongObstacles(std::string name,std::string NAO_IP,int NAO_PORT) :
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0)
	{
		motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);
		robotPosture = new AL::ALRobotPostureProxy(NAO_IP,NAO_PORT);
	}

	~TrackingAmongObstacles()
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

		// Depth
		depth = computeDistance();

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

		// Publish bearings
		nao_behavior_tree::Bearing bearing;
		bearing.relative = relativeBearing(PF.getObject(0));
		bearing.absolute = absoluteBearing();
		bearing.robotDetected = robotDetected;
		bearing_pub.publish(bearing);

		/*
		// Robot not detected
		if(!robotDetected)
		{
			set_feedback(FAILURE);
			finalize();
			return 1;
		}
		*/

		// Controller
		double angular = alpha*modulo2Pi(V.theta-(bearing.relative+bearing.absolute));
		double linear;
		if(fabs(angular) < angularThreshold) {linear = V.norm;}

		geometry_msgs::Twist cmd;
		cmd.linear.x = linear;
		cmd.angular.z = angular;
		//cmd_pub.publish(cmd);

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


void receive_velocity(const nao_behavior_tree::Velocity::ConstPtr &msg)
{
	V.norm = msg->norm;
	V.theta = msg->theta;
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
		ros::Subscriber sonar_sub = nh.subscribe("/sonar" + r1.id,1,receive_sonar);

		// Odometry subscribers
		ros::Subscriber odom1_sub = nh.subscribe("/odometry" + r1.id,1,receive_odometry1);
		ros::Subscriber odom2_sub = nh.subscribe("/odometry" + r2.id,1,receive_odometry2);

		// Velocity subscriber
		ros::Subscriber vel_sub = nh.subscribe("/vel" + r1.id,1,receive_velocity);

		// Walker publisher
		cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel" + r1.id,1);

		// Bearing publisher
		bearing_pub = nh.advertise<nao_behavior_tree::Bearing>("/bearing" + r1.id,1);

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
