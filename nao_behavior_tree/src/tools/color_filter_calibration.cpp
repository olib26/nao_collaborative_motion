/*
 * color_filter_calibration.cpp
 *
 *  Created on: Mar 19, 2014
 *      Author: Olivier BALLAND
 */


#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "nao_behavior_tree/tools/color_filter_calibration.hpp"

namespace enc = sensor_msgs::image_encodings;


IplImage* filter(IplImage* img)
{
	CvSize sz = cvGetSize(img);
	IplImage* hsv_image = cvCreateImage(sz,8,3);
	IplImage* hsv_mask = cvCreateImage(sz,8,1);
	cvCvtColor(img,hsv_image,CV_BGR2HSV);
	CvScalar hsv_min = cvScalar(H_MIN,S_MIN,V_MIN,0);
	CvScalar hsv_max = cvScalar(H_MAX,S_MAX,V_MAX,0);
	cvInRangeS(hsv_image, hsv_min, hsv_max, hsv_mask);

	return hsv_mask;
}


class ImageConverter
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;

public:
	IplImage* img;
	IplImage* hsv_image;
	IplImage* hsv_mask;

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

		hsv_mask = filter(img);

		cvNamedWindow("hsv-msk",1); cvShowImage("hsv-msk", hsv_mask);
		cvWaitKey(10);
	}
};


void on_trackbar( int,void*)
{
}


int main(int argc, char** argv)
{
	ros::init(argc, argv,"color_filter_calibration");
	ros::NodeHandle pnh("~");

	// Trackbars
	cvNamedWindow("HSV Thresholds",1);
	cv::createTrackbar("HUE MIN:","HSV Thresholds",&H_MIN,180,on_trackbar);
	cv::createTrackbar("HUE MAX:","HSV Thresholds",&H_MAX,180,on_trackbar);
	cv::createTrackbar("SAT MIN:","HSV Thresholds",&S_MIN,255,on_trackbar);
	cv::createTrackbar("SAT MAX:","HSV Thresholds",&S_MAX,255,on_trackbar);
	cv::createTrackbar("VAL MIN:","HSV Thresholds",&V_MIN,255,on_trackbar);
	cv::createTrackbar("VAL MAX:","HSV Thresholds",&V_MAX,255,on_trackbar);

	// Init trackbars
	pnh.param("H_MIN",H_MIN,int(0)); cv::setTrackbarPos("HUE MIN:","HSV Thresholds",H_MIN);
	pnh.param("H_MAX",H_MAX,int(0)); cv::setTrackbarPos("HUE MAX:","HSV Thresholds",H_MAX);
	pnh.param("S_MIN",S_MIN,int(0)); cv::setTrackbarPos("SAT MIN:","HSV Thresholds",S_MIN);
	pnh.param("S_MAX",S_MAX,int(0)); cv::setTrackbarPos("SAT MAX:","HSV Thresholds",S_MAX);
	pnh.param("V_MIN",V_MIN,int(0)); cv::setTrackbarPos("VAL MIN:","HSV Thresholds",V_MIN);
	pnh.param("V_MAX",V_MAX,int(0)); cv::setTrackbarPos("VAL MAX:","HSV Thresholds",V_MAX);

	cvWaitKey(10);

	if(argc != 1)
	{
		bool webcam;

		// Robot selection
		if(atoi(argv[1]) == 1) {nao = "1";}
		if(atoi(argv[1]) == 2) {nao = "2";}

		if(atoi(argv[1]) == 0) {webcam = true;}

		if(!webcam)
		{
			ImageConverter ic;
			ros::spin();
		}
		else
		{
			int id;
			pnh.param("id",id,int(0));
			CvCapture* capture = cvCaptureFromCAM(id);

			while(ros::ok())
			{

				IplImage* img = cvQueryFrame(capture);
				IplImage* hsv_mask = filter(img);
				cvNamedWindow("Camera_Output",1); cvShowImage("Camera_Output", hsv_mask);
				cvWaitKey(10);
			}

			cvReleaseCapture(&capture);
		}

		cvDestroyAllWindows();
	}

	else
	{
		puts("Error, not enough arguments");
	}

	return 0;
}
