//TO UNDERSTAND THE CODE, WATCH THE REPORT

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "WQUPC.h"
#include "imageconverter.h"
#include "nao_behavior_tree/BallPosForHead.h"
#include "nao_behavior_tree/BallPosForHand.h"

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <Eigen/Dense>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

const char WINDOW[] = "Image window";
const char WINDOW2[] = "Image window2";
const char WINDOW3[] = "Image window3";
const char WINDOW4[] = "Image window4";
AL::ALMotionProxy* motion_proxy_ptr;
std::string NAO_ID;

class ImageConverter {
    ros::NodeHandle nh_;

    ros::ServiceServer service1;
    ros::ServiceServer service2;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    Mat imgcam;
    vector<vector<int> >meltingList;

public:

    ImageConverter() :
        it_(nh_) {
        image_sub_ = it_.subscribe("/image_raw" + NAO_ID, 1,
                &ImageConverter::receiveImage, this);
        cv::namedWindow(WINDOW);
        cv::namedWindow(WINDOW2);
        cv::namedWindow(WINDOW3);
        cv::namedWindow(WINDOW4);

//====> ici j'ai rajouté les NAO_ID pour spécifier le nom
        service1 = nh_.advertiseService("ballposhead"+NAO_ID, &ImageConverter::ballPosForHead,this);
        service2 = nh_.advertiseService("ballposhand"+NAO_ID, &ImageConverter::ballPosForHand,this);
        ROS_INFO("Ready to send info");
        ROS_INFO("ID = %s",NAO_ID.c_str());
    }

    ~ImageConverter() {
        cv::destroyWindow(WINDOW);
    }

    //Method to print images
    void publishImage(const cv::Mat img,const char* WINDOW) {
        cv::waitKey(10);
        cv::imshow(WINDOW, img);
        cv::waitKey(10);
    }
    //////////

    //Method to receive an image from camera and to apply a treatment on
    void receiveImage(const sensor_msgs::ImageConstPtr& msg) {

        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //update the image
        imgcam = cv_ptr->image;
        //publishImage(imgcam,WINDOW);


        //here you can put some test functions
        //EXAMPLES:
        //test7(imgcam);
        //publishImage(test3(imgcam),WINDOW);


    }
    //////////




    //ZONE 1 GLOBAL TESTS

    //algo pour faire juste la detection
    Mat test1(Mat img_cam){
        //nao red simulator 0-30
        //pink ball 150-180
        //yellow ball 15-35
        //blu dino 100-120

        Mat output_im =colorDetectionHSV(img_cam,150,180,80,255,80,255);
        //Mat output_im = colorDetectionRGB(img_cam, 0.94, 0.08, 0.01, 80);
        publishImage(output_im,WINDOW3);
        return output_im;
    }
    //////////

    //algo pour tout avoir la position
    Mat test2(Mat img_cam){
        //with RGB
        //vector<vector<int> > chain =getLongestChain(getContourBall2(imgcam, 0.94, 0.08, 0.01, 140));
        //with HSV
        vector<vector<int> > chain =getLongestChain(getContourBall(img_cam, 150,180,80,255,80,255),1);


        Mat output_im = drawMatFromList(chain,img_cam);
        vector <double> ballinfo; ballinfo = ballInfo(chain);


        Eigen::Vector3f coord=get3DcoordBall(ballinfo);

        ROS_INFO("x = %f",coord(0));
        ROS_INFO("y = %f",coord(1));
        ROS_INFO("z = %f",coord(2));

        return output_im;
    }
    //////////

    //algo pour tester les frontières
    Mat test3(Mat img_cam){

        Mat im(7,11, CV_8UC1, Scalar(0));
        im.at<uchar>(Point(2,1))=255;
        im.at<uchar>(Point(3,1))=255;
        im.at<uchar>(Point(1,1))=255;
        im.at<uchar>(Point(1,1))=255;
        im.at<uchar>(Point(1,1))=255;
        im.at<uchar>(Point(1,1))=255;

        Mat grad=testGetContourBall(im);
        //Mat grad = getContourBall3(img_cam,150,180,80,255,80,255);
        return grad;
    }
    //////////

    //algo pour dessiner les chaines
    Mat test4(Mat img_cam){
        Mat chains = drawChains(getChains(getContourBall(img_cam,150,180,80,255,80,255)),img_cam);
        return chains;
    }
    //////////

    //algo pour l'algo global simplifié avec WQUPC
    Mat test5(Mat img_cam){
        //with RGB
        //vector<vector<int> > chain =getLongestChain(getContourBall2(imgcam, 0.94, 0.08, 0.01, 140),1);
        //with HSV

        vector<vector<vector<int> > > chains=getChains(getContourBall(img_cam, 150,180,80,255,80,255));
        vector<vector<vector<int> > >  meltedchainlist =meltChains(chains,meltingList);
        ROS_INFO("TAILLE=%i",meltedchainlist.size());

        vector<vector<int> > chain=chooseChain(meltedchainlist,1,img_cam);
        Mat Allchains = drawChains(chains,img_cam);

        Mat output_im = drawMatFromList(chain,img_cam);

        vector <double> ballinfo; ballinfo = ballInfo(chain);


        Eigen::Vector3f coord=get3DcoordBall(ballinfo);

                        ROS_INFO("x = %f",coord(0));
                        ROS_INFO("y = %f",coord(1));
                        ROS_INFO("z = %f",coord(2));

        return output_im;
    }
    //////////

    //algo pour tester les chaines
    void test6(){
        Mat im(7,11, CV_8UC1, Scalar(0));
        im.at<uchar>(Point(1,1))=255;
        im.at<uchar>(Point(3,1))=255;
        im.at<uchar>(Point(5,1))=255;
        im.at<uchar>(Point(7,1))=255;
        im.at<uchar>(Point(2,2))=255;
        im.at<uchar>(Point(4,2))=255;
        im.at<uchar>(Point(6,2))=255;
        im.at<uchar>(Point(8,2))=255;
        im.at<uchar>(Point(2,3))=255;
        im.at<uchar>(Point(4,3))=255;
        im.at<uchar>(Point(8,3))=255;
        im.at<uchar>(Point(2,4))=255;
        im.at<uchar>(Point(5,4))=255;
        im.at<uchar>(Point(6,4))=255;
        im.at<uchar>(Point(7,4))=255;
        im.at<uchar>(Point(9,4))=255;
        im.at<uchar>(Point(3,5))=255;
        im.at<uchar>(Point(9,5))=255;
        im.at<uchar>(Point(1,6))=255;
        im.at<uchar>(Point(3,6))=255;
        im.at<uchar>(Point(4,6))=255;
        im.at<uchar>(Point(5,6))=255;
        im.at<uchar>(Point(6,6))=255;
        im.at<uchar>(Point(7,6))=255;
        im.at<uchar>(Point(8,6))=255;

        vector<vector<vector<int> > > chains=getChains(im);

        vector<vector<vector<int> > >  metledchainlist =meltChains(chains,meltingList);

        Mat Allchains = drawChains(chains,im);
        Mat mAllchains = drawChains(metledchainlist,im);

        ROS_INFO("taille = %i",meltingList.size());
        for (int i=0;i<meltingList.size();i++){
            ROS_INFO("noeud 1 = %i",meltingList.at(i).at(0));
            ROS_INFO("noeud 2 = %i",meltingList.at(i).at(1));
        }

        publishImage(zoom2(zoom2(zoom2(zoom2(zoom2(Allchains))))),WINDOW);
        publishImage(zoom2(zoom2(zoom2(zoom2(zoom2(mAllchains))))),WINDOW2);

    }
    //////////

    //algo pour détecter rapidement la balle et dire de faire touner la tête
    void test7(Mat img_cam){

        Mat lowResBall = lowResBallFindingHSV(img_cam,150,180,80,255,80,255,2);
        int ballLocation = isBallInScreen(lowResBall,30);
        ROS_INFO("LOCATION=%i",ballLocation);
    }
    //////////


    //ZONE 2 CELL METHODS

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //IMAGE ANALYSIS METHODS

    //Method to invert and image in RGB
    Mat invertImage(Mat img_cam) {

        Mat im = img_cam;

        for (int i = 0; i < im.rows; ++i) {
            for (int j = 0; j < im.cols; ++j) {
                for (int k = 0; k < im.channels(); ++k) {

                    im.at<Vec3b>(Point(j, i))[k] = 255
                            - im.at<Vec3b>(Point(j, i))[k];
                }
            }
        }

        return im;
    }
    //////////

    //Method to subsample the image once
    Mat subsampleOnce(Mat img_cam) {

        Mat im(img_cam.rows / 2, img_cam.cols / 2, CV_8UC3, Scalar(0, 0, 0));

        for (int i = 0; i < im.rows; ++i) {
            for (int j = 0; j < im.cols; ++j) {
                for (int k = 0; k < im.channels(); ++k) {

                    im.at<Vec3b>(Point(j, i))[k] = img_cam.at<Vec3b>(
                            Point(2 * j, 2 * i))[k];
                }
            }
        }
        return im;
    }
    //////////

    //Method to subsample the image multiple times
    Mat subsampleMulti(Mat img_cam, int nbsub) {

        if (nbsub == 0) {
            return img_cam;
        }

        return subsampleMulti(subsampleOnce(img_cam), nbsub - 1);

    }
    //////////

    //Method to detect a certain color within a given threshold
    Mat colorDetectionRGB(Mat img_cam, double R, double G, double B,
            double thres) {

        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));

        for (int i = 0; i < im.rows; ++i) {
            for (int j = 0; j < im.cols; ++j) {
                    im.at<uchar>(Point(j, i)) = 255*thresCondOKRGB(img_cam,i,j,R,G,B,thres);
                //ROS_INFO("distance = %i",im.at<uchar>(Point(j, i)));
            }
        }

        return im;
    }
    //////////

    //Method to detect a certain color within a given threshold
    Mat colorDetectionHSV(Mat img_cam,double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX) {

        Mat imHSV;
        cvtColor(img_cam, imHSV, CV_BGR2HSV);


        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));

        for (int i = 0; i < im.rows; ++i) {
            for (int j = 0; j < im.cols; ++j) {
                im.at<uchar>(Point(j, i)) = 255*thresCondOKHSV(imHSV,i,j,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);
                //ROS_INFO("hvalue = %f",imHSV.at<uchar>(Point(j, i)));
            }
        }

        return im;
    }
    //////////


    //Method to verify the thresholding condition (dist<thres)
    int thresCondOKRGB(Mat img_cam,int i,int j,double R, double G, double B,
            double thres){

        double dx = (img_cam.at<Vec3b>(Point(j, i))[0] - B * 255);
        double dy = (img_cam.at<Vec3b>(Point(j, i))[1] - G * 255);
        double dz = (img_cam.at<Vec3b>(Point(j, i))[2] - R * 255);
        double dist = dx * dx + dy * dy + dz * dz;
        dist=sqrt(dist);
        if (dist < thres) return 1;
        return 0;
    }
    //////////


    //Method to threshold hsv image
    int thresCondOKHSV(Mat img_cam,int i, int j, double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX){

        double hvalue=img_cam.at<Vec3b>(Point(j, i))[0];
        //if (hvalue>90) hvalue-=180;

        if (hvalue>=HMIN){
            if(hvalue<=HMAX){
                if(img_cam.at<Vec3b>(Point(j, i))[1]>=SMIN){
                    if(img_cam.at<Vec3b>(Point(j, i))[1]<=SMAX){
                        if(img_cam.at<Vec3b>(Point(j, i))[2]>=VMIN){
                            if(img_cam.at<Vec3b>(Point(j, i))[2]<=VMAX){
                                return 1;
                            }
                        }
                    }
                }
            }
        }
        return 0;
    }
    //////////

    //Method to blur homogeneously an image
    Mat blurImage(Mat img_cam,int size){

        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));
        for ( int i = 1; i < 10; i = i + 2 )
        { blur( img_cam, im, Size( i, i ), Point(-1,-1) );
        }
        return im;
    }
    //////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //METHODS TO GET THE CONTOURS OF THE OBJECT


    //Method to get the contours of the ball with a gradient method for HSV. the output is not included into the input
    Mat getContourBall(Mat img_cam,double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX){

        Mat imHSV;
        cvtColor(img_cam, imHSV, CV_BGR2HSV);

        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));
        Mat im2(1, img_cam.cols, CV_8UC1, Scalar(0));
        int a,b;
        for (int j=0;j<im.cols;j++){im2.at<uchar>(Point(j,0))=thresCondOKHSV(imHSV,0,j,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);}

        for (int i=0;i<im.rows;i++){
            b=thresCondOKHSV(imHSV,i,0,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);

            for(int j=0;j<im.cols;j++){

                //gradient on x direction
                a=b;
                b=thresCondOKHSV(imHSV,i,j,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);
                im.at<uchar>(Point(j, i))=-fabs(b-a);

                //gradient on y direction
                if (b-a==0){
                    im.at<uchar>(Point(j,i))=-fabs(b-im2.at<uchar>(Point(j,0)));
                }
                im2.at<uchar>(Point(j,0))=b;
            }
        }
        return im;
    }
    //////////

    //Method to get the contours of the ball with a gradient method for RGB. the output is not included into the input
    Mat getContourBall2(Mat img_cam,double R, double G, double B,
            double thres){
        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));
        Mat im2(1, img_cam.cols, CV_8UC1, Scalar(0));
        int a,b;
        for (int j=0;j<im.cols;j++){im2.at<uchar>(Point(j,0))=thresCondOKRGB(img_cam,0,j,R,G,B,thres);}

        for (int i=0;i<im.rows;i++){
            b=thresCondOKRGB(img_cam,i,0,R,G,B,thres);

            for(int j=0;j<im.cols;j++){

                //gradient on x direction
                a=b;
                b=thresCondOKRGB(img_cam,i,j,R,G,B,thres);
                im.at<uchar>(Point(j, i))=-fabs(b-a);

                //gradient on y direction
                if (b-a==0){
                    im.at<uchar>(Point(j,i))=-fabs(b-im2.at<uchar>(Point(j,0)));
                }
                im2.at<uchar>(Point(j,0))=b;
            }
        }
        return im;
    }
    //

    //Method to get the contours of the ball with a gradient method for HSV. the output is "included" into the input
    Mat getContourBall3(Mat img_cam,double HMIN,double HMAX,double SMIN,double SMAX,double VMIN,double VMAX){

        Mat imHSV;//HSV image
        cvtColor(img_cam, imHSV, CV_BGR2HSV);


        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));//output image
        Mat im2(1, img_cam.cols, CV_8UC1, Scalar(0));//previous line
        int a,b;//backup variables

        for (int j=0;j<im.cols;j++){im2.at<uchar>(Point(j,0))=thresCondOKHSV(imHSV,0,j,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);}//initialization of im2

        for (int i=0;i<im.rows;i++){
            b=thresCondOKHSV(imHSV,i,0,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);//initialization of b=first column value

            for(int j=0;j<im.cols;j++){

                a=b;
                b=thresCondOKHSV(imHSV,i,j,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);//a always on pixel to the left of b excepted for j=0

                if((b-a)>0){im.at<uchar>(Point(j, i))=255;}
                else{    if((b-a)<0){im.at<uchar>(Point(j-1, i))=255;}
                        else{    if((b-im2.at<uchar>(Point(j,0)))>0){im.at<uchar>(Point(j, i))=255;}
                                else{    if((b-im2.at<uchar>(Point(j,0)))<0){im.at<uchar>(Point(j, i-1))=255;}
                                }
                        }
                }
                im2.at<uchar>(Point(j,0))=b;
            }
        }
        return im;
    }
    //////////


    //Method to get the contours of the ball with a gradient method for HSV. the output is "included" into the input
    Mat testGetContourBall(Mat img_cam){

        Mat imHSV=img_cam;


        Mat im(img_cam.rows, img_cam.cols, CV_8UC1, Scalar(0));//output image
        Mat im2(1, img_cam.cols, CV_8UC1, Scalar(0));//previous line
        int a,b;//backup variables

        for (int j=0;j<im.cols;j++){im2.at<uchar>(Point(j,0))=imHSV.at<uchar>(Point(j,0));}//initialization of im2

        for (int i=0;i<im.rows;i++){
            b=imHSV.at<uchar>(Point(0,i));//initialization of b=first column value

            for(int j=0;j<im.cols;j++){

                a=b;
                b=imHSV.at<uchar>(Point(j,i));//a always on pixel to the left of b excepted for j=0

                if((b-a)>0){im.at<uchar>(Point(j, i))=255;}
                else{    if((b-a)<0){im.at<uchar>(Point(j-1, i))=255;}
                        else{    if((b-im2.at<uchar>(Point(j,0)))>0){im.at<uchar>(Point(j, i))=255;}
                                else{    if((b-im2.at<uchar>(Point(j,0)))<0){im.at<uchar>(Point(j, i-1))=255;}
                                }
                        }
                }
                im2.at<uchar>(Point(j,0))=b;
            }
        }
        return im;
    }
    //////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //METHODS TO GET THE CHAINS

    //Method that gets all the chains in the image
    vector<vector<vector<int> > > getChains(Mat contoursImage){
                Mat im = contoursImage;
                vector<vector<vector<int> > > chainList;
                int numList=0;
                meltingList.clear();

                //preparation de l'image
                for(int i=0; i<contoursImage.rows;i++){
                    im.at<uchar>(Point(0,i))=0;
                    im.at<uchar>(Point(contoursImage.cols-1,i))=0;
                }
                for(int i=0; i<contoursImage.cols;i++){
                            im.at<uchar>(Point(i,0))=0;
                        }

                //lancement
                for (int i=1;i<contoursImage.rows;i++){
                    for (int j=1;j<contoursImage.cols-1;j++){

                        //white pixel arrives
                        if (contoursImage.at<uchar>(Point(j,i))!=0){

                            //if it is a lonely point
                            if ((contoursImage.at<uchar>(Point(j-1,i-1))==0 && contoursImage.at<uchar>(Point(j,i-1))==0 &&
                                    contoursImage.at<uchar>(Point(j+1,i-1))==0 && contoursImage.at<uchar>(Point(j-1,i))==0)){
                //ROS_INFO("point unique");
                                numList++;
                                im.at<uchar>(Point(j,i))=numList;
                                vector<int> dublet;
                                dublet.push_back(i);
                                dublet.push_back(j);
                                vector<vector<int> > pointList;
                                pointList.push_back(dublet);
                                chainList.push_back(pointList);
                            }


                            //if it has neighbours
                            else {

                                //left
                                if (contoursImage.at<uchar>(Point(j-1,i))!=0){
                //ROS_INFO("a un voisin a gauche");
                //cv::waitKey();

                                    vector<int> dublet;
                                    dublet.push_back(i);
                                    dublet.push_back(j);
                                    chainList.at(im.at<uchar>(Point(j-1,i))-1 ).push_back(dublet);
                                    im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j-1,i));

                                }
                                //upleft
                                else{if (contoursImage.at<uchar>(Point(j-1,i-1))!=0){
                //ROS_INFO("a un voisin en haut a gauche");

                                    vector<int> dublet;
                                    dublet.push_back(i);
                                    dublet.push_back(j);
                                    chainList.at(im.at<uchar>(Point(j-1,i-1))-1).push_back(dublet);
                                    im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j-1,i-1));

                                    }
                                //up
                                    else{if (contoursImage.at<uchar>(Point(j,i-1))!=0){
                //ROS_INFO("a un voisin en haut");

                                    vector<int> dublet;
                                    dublet.push_back(i);
                                    dublet.push_back(j);
                                    chainList.at(im.at<uchar>(Point(j,i-1))-1).push_back(dublet);
                                    im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j,i-1));

                                        }
                                        }
                                    }


                //ROS_INFO("verifie si haut droit existe");
                                //up right
                                if (contoursImage.at<uchar>(Point(j+1,i-1))!=0 ){

                                    if(contoursImage.at<uchar>(Point(j-1,i-1))!=0 &&
                                        contoursImage.at<uchar>(Point(j-1,i-1))!=contoursImage.at<uchar>(Point(j+1,i-1))){
                //ROS_INFO("a un voisin en haut a droite et en haut a gauche");
                                        vector<int> dublet2;
                                        dublet2.push_back(im.at<uchar>(Point(j+1,i-1))-1);
                                        dublet2.push_back(im.at<uchar>(Point(j-1,i-1))-1);
                                        meltingList.push_back(dublet2);

                                    }
                                    else{
                                        if(contoursImage.at<uchar>(Point(j-1,i))!=0 &&
                                                contoursImage.at<uchar>(Point(j-1,i))!=contoursImage.at<uchar>(Point(j+1,i-1))){
                //ROS_INFO("a un voisin en haut a droite et a gauche");
                                            vector<int> dublet2;
                                            dublet2.push_back(im.at<uchar>(Point(j+1,i-1))-1);
                                            dublet2.push_back(im.at<uchar>(Point(j-1,i))-1);
                                            meltingList.push_back(dublet2);

                                        }

                                        else{
                //ROS_INFO("a un voisin en haut a droite seulement");
                                            if( (contoursImage.at<uchar>(Point(j-1,i))==0) & (contoursImage.at<uchar>(Point(j-1,i-1))==0) &
                                                    (contoursImage.at<uchar>(Point(j,i-1))==0)){

                                            vector<int> dublet;
                                            dublet.push_back(i);
                                            dublet.push_back(j);

                                            chainList.at(im.at<uchar>(Point(j+1,i-1))-1).push_back(dublet);

                                            im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j+1,i-1));
                                            }

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                return chainList;
    }
    //////////

    //Second method that gets all the chains in the image
    vector<vector<vector<int> > > getChains2(Mat contoursImage){
        Mat im = contoursImage;
        vector<vector<vector<int> > > chainList;
        int numList=0;
        meltingList.clear();

        //preparation de l'image
        for(int i=0; i<contoursImage.rows;i++){
            im.at<uchar>(Point(0,i))=0;
            im.at<uchar>(Point(contoursImage.cols-1,i))=0;
        }
        for(int i=0; i<contoursImage.cols;i++){
            im.at<uchar>(Point(i,0))=0;
        }

        //lancement
        for (int i=1;i<contoursImage.rows;i++){
            for (int j=1;j<contoursImage.cols-1;j++){

                //white pixel arrives
                if (contoursImage.at<uchar>(Point(j,i))!=0){

                    //up
                    if (contoursImage.at<uchar>(Point(j,i-1))!=0){
                        //ROS_INFO("top neighbor");
                        vector<int> dublet;
                        dublet.push_back(i);
                        dublet.push_back(j);
                        chainList.at(im.at<uchar>(Point(j,i-1))-1).push_back(dublet);
                        im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j,i-1));
                    }
                    else{
                        if (contoursImage.at<uchar>(Point(j-1,i))!=0){
                            //ROS_INFO("left neighbor");
                            vector<int> dublet;
                            dublet.push_back(i);
                            dublet.push_back(j);
                            chainList.at(im.at<uchar>(Point(j-1,i))-1 ).push_back(dublet);
                            im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j-1,i));

                            if (contoursImage.at<uchar>(Point(j+1,i-1))!=0 ){
                                vector<int> dublet2;
                                dublet2.push_back(im.at<uchar>(Point(j+1,i-1))-1);
                                dublet2.push_back(im.at<uchar>(Point(j-1,i))-1);
                                meltingList.push_back(dublet2);
                            }
                        }
                        else{
                            if (contoursImage.at<uchar>(Point(j-1,i-1))!=0){
                                //ROS_INFO("top left neighbor");
                                vector<int> dublet;
                                dublet.push_back(i);
                                dublet.push_back(j);
                                chainList.at(im.at<uchar>(Point(j-1,i-1))-1).push_back(dublet);
                                im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j-1,i-1));

                                if (contoursImage.at<uchar>(Point(j+1,i-1))!=0 ){
                                    vector<int> dublet2;
                                    dublet2.push_back(im.at<uchar>(Point(j+1,i-1))-1);
                                    dublet2.push_back(im.at<uchar>(Point(j-1,i-1))-1);
                                    meltingList.push_back(dublet2);
                                }
                            }
                            else{
                                if (contoursImage.at<uchar>(Point(j+1,i-1))!=0){
                                    vector<int> dublet;
                                    dublet.push_back(i);
                                    dublet.push_back(j);
                                    chainList.at(im.at<uchar>(Point(j+1,i-1))-1).push_back(dublet);
                                    im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j+1,i-1));
                                }
                                else{
                                    //ROS_INFO("point unique");
                                    numList++;
                                    im.at<uchar>(Point(j,i))=numList;
                                    vector<int> dublet;
                                    dublet.push_back(i);
                                    dublet.push_back(j);
                                    vector<vector<int> > pointList;
                                    pointList.push_back(dublet);
                                    chainList.push_back(pointList);
                                }
                            }
                        }
                    }
                }
            }
        }
        return chainList;
    }
    //////////

    //Method that prints all the chains
    Mat drawChains(vector<vector<vector<int> > > chains,Mat contoursImage){
        Mat im(contoursImage.rows, contoursImage.cols, CV_8UC1, Scalar(0));
        for(int chain=0;chain<chains.size();chain++){
                for (int i=0;i<chains.at(chain).size();i++){
                    int y=chains.at(chain).at(i).at(1);
                    int x=chains.at(chain).at(i).at(0);
                    im.at<uchar>(Point(y,x))=255-1.5*chain*100/chains.size();
                }
        }
                return im;
    }
    //////////


    //Method WQUPC to meltable chains
    vector<vector<vector<int> > > meltChains(vector<vector<vector<int> > > chains,vector<vector<int> > meltingList){

        WQUPC meltedChain;
        meltedChain.meltChains(chains,meltingList);
        return meltedChain.sendBack();
    }
    //////////


    //Method to return the wanted chain
    vector<vector<int> > chooseChain(vector<vector<vector<int> > > chains,int kindOfLongestChain,Mat contoursImage){
        vector<vector<int> > finalList;

                if(kindOfLongestChain==0){

                    int max=0;
                    int pos=0;
                    for(int i=0;i<chains.size();i++){
                        if (chains.at(i).size()>max){
                            max=chains.at(i).size();pos=i;
                        }
                    }
                    finalList=chains.at(pos);
                    return finalList;
                }


                if(kindOfLongestChain==1){

                    int p0=0; int pos=0;
                    for(int i=0;i<chains.size();i++){
                        int minx=contoursImage.rows; int miny=contoursImage.cols; int maxx=0; int maxy=0;
                        for(int j=0;j<chains.at(i).size();j++){
                            if (chains.at(i).at(j).at(0)<minx){minx=chains.at(i).at(j).at(0);}
                            if (chains.at(i).at(j).at(0)>maxx){maxx=chains.at(i).at(j).at(0);}
                            if (chains.at(i).at(j).at(1)<miny){miny=chains.at(i).at(j).at(1);}
                            if (chains.at(i).at(j).at(1)>maxy){maxy=chains.at(i).at(j).at(1);}
                        }
                        if((maxx-minx)*(maxy-miny)>p0){p0=(maxx-minx)*(maxy-miny);pos=i;}
                    }

                    finalList=chains.at(pos);

                    return finalList;
                }
    }
    //////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //Method that get from the image with the contours the longest chain of pixels
    //USELESS NOW
    vector<vector<int> > getLongestChain(Mat contoursImage,int kindOfLongestChain){
        Mat im = contoursImage;
        vector<vector<vector<int> > > chainList;
        int numList=0;
        vector<vector<int> > meltingList;

        for(int i=0; i<contoursImage.rows;i++){
            im.at<uchar>(Point(0,i))=0;
            im.at<uchar>(Point(contoursImage.cols-1,i))=0;
        }

        for(int i=0; i<contoursImage.cols;i++){
                    im.at<uchar>(Point(i,0))=0;
                }

        for (int i=1;i<contoursImage.rows;i++){
            for (int j=1;j<contoursImage.cols-1;j++){

                //white pixel arrives
                if (contoursImage.at<uchar>(Point(j,i))!=0){
//                    ROS_INFO("i = %i",i);
//                    ROS_INFO("j = %i",j);
//                    ROS_INFO("pixel_gauche = %i",contoursImage.at<uchar>(Point(j-1,i)) );

                    //if it is a lonely point
                    if ((contoursImage.at<uchar>(Point(j-1,i-1))==0 && contoursImage.at<uchar>(Point(j,i-1))==0 &&
                            contoursImage.at<uchar>(Point(j+1,i-1))==0 && contoursImage.at<uchar>(Point(j-1,i))==0)){
        //ROS_INFO("point unique");
                        numList++;
                        im.at<uchar>(Point(j,i))=numList;
                        vector<int> dublet;
                        dublet.push_back(i);
                        dublet.push_back(j);
                        vector<vector<int> > pointList;
                        pointList.push_back(dublet);
                        chainList.push_back(pointList);
                    }


                    //if it has neighbours
                    else {

                        //left
                        if (contoursImage.at<uchar>(Point(j-1,i))!=0){
        //ROS_INFO("a un voisin a gauche");
        //cv::waitKey();

                            vector<int> dublet;
                            dublet.push_back(i);
                            dublet.push_back(j);
                            chainList.at(im.at<uchar>(Point(j-1,i))-1 ).push_back(dublet);
                            im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j-1,i));
                        }
                        //upleft
                        else{if (contoursImage.at<uchar>(Point(j-1,i-1))!=0){
        //ROS_INFO("a un voisin en haut a gauche");

                            vector<int> dublet;
                            dublet.push_back(i);
                            dublet.push_back(j);
                            chainList.at(im.at<uchar>(Point(j-1,i-1))-1).push_back(dublet);
                            im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j-1,i-1));
                        }
                        //up
                        else{if (contoursImage.at<uchar>(Point(j,i-1))!=0){
        //ROS_INFO("a un voisin en haut");

                            vector<int> dublet;
                            dublet.push_back(i);
                            dublet.push_back(j);
                            chainList.at(im.at<uchar>(Point(j,i-1))-1).push_back(dublet);
                            im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j,i-1));
                        }
                        }
                        }

        //ROS_INFO("verifie si haut droit existe");
                        //up right
                        if (contoursImage.at<uchar>(Point(j+1,i-1))!=0 ){

                            if(contoursImage.at<uchar>(Point(j-1,i-1))!=0 &&
                                contoursImage.at<uchar>(Point(j-1,i-1))!=contoursImage.at<uchar>(Point(j+1,i-1))){
        //ROS_INFO("a un voisin en haut a droite et en haut a gauche");
                                vector<int> dublet2;
                                dublet2.push_back(im.at<uchar>(Point(j+1,i-1)));
                                dublet2.push_back(im.at<uchar>(Point(j-1,i-1)));
                                meltingList.push_back(dublet2);

                            }
                            else{
                                if(contoursImage.at<uchar>(Point(j-1,i))!=0 &&
                                        contoursImage.at<uchar>(Point(j-1,i))!=contoursImage.at<uchar>(Point(j+1,i-1))){
        //ROS_INFO("a un voisin en haut a droite et a gauche");
                                    vector<int> dublet2;
                                    dublet2.push_back(im.at<uchar>(Point(j+1,i-1)));
                                    dublet2.push_back(im.at<uchar>(Point(j-1,i)));
                                    meltingList.push_back(dublet2);

                                }

                                else{
        //ROS_INFO("a un voisin en haut a droite seulement");
                                    vector<int> dublet;
                                    dublet.push_back(i);
                                    dublet.push_back(j);
                                    chainList.at(im.at<uchar>(Point(j+1,i-1))-1).push_back(dublet);
                                    im.at<uchar>(Point(j,i))=im.at<uchar>(Point(j+1,i-1));
                                }
                            }
                        }
                    }
                }
            }
        }

//FUSION DES LISTES
        int size=meltingList.size();
//        ROS_INFO("taille de meltinglist = %i",size);
//        ROS_INFO("taille de chainlist = %i",chainList.size());

        for (int j=0;j<size;j++){
            //ROS_INFO("j = %i",j);
            vector<int> fusion = meltingList.back();
            meltingList.pop_back();

//            ROS_INFO("taille de fusion = %i",fusion.size());
//            ROS_INFO("fusion 0 = %i",fusion.at(0));
//            ROS_INFO("fusion 1 = %i",fusion.at(1));

            int vectorToReportSize = chainList.at(fusion.at(1)-1).size();
            //ROS_INFO("taille vecteur a reporter = %i",vectorToREportSize);

            for(int i=0;i<vectorToReportSize;i++){
            chainList.at(fusion.at(0)-1).push_back(chainList.at(fusion.at(1)-1).at(i));
            }
        }


//CHOIX DE CHAINE
        vector<vector<int> > finalList;

        if(kindOfLongestChain==0){

            int max=0;
            int pos=0;
            for(int i=0;i<chainList.size();i++){
                if (chainList.at(i).size()>max){
                    max=chainList.at(i).size();pos=i;
                }
            }
            finalList=chainList.at(pos);
            return finalList;
        }


        if(kindOfLongestChain==1){

            ROS_INFO("size=%i",chainList.size());
            int p0=0; int pos=0;
            for(int i=0;i<chainList.size();i++){
                int minx=contoursImage.rows; int miny=contoursImage.cols; int maxx=0; int maxy=0;
                for(int j=0;j<chainList.at(i).size();j++){
                    if (chainList.at(i).at(j).at(0)<minx){minx=chainList.at(i).at(j).at(0);}
                    if (chainList.at(i).at(j).at(0)>maxx){maxx=chainList.at(i).at(j).at(0);}
                    if (chainList.at(i).at(j).at(1)<miny){miny=chainList.at(i).at(j).at(1);}
                    if (chainList.at(i).at(j).at(1)>maxy){maxy=chainList.at(i).at(j).at(1);}
                }
                if((maxx-minx)*(maxy-miny)>p0){p0=(maxx-minx)*(maxy-miny);pos=i;}
            }
            ROS_INFO("pos=%i",pos);
            finalList=chainList.at(pos);
            ROS_INFO("pos=%i",pos);
            return finalList;
        }

    }
    //////////





    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //METHODS TO GET INFO ON THE BALL

    //Method to get the position a the ball on the image given its coutours
    vector <double> ballInfo (vector<vector<int> > contours){

        int N=contours.size();
        Eigen::MatrixXd pos(N,2);
        Eigen::MatrixXd ave(2,1);
        Eigen::MatrixXd cov(2,2);

        for(int i=0;i<N;i++){
            pos(i,0)=contours.at(i).at(0);
            ave(0,0)+=pos(i,0);
            pos(i,1)=contours.at(i).at(1);
            ave(1,0)+=pos(i,1);
        }
        ave=ave/N;
        //pos=pos-ave;


        for(int i=0;i<N;i++){
                    pos(i,0)=pos(i,0)-ave(0,0);
                    pos(i,1)=pos(i,1)-ave(1,0);
                }

        cov=(double)1/N*pos.transpose()*pos;


        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov);
        Eigen::MatrixXd eigenvalues=eigensolver.eigenvalues();
        Eigen::MatrixXd eigenvectors=eigensolver.eigenvectors();

        vector<double> ballinfo;
        ballinfo.push_back(ave(0,0));
        ballinfo.push_back(ave(1,0));
        ballinfo.push_back(eigenvalues(0,0));
        ballinfo.push_back(eigenvalues(1,0));
        //because ev are classified in crescent order
        ballinfo.push_back(eigenvectors(0,1));
        ballinfo.push_back(eigenvectors(1,1));
        return ballinfo;

    }
    //////////

    //Method to give the 3d position of the ball given its size
    Eigen::Vector3f get3DcoordBall (vector <double> ballinfo){

    //Get the 2 points
    Eigen::Vector2d position2D;
    position2D(0)=ballinfo.at(0);
    position2D(1)=ballinfo.at(1);
    double length=sqrt(ballinfo.at(3));//radius of the ball
    Eigen::Vector2d longaxis;
    longaxis(0)=ballinfo.at(4);
    longaxis(1)=ballinfo.at(5);
    longaxis=longaxis/longaxis.norm();

    //ROS_INFO("p0x = %f",position2D(0)-imgcam.rows/2);
    //ROS_INFO("p0y = %f",position2D(1)-imgcam.cols/2);
    Eigen::Vector2d p1,p2;
    p1=position2D+length*longaxis;
    p2=position2D-length*longaxis;
//    ROS_INFO("length = %f",length);
//    ROS_INFO("p1x = %f",p1(0)-imgcam.rows/2);
//    ROS_INFO("p1y = %f",p1(1)-imgcam.cols/2);
//    ROS_INFO("p2x = %f",p2(0)-imgcam.rows/2);
//    ROS_INFO("p2y = %f",p2(1)-imgcam.cols/2);


    //Get tan(alpha) and tan(beta) for both points
    double half_vertical_angle = 47.64*M_PI/180.0/2;
    double half_horizontal_angle = 60.97*M_PI/180.0/2;
    double H=imgcam.rows;
    double L=imgcam.cols;
    //with alpha on x image direction and beta on y image direction
//=====>au cas ou ici j'ai inversé les /H et /L en fin de ligne et les vertical horizontal
    double tanalpha1 = (p1(0)-H/2)*2*tan(half_vertical_angle)/H;
    double tanbeta1 = (p1(1)-L/2)*2*tan(half_horizontal_angle)/L;
    double tanalpha2 = (p2(0)-H/2)*2*tan(half_vertical_angle)/H;
    double tanbeta2 = (p2(1)-L/2)*2*tan(half_horizontal_angle)/L;
//    ROS_INFO("a1 = %f",atan(tanalpha1));
//    ROS_INFO("b1 = %f",atan(tanbeta1));
//    ROS_INFO("a2 = %f",atan(tanalpha2));
//    ROS_INFO("b2 = %f",atan(tanbeta2));


    //Get 3d vectors rewritten in the x,y,z camera referential
    Eigen::Vector3d v1,v2,v3;
    v1(0)=1;v1(1)=-tanbeta1;v1(2)=-tanalpha1;
    v2(0)=1;v2(1)=-tanbeta2;v2(2)=-tanalpha2;
    v1=v1/v1.norm();
    v2=v2/v2.norm();
    v3=v1+v2;v3/=2;v3=v3/v3.norm();
//===>LA
    //ROS_INFO("v3x = %f",v3(0));
    //ROS_INFO("v3y = %f",v3(1));
    //ROS_INFO("v3z = %f",v3(2));

    //Get distance to ball
    double rayon=0.028;
    double cosdelta=v3.transpose()*v1;
    double distance = rayon*sqrt(1+1/pow(tan(acos(cosdelta)),2.));
//=====>LA
    //ROS_INFO("dist = %f",distance);

    //With v3 and distance, get 3D position in camera referential
    Eigen::Vector4f position3D_rob;
    Eigen::Vector4f position3D_cam;
    position3D_cam << distance*v3(0),distance*v3(1),distance*v3(2),1;
    //position3D_cam << 0,0,0,1;//test showing that the center of the ref2 is on the ground


    //get matrix transform to the robot referential
    int space = 0; //FRAME_TORSO
    //we go in frame_robot but may try to go in frame torso because easier for the arms
    bool useSensorValues = true;
    std::string cameraName = "CameraTop";
    std::vector<float> transVec = motion_proxy_ptr->getTransform(cameraName, space, useSensorValues);
    Eigen::Matrix4f trans;
        trans <<
            transVec[0] , transVec[1] , transVec[2] , transVec[3] ,
            transVec[4] , transVec[5] , transVec[6] , transVec[7] ,
            transVec[8] , transVec[9] , transVec[10], transVec[11],
            transVec[12], transVec[13], transVec[14], transVec[15];

    position3D_rob=trans*position3D_cam;

    //renormalization of the vector
    position3D_rob/=position3D_rob(3);

//====>LA
    //ROS_INFO("fini = %f",5.);

    //the final vector is
    Eigen::Vector3f position3D;
    position3D << position3D_rob(0),position3D_rob(1),position3D_rob(2);
    return position3D;
    }
    //////////

    //method to get the ball in low resolution with RGB
    Mat lowResBallFindingRGB(Mat img_cam,double R, double G, double B,
            double thresColor,int nbSubsampling){

        Mat subsampledImage;
        subsampledImage = subsampleMulti(img_cam, nbSubsampling);


        for (int i = 0; i < subsampledImage.rows; ++i) {
            for (int j = 0; j < subsampledImage.cols; ++j) {
                subsampledImage.at<uchar>(Point(j, i))=
                        thresCondOKRGB(subsampledImage,i,j,R,G,B,thresColor);
            }
        }
        return subsampledImage;
    }
    //////////

    //method to get the ball in low resolution with HSV
    Mat lowResBallFindingHSV(Mat img_cam,int HMIN,int HMAX,int SMIN,int SMAX,int VMIN,int VMAX,
            int nbSubsampling){

        Mat imHSV;
        cvtColor(img_cam, imHSV, CV_BGR2HSV);

        Mat subsampledImage;
        subsampledImage = subsampleMulti(imHSV, nbSubsampling);


        Mat im(subsampledImage.rows, subsampledImage.cols, CV_8UC1, Scalar(0));

        for (int i = 0; i < subsampledImage.rows; ++i) {
            for (int j = 0; j < subsampledImage.cols; ++j) {
                im.at<uchar>(Point(j, i))=
                        255*thresCondOKHSV(subsampledImage,i,j,HMIN,HMAX,SMIN,SMAX,VMIN,VMAX);
            }
        }
        //publishImage(zoom2(zoom2(im)),WINDOW);
        return im;
    }
    //////////

    //method to tell if the ball is in the screen and where it is
    int isBallInScreen(Mat lowRes,double thresBallPresence){

        double wherex=0;
        double wherey=0;
        double sum=0;

        for (int i = 0; i < lowRes.rows; ++i) {
            for (int j = 0; j < lowRes.cols; ++j) {
                if(lowRes.at<uchar>(Point(j, i))==255){
                    sum+=1;
                    wherex+=i;
                    wherey+=j;
                }
            }
        }
        //ROS_INFO("sum = %f",sum);
        wherex/=sum;
        wherey/=sum;
        //ROS_INFO("X = %f",wherex);
        //ROS_INFO("Y = %f",wherey);

        if (sum>thresBallPresence){
            if (wherex>2*lowRes.rows/3){
                return 3;
            }
            if (wherex<1*lowRes.rows/3){
                return 1;
            }
            if (wherey<1*lowRes.rows/3){
                return 4;
            }
            if (wherey>2*lowRes.rows/3){
                return 2;
            }
            return 5;
        }
        else return 0;

        //0 to say no ball
        //1 to turn up
        //2 to right
        //3 to bottom
        //4 to left
        //5 in the center
    }
    //////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //METHODS TO PLAY WITH IMAGE AND MATRIX

    //Method to draw matrix for given list
    Mat drawMatFromList (vector<vector<int> > contours,Mat contoursImage){
        Mat im(contoursImage.rows, contoursImage.cols, CV_8UC1, Scalar(0));
        for (int i=0;i<contours.size();i++){
            int y=contours.at(i).at(1);
            int x=contours.at(i).at(0);
            im.at<uchar>(Point(y,x))=255;
        }
        return im;
    }
    ///////////

    //Method to sends back a 2*zoomed image
    Mat zoom2(Mat img){
        Mat im(2*img.rows, 2*img.cols, CV_8UC1, Scalar(0));
        for(int i=0;i<img.rows;i++){
            for(int j=0;j<img.cols;j++){
                im.at<uchar>(Point(2*j, 2*i))=img.at<uchar>(Point(j, i));
                im.at<uchar>(Point(2*j+1, 2*i))=img.at<uchar>(Point(j, i));
                im.at<uchar>(Point(2*j, 2*i+1))=img.at<uchar>(Point(j, i));
                im.at<uchar>(Point(2*j+1, 2*i+1))=img.at<uchar>(Point(j, i));
            }
        }
        return im;
    }
    //////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SERVERS

    //Method to send back the ball position for head
    bool ballPosForHead(nao_behavior_tree::BallPosForHead::Request &req,
            nao_behavior_tree::BallPosForHead::Response &res){
        ROS_INFO("FIND HEAD");
        //finds ball in low res
        Mat lowResBall = lowResBallFindingHSV(imgcam,req.min_h,req.max_h,80,255,80,255,3);
        //publishImage(zoom2(zoom2(lowResBall)),WINDOW2);
        sleep(2);
        res.pos = isBallInScreen(lowResBall,30);
        return true;
    }

    //Method to send back the ball position for hand
    bool ballPosForHand(nao_behavior_tree::BallPosForHand::Request &req,
            nao_behavior_tree::BallPosForHand::Response &res){

        Mat color = colorDetectionHSV(imgcam, req.min_h,req.max_h,80,255,80,255);
        //get the chain of the ball
        Mat grad=getContourBall(imgcam, req.min_h,req.max_h,80,255,80,255);
        //vector<vector<vector<int> > > chains=getChains2(getContourBall3(imgcam, req.min_h,req.max_h,120,255,120,255));
        vector<vector<vector<int> > > chains=getChains2(grad);
        vector<vector<vector<int> > >  meltedchainlist =meltChains(chains,meltingList);
        vector<vector<int> > chain=chooseChain(meltedchainlist,1,imgcam);

        publishImage(grad,WINDOW);
        publishImage(drawChains(meltedchainlist,imgcam),WINDOW2);
        publishImage(drawMatFromList(chain,imgcam),WINDOW3);
        publishImage(color,WINDOW4);

        //get info about the ball
        vector <double> ballinfo; ballinfo = ballInfo(chain);
        Eigen::Vector3f coord=get3DcoordBall(ballinfo);
        res.pos_x=coord(0);
        res.pos_y=coord(1);
        res.pos_z=coord(2);

        //ROS_INFO("x = %f",coord(0));
        //ROS_INFO("y = %f",coord(1));
        //ROS_INFO("z = %f",coord(2));


        return true;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};





int main(int argc, char** argv) {
    if(argc != 1)
{
        // Robot selection
        if(atoi(argv[1]) == 1) {NAO_ID = "1";}
        if(atoi(argv[1]) == 2) {NAO_ID = "2";}

    ros::init(argc, argv, "image_converter" + NAO_ID);
    ros::NodeHandle pnh("~");

    // Robot parameters
    std::string NAO_IP;
    int NAO_PORT;
    pnh.param("NAO_IP",NAO_IP,std::string("127.0.0.1"));
    pnh.param("NAO_PORT",NAO_PORT,int(9559));




//create proxy to get the position of the head
    //proxy no nao
    //motion_proxy_ptr = new AL::ALMotionProxy("127.0.0.1",9559);
    //proxy with nao
    motion_proxy_ptr = new AL::ALMotionProxy(NAO_IP,NAO_PORT);


    ImageConverter ic;
    ros::spin();
}
    return 0;
}
