#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_test/Drone_odo.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv/cv.hpp"
#include "opencv2/core/version.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>


//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif


using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;

cv::Mat color_img,depth_img;
cv_bridge::CvImagePtr img_ptr_color;
cv_bridge::CvImagePtr img_ptr_depth;
ros::Publisher Num_pub;
ardrone_test::Drone_odo val;
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
image_transport::Publisher image_pub_;
float x_m,y_m,z_m;

void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage);
void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage);


void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{

	try
	{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
	}
	catch (cv_bridge::Exception& e)
	{
	        ROS_ERROR("cv_bridge exception:  %s", e.what());
	        return;
	}

	try
	{
        img_ptr_color = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
	}
  	catch (cv_bridge::Exception& e)
    {
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		    return;
    }

  	color_img = img_ptr_color->image;

  	cv::Mat outputImage;
  	callFilter(color_img, outputImage);

  	cv::imshow("view", color_img);

  	cv::waitKey(3);

 	depth_img = img_ptr_depth->image;

 	float depth_val=0;
///*
 	for(int i=-10; i<10; i++)
 	{
 		for(int j=-10; j<10; j++)
 		{
 			depth_val = depth_val + (float) depth_img.at<uint16_t>(val.x + i, val.y + j);
 		}
 	}

 	depth_val = depth_val/200;  //in mm (taking avg of 100)
//*/

 	//depth_val = (float) depth_img.at<uint16_t>(val.x , val.y );

 	x_m = depth_val/1000;       //in metres
 	y_m = (val.y-480)*x_m*0.001459;
 	z_m = (val.x-270)*x_m*0.002138;

 	//	z_m = depth_val/1000;       //in metres
 	//	x_m = (val.y-480)*x_m*0.001459;
 	// 	y_m = (val.x-270)*x_m*0.002138;


 	ROS_INFO("The real world co-ordinates of the centroid = %f, %f, %f",x_m,y_m,z_m );
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     cv::namedWindow("view");
     cv::startWindowThread();

   /*  image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub_color = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
     image_transport::Subscriber sub_color = it.subscribe("/kinect2/qhd/image_depth_rect", 1, imageCallback);
    */

     message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/kinect2/qhd/image_depth_rect" , 1 );
     message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/kinect2/qhd/image_color_rect" , 1 );


   #ifdef EXACT
       typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
   #endif
   #ifdef APPROXIMATE
       typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
   #endif


     // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
     sync.registerCallback(boost::bind(&callback, _1, _2));


     ros::spin();

     //Num_pub = nh_.advertise<ardrone_test::Drone_odo>("chatter1", 1000);

     cv::destroyWindow("view");
}


void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage)
{

	getRedCircles(inputImage, outputImage);

}


void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage)
{
	cv::Mat img_hsv;
	int iLastX = -1;
	int iLastY = -1;

	//Create a black image with the size as the camera output
	 cv::Mat imgLines = cv::Mat::zeros( img_hsv.size(), CV_8UC3 );


	cvtColor(inputImage,img_hsv,CV_BGR2HSV);
    cv::Mat img_hsv_upper;
    cv::Mat img_hsv_lower;

	cv::inRange(img_hsv,cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_hsv_lower);
	imshow("HSV LOWER", img_hsv_lower); //show the img_hsv_lower

	cv::inRange(img_hsv,cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_hsv_upper);
	imshow("HSV UPPER", img_hsv_upper); //img_hsv_upper

	cv::Mat img_hsv_red;
	cv::addWeighted(img_hsv_lower, 1.0, img_hsv_upper, 1.0, 0.0, img_hsv_red);

	cv::GaussianBlur(img_hsv_red, img_hsv_red, cv::Size(9,9), 2,2);

	cv::Mat imgThresholded = img_hsv_red;
    //cv::Mat img_hsv_lower;

	//morphological opening (removes small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );

	//morphological closing (removes small holes from the foreground)
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))  );


	 cv::Moments oMoments = cv::moments(imgThresholded);

	  double dM01 = oMoments.m01;
	  double dM10 = oMoments.m10;
	  double dArea = oMoments.m00;

	  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
	  	  if (dArea > 10000)
	  	  {
	  	   //calculate the position of the ball
	  	   int posX = dM10 / dArea;
	  	   int posY = dM01 / dArea;

	  	   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
	  	   {
	  	    //Draw a red line from the previous point to the current point
	  	    cv::line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0,0,255), 2);
	  	   }

		   iLastX = posX;
		   iLastY = posY;
		  }



		  imshow("Thresholded Image", imgThresholded); //show the thresholded image

		  img_hsv = img_hsv + imgLines;
		  //imshow("Original", img_hsv); //show the original image

		ROS_INFO("got the image = ");


	   	ROS_INFO("%d, %d", iLastX, iLastY);

     outputImage = img_hsv;
  	 val.x = iLastY;
  	 val.y = iLastX;
//  	 ROS_INFO("%f", val.data);
  	 Num_pub.publish(val);

}

