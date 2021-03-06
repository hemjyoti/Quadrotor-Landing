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

cv::Mat img;
cv_bridge::CvImagePtr cv_ptr;
ros::Publisher Num_pub;
ardrone_test::Drone_odo val;
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
image_transport::Publisher image_pub_;

void getRedCircles(const cv::Mat& inputImage, cv::Mat& outputImage);
void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage);


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{

			cv_ptr = cv_bridge::toCvCopy(msg, "8UC3");

	}
  	catch (cv_bridge::Exception& e)
     	{
  			ROS_ERROR("cv_bridge exception: %s", e.what());
  		    return;
     	}

  	img = cv_ptr->image;
  	cv::Mat outputImage;
  	callFilter(cv_ptr->image, outputImage);

  	cv::imshow("view", cv_ptr->image);

  	cv::waitKey(3);

  //	float d_val = (float) cv_ptr->image.at<uint16_t>(msg->height/2,msg->width/2);
  //	ROS_INFO("depth = %f",d_val/1000 );
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     cv::namedWindow("view");
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
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

