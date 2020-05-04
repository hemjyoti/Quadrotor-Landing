#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/navdata_gps.h"    //Accessing ardrone's published data
#include "ardrone_autonomy/vector31.h"
#include "ardrone_autonomy/vector21.h"
#include <termios.h>
#include <sstream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;		// ardrone navdata subsciber
ros::Subscriber est_sub;		// ardrone navdata subsciber
ros::Subscriber num_sub;

ros::Publisher est_pub;
ardrone_test::est_co estdata;

ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;

float angx,angy,angz, est_x, est_y, est_z, pos_x, pos_y,pos_z;
int redX, redY;

double lat, lon, ele, ang_time,dt, tp;

double x=0,y=0,z=0,Vx0=0,Vy0=0;
int k_flag=0;

void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_test::est_co::ConstPtr& est);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data

//using namespace cv;
using namespace std;



int main( int argc, char **argv)
{
	ros::init(argc, argv, "kalman_estimation");
	ros::NodeHandle ni;

	pose_subscriber = ni.subscribe("/ardrone/navdata", 200, poseCallback);
	imu_subscriber = ni.subscribe("/ardrone/imu", 200, imuCallback);
	gps_subscriber = ni.subscribe("/ardrone/navdata_gps", 10, gpsCallback);
	est_sub = ni.subscribe("Coordinate", 50, EstCallback);
	num_sub = ni.subscribe("/chatter1", 100, NumCallback);

	est_pub = ni.advertise<ardrone_test::est_co>("estimation_data", 1000);

	ros::spin();
	return 0;

}

void EstCallback(const ardrone_test::est_co::ConstPtr& est)
{	////kinect data cb

  //cout<<"Target:"<< val->x<< val->y;
	est_x = -est->z+0.35;
	est_y = est->y;
	est_z = est->x;
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;

	if(k_flag==0)
	{
	x = est_x;
	y = est_y;
	z = est_z;
//	k_flag=1;

	ROS_INFO("est_data = %lf,%lf",x,y);
	estdata.x = x;
	estdata.y = y;
	estdata.z = z;
	est_pub.publish(estdata);
	}
}


void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{   ////INS data callback


	drone_navdata.vx	= 	pose_message->vx*0.001;
	drone_navdata.vy 	= 	pose_message->vy*0.001;
	drone_navdata.vz 	= 	pose_message->vz*0.001;
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay 	= 	pose_message->ay;
	drone_navdata.az 	= 	pose_message->az;
	drone_navdata.rotX	= 	pose_message->rotX;
	drone_navdata.rotY	= 	pose_message->rotY;
	drone_navdata.rotZ	= 	pose_message->rotZ;
	drone_navdata.magX	= 	pose_message->magX;
	drone_navdata.magY	= 	pose_message->magY;
	drone_navdata.magZ	= 	pose_message->magZ;
	drone_navdata.altd 	= 	pose_message->altd*0.001;
	dt = (pose_message->tm-tp)*0.000001;
	drone_navdata.tm	= 	pose_message->tm;
	drone_navdata.header	= 	pose_message->header;
	tp = drone_navdata.tm;

//	if(k_flag ==1)
	{
		x = x + ((drone_navdata.vx+Vx0)/2)*dt;
		y = y + ((drone_navdata.vy+Vy0)/2)*dt;
		z = drone_navdata.altd;

		ROS_INFO("est_data = %lf,%lf",x,y);
		estdata.x = x;
		estdata.y = y;
		estdata.z = z;
		est_pub.publish(estdata);

	}

	Vx0=drone_navdata.vx;
	Vy0=drone_navdata.vy;


}
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message)
{
	ang.header	= 	imu_message->header;
	ang.angular_velocity	= 	imu_message->angular_velocity;
	}

void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message)
{
	lat		= 	gps_message->latitude;
	lon 	= 	gps_message->longitude;
	ele 	= 	gps_message->elevation;
}

void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val)
{

	//cout<<"TaRgOt:"<< val->x<< val->y;
	redX = val->x;
	redY = val->y;
	if((redX==0) && (redY==0))
	k_flag=1;
	else
	k_flag=0;

}



