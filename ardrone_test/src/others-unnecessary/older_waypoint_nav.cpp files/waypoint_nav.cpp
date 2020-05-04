﻿#include "ros/ros.h"
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
#include "ardrone_autonomy/SetGPSTarget.h" // For setting gps target for gps waypoint navigation built into ardrone_autonomy
#include "ardrone_autonomy/RecordEnable.h" // For starting gps waypoint navigation built into ardrone_autonomy
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_autonomy/vector31.h"
#include "ardrone_autonomy/vector21.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include "ardrone_autonomy/navdata_gps.h"    //Accessing ardrone's published data
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"
#include "tum_ardrone/filter_state.h"

// class variables
std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class

// Variables for Publishing
ros::Publisher T_pub_empty;				//take off publisher
ros::Publisher L_pub_empty;				//landing publisher
ros::Publisher velocity_publisher;		// velocity publisher

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;		// ardrone navdata subsciber
ros::Subscriber est_sub;		// ardrone navdata subsciber
ros::Subscriber INS_K_est;		// ardrone navdata subsciber
ros::Subscriber joy_sub_;
ros::Subscriber num_sub;
ros::Subscriber new_gps_sub;
ros::Subscriber test_sub;
ros::Subscriber odo_subscriber;
ros::Subscriber tum_pose;
//ros::Subscriber filter_subscriber;		// tum estimation subscriber

// Variables for Service
ros::ServiceClient client1;		// ardrone camera service
ros::ServiceClient client;		// ardrone setgpstarget service
ros::ServiceClient client2;		// ardrone setautoflight service

using namespace std;

float lx, ly, lz, ax, ay, az,foc = 685, angx,angy,angz, est_x, est_y, est_z,K_est_x, K_est_y, K_est_z,test_x,test_y,test_z;
int to, la, f = 200 , redX, redY, res =0, H=0,I=0,J=0;
double k = 0.51, z, tf, ang_time,hem; // k value for tau
double lat, lon, ele;
//gps variables///
double new_lat, new_lon;
bool gps_cntr = 0;
//static float pos_x = 0, pos_y = 0, pos_z = 0;
const double PI = 3.14159265359;
double Kp = 0.2, Kd = 1,Kpz;// Gain for the controller
double k1 = 2.438, k2 = 0.779, k3 = 1.234, k4 = 0.221;// Gain for the controller
double tol = 0.4;// error tolerance
int kinect_ready=0;

//class instances
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;
nav_msgs::Odometry odo;
tum_ardrone::filter_state posisi;
ardrone_autonomy::SetGPSTarget gps_srv; //class instance for setgpstarget service
ardrone_autonomy::RecordEnable auto_srv; //class instance for setautoflight service

//callback function declarations
void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_test::est_co::ConstPtr& est);
void INS_K_Callback(const ardrone_test::est_co::ConstPtr& esti);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void new_gpsCallback(const std_msgs::String::ConstPtr& gps1);
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data
void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);		// Joy data
void record();
void testdataCallback(const ardrone_autonomy::vector31::ConstPtr & test );
void hem_odo(const nav_msgs::Odometry::ConstPtr & nav_message);
void tum_posecallback(const tum_ardrone::filter_state::ConstPtr & posi);
// basic func
void hover();		// hovering
void takeoff();		// take off
void land();		//landing
void move(float lx, float ly, float lz, float ax, float ay, float az ); // publishing command velocity
// Basic Navigation func
void yawing(double angle);				// yawing movements

//Advanced Navigation function
void moveadv(double x_d, double y_d, double z_d);
void moveto(double x_d, double y_d, double z_d);
void moveto1(double x_d, double y_d, double z_d);//with kinect addition to moveto
void moveto_GpsWaypointBased(double x_d, double y_d, double z_d);// using gps waypoint service in ardrone_autonomy
void k_move(double x_d, double y_d, double z_d);
void k_hold();
void moveup(double z_d);
void validate(double x_d, double y_d, double z_d);
void tau(double x_d, double y_d, double z0);
void tau1(double x_d, double y_d, double z0);
void min_jerk(double x_d, double y_d, double z0, double tf);
//void itau(double x_d, double y_d, double z0);
void onTarget(int redX, int redY);
// Miscellaneous func
int getch();								//similar to getting input
double deg2rad(double angle_in_degrees);	//degree to radian converter
double getDistance(double x1, double y1, double z1, double x2, double y2, double z2);
void compare_tum_position_vals();
void hemdu(double xdd,double ydd,double zdd);void gps_odom_ekf();
void hem_gps();
void imu_ekf();
void imu_eqns();
//void hem_gps_2(int &x, int &y);



int main(int argc, char **argv)
{
	//Initiate the ROS

	ros::init(argc, argv, "waypuoint_nav");
	ros::NodeHandle n; 			// Nodehandle is like a class where you are operating

	// Publishing the data
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100); // initialize to send the Control Input

	T_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);  //initialize to send the take off command  /* Message queue length is just 1 */

	L_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/land", 1); //initialize to send the land command /* Message queue length is just 1 */

	// Subscribing the data
	pose_subscriber = n.subscribe("/ardrone/navdata", 200, poseCallback);	//initialize to receive processed sensor data
	imu_subscriber = n.subscribe("/ardrone/imu", 200, imuCallback); //initialize to receive raw sensor data
	gps_subscriber = n.subscribe("/ardrone/navdata_gps", 10, gpsCallback); //initialize to receive gps data
	est_sub = n.subscribe("Coordinate", 100, EstCallback); //initialize to receive ground cam target data
	INS_K_est = n.subscribe("/estimation_data", 200, INS_K_Callback); //initialize to receive kinect+ uav sensor estimation sensor data
	test_sub = n.subscribe("/test", 100, testdataCallback);
    odo_subscriber = n.subscribe("/ardrone/odometry", 100, hem_odo);
	new_gps_sub = n.subscribe("/uc0Response", 10, new_gpsCallback);

	num_sub = n.subscribe("/chatter1", 100, NumCallback);
	tum_pose = n.subscribe("/ardrone/predictedPose", 100,tum_posecallback );
	joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);

	// Calling toggle cam service
/*
	client1 = n.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel", 1);

	ardrone_autonomy::CamSelect srv;

	srv.request.channel = 1;
	if (client1.call(srv))
	{
		ROS_INFO("Called");
	}
	else
	{
		ROS_INFO("Nope");
	}
*/
	// Operations the actual main function after ros initialization
while(ros::ok())
{

	cout<<"Press any key"<<endl;
    cout<<"t -> takeoff"<<endl;
    cout<<"l -> land"<<endl;
    cout<<"n -> Record data"<<endl;
    cout<<"y -> yaw"<<endl;
    cout<<"h -> hover"<<endl;
    cout<<"q -> Go to a coordinate"<<endl;
    cout<<"j -> Min_jerk"<<endl<<endl;

    //Hemjyothi and Kaustubhs functions
    cout<<"b -> Go to a coordinate using gps waypoint(setgpstarget) service in ardrone_autonomy"<<endl;
    cout<<"r -> imu ekf velocity estimator(and integrated to find distance drone thinhs has travelled)"<<endl;
    cout<<"o -> compare position from tum_ardrone packages drone_stateestimation node w/ odometry"<<endl;
    cout<<"w -> kalman gps and odometry"<<endl;
    cout<<"a -> Get gps data (lat,lng,x,y,d)"<<endl;
    cout<<"p -> Tau"<<endl;
    cout<<"i -> Image Tau"<<endl;
    cout<<"v -> Validate"<<endl;
   // ROS_INFO("%d",kinect_ready);spjivhltny

    int c = getch();   // call your non-blocking input function

    double x, y, z;			// for reference value

    switch (c)
    {

    case 'r':
		imu_ekf();
		//imu_eqns();
		break;
    case 'Z':

    	   cout<<drone_navdata.rotZ<<endl;
    	   break;
	case 'o':
		compare_tum_position_vals();
			break;
	case 'a':
		hem_gps();
			break;
	case 'w':
		gps_odom_ekf();
			break;
	case 'k':
	//		cout<<endl<<"z Gain"<<endl;
		//	cin>>Kpz;
			cout<<endl<<"Height"<<endl;
			cin>>z;
			//cout<<endl<<"Co - ordinates"<<endl;
			//cin>>x>>y;
			//cout<<"co or"<<x<<setw(10)<<y<<endl;
			moveup(z);//tau(x,y,z);
			//hover();
			//z = drone_navdata.altd*0.001;

    		cout<<endl<<"Co - ordinates"<<endl;
    		cin>>x>>y;
      		cout<<endl<<"x & y Gain"<<endl;
      		cin>>Kp;
    		cout<<endl<<"Tol"<<endl;
       		cin>>tol;
    //    		Kp=0.1;
    		//z =0;
    		k_move(x,y,0);//tau(x,y,z);
    		//moveadv(-x,0,0);//tau(x,y,z);
    		break;
	case 'f':
	/*			cout<<endl<<"Height"<<endl;
				cin>>z;
				moveup(z);//tau(x,y,z);
				hover();
				//z = drone_navdata.altd*0.001;
	*/
	    		cout<<endl<<"Co - ordinates"<<endl;
	    		x = est_x;
	    		y = est_y;
	    		z = est_z;
	    		cout<<x<<setw(20)<<y<<endl;
	    		//z =0;
	    		//k_hold(x,y,0);//tau(x,y,z);
	    		//moveadv(-x,0,0);//tau(x,y,z);
	    		break;
	case 'b':

			//calling setgpstarget service

			client = n.serviceClient<ardrone_autonomy::SetGPSTarget>("/ardrone/setgpstarget", 10);

			//ardrone_autonomy::SetGPSTarget gps_srv--->class instance gps_srv declared above along with other class instances
			 //gps_srv.request.target.id.uuid = "[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]";
			 gps_srv.request.target.position.latitude = 13.027177;
			 gps_srv.request.target.position.longitude = 77.564008;
			 gps_srv.request.target.position.altitude = 1;
			 gps_srv.request.target.props[0].key = 'velocity';
			 gps_srv.request.target.props[0].value = '0.5';



			if (client.call(gps_srv))
			{
				ROS_INFO("Called setgpstarget");
			}
			else
			{
				ROS_INFO("Nope gps");
			}
			//calling setautoflight service
			client2 = n.serviceClient<ardrone_autonomy::RecordEnable>("/ardrone/setautoflight", 10);
			//ardrone_autonomy::RecordEnable auto_srv--->class instance auto_srv declared above along with other class instances
			auto_srv.request.enable =true;
			if (client.call(auto_srv))
			{
				ROS_INFO("Called setautoflight");
			}
			else
			{
				ROS_INFO("Nope auto");
			}

    		break;

	case 'q':
			cout<<endl<<"Height"<<endl;
			cin>>z;
			moveup(z+0.5);//tau(x,y,z);
			//hover();
			z = drone_navdata.altd*0.001;

			//cout<<endl<<"Go to coordinate"<<est_x<<" "<<est_y<<endl;
    		cout<<"enter the coordinates";
		    cin>>x>>y;
    		//z =0;
    		moveto1(x,y,0);//tau(x,y,z);
    		//hover();

			//cout<<endl<<"Co - ordinates"<<est_x<<setw(20)<<est_y<<setw(20)<<est_z<<endl;
			//hemdu(est_x,est_y,est_z);
    		//tau(x,y,z);
    		//tau(est_x,est_y,est_z);

    		//tau1(2,3,2);


    		//moveadv(-x,0,0);//tau(x,y,z);
    		break;
	case 's':
				cout<<endl<<"UP"<<endl;
			    cin>>z;
			   // moveup(z);
	    		cout<<endl<<"side"<<endl;
	    		cin>>x;
	    		z =0;
	    		moveto(x,0,0);
	    		hover();
	    		moveto(0,x,0);
	    		hover();
	    		moveto(-x,0,0);
	    		hover();
	    		moveto(0,-x,0);
	    		hover();
	    		break;

	case 'p':
				cout<<endl<<"Height"<<endl;
				cin>>z;
				moveup(z);//tau(x,y,z);
				z = drone_navdata.altd*0.001;
				cout<<setw(20)<<z;
				cout<<endl<<"Co - ordinates"<<endl;
		    	cin>>x>>y;
		    	tau1(x,y,z);
		    	break;
	case 'j':
				cout<<endl<<"Height"<<endl;
				cin>>z;
				//moveup(z);//tau(x,y,z);
				z = drone_navdata.altd*0.001;
				cout<<setw(20)<<z;

				cout<<endl<<"Co - ordinates"<<endl;
		    	cin>>x;
		    	cout<<endl<<"Final Time"<<endl;
		    	cin>>tf;
		    	hover();
		    	min_jerk(x,0,z,tf);
		    	break;

	case 'i':
				cout<<endl<<"Height"<<endl;
				cin>>z;
			    //moveup(z);//tau(x,y,z);
				z = drone_navdata.altd*0.001;
				cout<<setw(20)<<z;
				//itau(redX,redY,z);
		    	break;

	case 'v':
    			cout<<endl<<"Co - ordinates"<<endl;
    			cin>>x;
    			//z =0;
    			validate(x,0,0);//tau(x,y,z);
    			//moveto(-x,0,0);//tau(x,y,z);
    			break;

    case 'h':
    			cout<<"Hovering in a place"<<endl;
    			hover();
    			break;
    case 'l':
    			cout<<endl<<"Landing initiated"<<endl;
    			land();
    			break;
    case 't':
    			cout<<endl<<"Take off initiated"<<endl;
    			takeoff();
    			hover();
    			break;
	case 'n':
    			record();
    			break;
    case 'y':
    			cout<<endl<<"Rotation in degree :"<<endl;
    			cin>>x;
    			yawing(x);
    			break;
    default:
    			land();
    			break;

    }
    hover();
}
land();
cout<<"landed";

}

void tum_posecallback(const tum_ardrone::filter_state::ConstPtr & posi)
{
	posisi.x = posi->x;
	posisi.y = posi->y;
	posisi.z = posi->z;
	cout<<posisi.x<<setw(20)<<posisi.y<<setw(20)<<posisi.z<<endl;
}
void imu_eqns()//using INS equations of motion
{
	ros::Rate loop_rate(200);//loop 200 hz
	ofstream myfile;
	myfile.open("/home/icgel/icgel7/MotionEqnsEKF.txt", ios::out | ios::app );
	myfile<<"odom_vx odom_vy eqn_vx eqn_vy x_eqn y_eqn x_odom y_odom"<<endl;
	double i=0,f=200.0,d2r=3.14159265/180.0;
	double x=0,y=0,z=0,xo=0,yo=0,zo=0;
	double u=0,v=0,w=0,p,q,r,theta=0,phi=0,psi=0,u_dot,v_dot,w_dot,phi_dot,theta_dot,psi_dot,Vxo,Vyo;
	//initial vals for attitude
	theta=(drone_navdata.rotY)*d2r;phi=(drone_navdata.rotX)*d2r;psi=(drone_navdata.rotZ)*d2r;

	do{ //variables initialization for this loop
		move(0.1,0,0,0,0,0);
		p=ang.angular_velocity.x;q=ang.angular_velocity.y;r=ang.angular_velocity.z;
		//equations of motion
		u_dot = drone_navdata.ax*9.81 - q*w + r*v - 9.81*sin(theta);
		v_dot = drone_navdata.ay*9.81 - r*u + p*w - 9.81*cos(theta)*sin(phi);
		w_dot = drone_navdata.az*9.81 - p*v + q*u - 9.81*cos(theta)*cos(phi);
		phi_dot = p + tan(theta)*(q*sin(phi)+r*cos(phi));
		theta_dot = q*cos(phi) - r*sin(phi);
		psi_dot = (q*sin(phi)+r*cos(phi))/cos(theta);
		//integrating
		u = u + u_dot/f;
		v = v + v_dot/f;
		w = w + w_dot/f;
		phi = phi + phi_dot/f;
		theta = theta + theta_dot/f;
		psi = psi + psi_dot/f;
		//distance
		x = x + u/f;
		y = y + v/f;
		z = z + w/f;
		//odometry distance
		Vxo = drone_navdata.vx*0.001;
		Vyo = drone_navdata.vy*0.001;
		xo = xo + Vxo/f;
		yo = yo + Vyo/f;
		myfile<<Vxo<<setw(20)<<Vyo<<setw(20)<<u<<setw(20)<<v<<setw(20)<<x<<setw(20)<<y<<setw(20)<<xo<<setw(20)<<yo<<endl;
		ros::spinOnce();
		loop_rate.sleep();
		i=i+1;
	}while(i<1500);
	cout<<"land";
	land();
	myfile.close();
}


void imu_ekf()
{
double i=0,d2r = 3.14159265/180, theta0=(drone_navdata.rotX)*d2r,phi0=(drone_navdata.rotY)*d2r;
//initialize files
ofstream myfile;
myfile.open("/home/icgel/icgel7/vel_ekf_vals.txt", ios::out | ios::app );
myfile<<"odom_vx odom_vy ekf_vx ekf_vy x_ekf y_ekf x_odom y_odom xf yf"<<endl;

double Vx=0,Vy=0,xo = 0,yo = 0, x=0,y=0,Vx0=0,Vy0=0,c=0.15,f=200.0,T=1/200.0;
double Vx_dot,Vy_dot,Vx_dot0=0,Vy_dot0=0;
double phi_dot,theta_dot,phi_dot0 = 0, theta_dot0 = 0;
double theta=(drone_navdata.rotX)*d2r,phi=(drone_navdata.rotY)*d2r;
double p00=1,p11=1,a00=1-c*T,a11=1-c*T;
double q00=0.0001,q11=0.0001;//obtained from analyzing values when hovering at a place
double r00=0.0001,r11=0.0001;
double c00=-c,c11=-c,k00=0,k11=0;
double Vxo,Vyo,Vxo_i=0,Vyo_i=0;
ros::Rate loop_rate(200);//loop 200 hz

	//loop begins here
	do{

	i=i+1;
	move(0.1,0,0,0,0,0);

	double ax=drone_navdata.ax*9.81,ay=drone_navdata.ay*9.81;
	//double ax = ang.linear_acceleration.x, ay = ang.linear_acceleration.y;
	Vxo = drone_navdata.vx*0.001;
	Vyo = drone_navdata.vy*0.001; //odometry velocities
	myfile<<Vxo<<setw(20)<<Vyo<<setw(20);
	//double gx=ang.angular_velocity.x,gy=ang.angular_velocity.y,gz=ang.angular_velocity.z;

	double theta=(drone_navdata.rotY)*d2r,phi=(drone_navdata.rotX)*d2r;

	//predict equations

	//phi_dot = gx + tan(theta-theta0)*cos(phi-phi0)*gz + tan(theta-theta0)*sin(phi-phi0)*gy;
	//theta_dot = cos(phi-phi0)*gy - sin(phi-phi0)*gz;

	Vx_dot = -9.81*sin(theta-theta0)-c*Vx;
	Vy_dot = 9.81*cos(theta-theta0)*sin(phi-phi0)-c*Vy;

	p00=p00+ (q00+2*a00*p00)/f;
	p11=p11+ (q11+2*a11*p11)/f;

	//kalman gain
	k00=(c00*p00)/(p00*c00*c00+r00);
	k11=(c11*p11)/(p11*c11*c11+r11);

	//update equations!
	Vx_dot=Vx_dot+k00*(ax - (-c*Vx));
	Vy_dot=Vy_dot+k11*(ay - (-c*Vy));
	//USING SECOND ORDER RUNGE KUTTA TO OBTAIN VX,VY FROM ABOVE Vx_dot and Vy_dot
	Vx = Vx + 0.5*(T*Vx_dot+T*Vx_dot0);
	Vy = Vy + 0.5*(T*Vy_dot+T*Vy_dot0);
	Vx_dot0 = Vx_dot;
	Vy_dot0 = Vy_dot;

	//theta = theta + 0.5*(T*theta_dot+T*theta_dot0);
	//phi = phi + 0.5*(T*phi_dot+T*phi_dot0);
	//theta_dot0 = theta_dot;
	//phi_dot0 = phi_dot;

	p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
	p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
	//save Vx,Vy obtained from above EKF
	myfile<<Vx<<setw(20)<<Vy<<setw(20);
	//obtain distance through trapezoidal integration of ekf velocity estimate
	x = x + 0.5*(Vx+Vx0)/f;
	y = y + 0.5*(Vy+Vy0)/f;
	Vx0=Vx;
	Vy0=Vy;
	myfile<<x<<setw(20)<<y<<setw(20);
	//obtain distance on integrating odometry velocities
	//xo = xo + 0.5*(Vxo+Vxo_i)/f;
	//yo = yo + 0.5*(Vyo+Vyo_i)/f;
	xo = xo + Vxo/f;
	yo = yo + Vyo/f;
	Vxo_i=Vxo;
	Vyo_i=Vyo;
	myfile<<xo<<setw(20)<<yo<<setw(20);
	//complimentary filter
	myfile<<(xo*0.5 + x*0.5)<<setw(20)<<(yo*0.5 + y*0.5)<<endl;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();
	cout<<i<<endl;
	}while(i<1500);

cout<<"land";
land();
myfile.close();
}


void compare_tum_position_vals()
{
	double xc=0,yc=0,zc=0,Vx,Vy,dtt=0.1;
	ros::Rate loop_rate(10);//loop 50 hz
	ofstream myfile,myfile2;
	myfile.open("/home/icgel/icgel7/cmp_tum_values.txt", ios::out | ios::app );
	myfile2.open("/home/icgel/icgel7/cmp_odom_values.txt", ios::out | ios::app );
	int i=0;
	do
	{
	//obtain current velocity values
	Vx=drone_navdata.vx*0.001;
	Vy=drone_navdata.vy*0.001;
	//obtain current x and y values by integrating above
	xc=xc+Vx*dtt;
	yc=yc+Vy*dtt;
	zc=drone_navdata.altd*0.001;
	//save current coordinates to file 5
	myfile2<<xc<<setw(20)<<yc<<setw(20)<<zc<<endl;
    //save current position values obtained from tum_ardrone package
	myfile<<posisi.x<<setw(20)<<posisi.y<<setw(20)<<posisi.z<<endl;
	//printing cause i like it!
	cout<<posisi.x<<setw(20)<<posisi.y<<setw(20)<<posisi.z<<endl;
	i=i+1;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
	}while(i<5000);
myfile.close();
}

void hem_gps()
{	ofstream myfile;
	float d2r =3.14159265/180;
	double r = 6371;
	double lat1,lat2, lng1, lng2, dlat, dlng, a, c, d, l, X=0, Y=0;
	myfile.open("/home/kaustubh/summer_2017/value_texts/gps_run1.txt", ios::out | ios::app );
	for(int i=0;i<10;i++){	
	ros::Rate loop_rate(5);//loop 50 hz	
	lat1=lat*d2r;
	lng1=lon*d2r;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over 
	}//since gps send initial 2 lat long values as 0s
        myfile<<"lat1|lon1"<<lat1<<setw(20)<<lng1<<endl;
	do{
		ros::Rate loop_rate(5);//loop 5 hz
		lat2=lat*d2r;
		lng2=lon*d2r;
		dlat=lat2-lat1;
		dlng=lng2-lng1;
		a=sin(dlat/2)*sin(dlat/2)+ cos(lat1)*cos(lat2)*sin(dlng/2)*sin(dlng/2);
		c=2*asin(sqrt(a));
		d=r*c;
		l=atan2(dlng,dlat);
		X = d*cos(l);
		Y = d*sin(l);
		cout<<lat<<setw(20)<<lon<<setw(20)<<X<<setw(20)<<Y<<setw(20)<<d<<endl;
                myfile<<"lat2|lng2|X|Y|d"<<lat<<setw(20)<<lon<<setw(20)<<X<<setw(20)<<Y<<setw(20)<<d<<endl;
	        //cout<<lat-lon<<endl;
		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over 
	}
	while(2>1);
myfile.close();
}
//-------------------------------------------------------------------------------------------------------------------------------------------
void gps_odom_ekf()
{	
	ros::Rate loop_rate(200);//loop 5 hz
	int j=0,i=0;
	float d2r =3.14159265/180;
	double r = 6371000;
	double lat1,lat2, lng1, lng2, dlat, dlng, a, c, d, l;
    	double initX,initY,initZ;
	double Vx=0,Vy=0,Vz=0;
	double dtt;
	double tp = ros::Time::now().toSec();
	double p00=1,p11=1,p22=1;
	double q00=0.1,q11=10,q22=0.09;//obtained from analyzing values when hovering at a place
	double r00=0.1,r11=0.1,r22=0.1;
	double c00=1,c11=1,c22=1;
	double a00=1,a11=1,a22=1;
	double k00,k11,k22,x_gps,y_gps,z_gps;
	//intial values of position are here as 0,0,takeoff height
	initX=0;
	initY=0;
	initZ=drone_navdata.altd*0.001;//takeoff height
	//x - kalman filtered value, xc - x_current, xp - x_previous ( initialising all below )
	double x=initX,y=initY,z=initZ;
	double xc=initX,yc=initY,zc=initZ;
	double xp=initX,yp=initY,zp=initZ;
	//destination position values can be specified here
	double xdes=2,ydes=2,zdes=1;
	double errorx,errory,errorz,totalt=0;
	//opening files
	ofstream myfile4;
	ofstream myfile5;
	ofstream myfile6;
	ofstream myfile3;
	myfile4.open("/home/icgel/icgel7/gps_kalmann4.txt", ios::out | ios::app );
    myfile5.open("/home/icgel/icgel7/gps_kalmann5.txt", ios::out | ios::app );
    myfile6.open("/home/icgel/icgel7/gps_kalmann6.txt", ios::out | ios::app );
	myfile4<<"kalman"<<endl;
	myfile5<<"odometry"<<endl;
	myfile6<<"gps"<<endl;
	//finding initial latitude and longitude
	for(int i=0;i<10;i++){
	ros::Rate loop_rate(200);//loop 50 hz
	lat1=lat*d2r;
	lng1=lon*d2r;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
	}//since gps send initial 2 lat long values as 0s

	//-----------------------------loop begins here-----------------------------
	do{
	//obtain current gps values
	ros::Rate loop_rate(200);//loop 5 hz
	lat2=lat*d2r;
	lng2=lon*d2r;
	dlat=lat2-lat1;
	dlng=lng2-lng1;
	a=sin(dlat/2)*sin(dlat/2)+ cos(lat1)*cos(lat2)*sin(dlng/2)*sin(dlng/2);
	c=2*asin(sqrt(a));
	d=r*c;
	l=atan2(dlng,dlat);
	y_gps = d*cos(l);//this change in formula comes after observing the values
	x_gps = d*sin(l);
	z_gps = drone_navdata.altd*0.001;
	//obtain time difference between current and previous
        double tc=ros::Time::now().toSec();
        dtt=tc-tp;
	//obtain totat time
        totalt=totalt+dtt;
	//save gps coordinates to file 6
        myfile6<<x_gps<<setw(20)<<y_gps<<setw(20)<<z_gps<<setw(20)<<d<<endl;
	//obtain current velocity values
        Vx=drone_navdata.vx*0.001;
        Vy=drone_navdata.vy*0.001;
	//obtain current x and y values by integrating above
        xc=xc+Vx*dtt;
        yc=yc+Vy*dtt;
        zc=drone_navdata.altd*0.001;
	//save current coordinates to file 5
        myfile5<<xc<<setw(20)<<yc<<setw(20)<<zc<<endl;
	//performing Kalman filter
          x=x+Vx*dtt;
          y=y+Vy*dtt;
          z=drone_navdata.altd*0.001;

          p00=p00+ (q00+2*a00*p00)*dtt;
          p11=p11+ (q11+2*a11*p11)*dtt;
          p22=p22+ (q22+2*a22*p22)*dtt;

          k00=(c00*p00)/(p00*c00*c00+r00);
          k11=(c11*p11)/(p11*c11*c11+r11);
          k22=(c22*p22)/(p22*c22*c22+r22);

          x=x+k00*(x_gps-x);
          y=y+k11*(y_gps-y);
          z=z+k22*(z_gps-z);
          // making it move forward
	  errorx=x-xdes;
          errory=ydes-y;
          errorz=zdes-z;
          //lx=0.1*errorx-0.03*Vx;
          //ly=0.1*errory-0.03*Vy;
          //z=0.35*errorz-0.03*Vz;
          //move(lx,ly,lz,0,0,0);

          p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
          p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
          p00=(k22*k22*r22)+p22*(c22*k22-1)*(c22*k22-1);
	  //savin kalman filtered position values to file 4,3
          myfile4<<x<<setw(20)<<y<<setw(20)<<z<<endl;
	  //printin cause i like it! :P
          cout<<i<<setw(20)<<"Kalman"<<setw(20)<<x<<setw(20)<<y<<endl;

    	 // xp=xc;
    	  tp=tc;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();
    	i=i+1;
	}while(2>1);
	//while(sqrt(pow(errorx,2)+pow(errory,2)+pow(errorz,2))>0.5);
	//(abs(errorx)>0.15 && abs(errory)>0.15 && abs(errorz)>0.15);
	
	cout<<"landing now at total time = "<<setw(20)<<totalt;
	land();

	myfile4.close();
	myfile5.close();
	myfile6.close();
	//ros::spinOnce();
}

void hemdu(double xdd, double ydd, double zdd)
{

	/*double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'
	double t1;
    int iii=1;
	ros::Rate loop_rate(15);
	ofstream myfile3;
	myfile3.open("/home/icgel/icgel2/hem_1A.txt", ios::out | ios::app );
	myfile3<<"Entering the loop"<<endl;
	hover();*/
	ros::Rate loop_rate(200);
    double olaX,olaY,olaZ;

	double Vx=0,Vy=0,Vz=0;

	double dtt;
	double tp = ros::Time::now().toSec();
	double p00=1,p11=1,p22=1;
	double q00=0.0002,q11=0.0002,q22=0.0002;
	double r00=0.15,r11=0.15,r22=0.15;
	double c00=1,c11=1,c22=1;
	double a00=1,a11=1,a22=1;
	double k00,k11,k22;
	olaX=est_x;
	olaY=est_y;
	olaZ=est_z;

	double x=olaX,y=olaY,z=olaZ,xf=olaX,yf=olaY,zf=olaZ;
		double xk,yk,zk;
		double xp=olaX,yp=olaY,zp=olaZ;

	int j=0,i=0;
	ofstream myfile4;
	ofstream myfile5;
	ofstream myfile6;
	double xdes=0.3,ydes=0,zdes=1;
	////ofstream myfile2;
double errorx,errory,errorz;
    myfile4.open("/home/icgel/icgel3/hem.txt", ios::out | ios::app );
    myfile5.open("/home/icgel/icgel3/hem2.txt", ios::out | ios::app );
    myfile6.open("/home/icgel/icgel3/hem3.txt", ios::out | ios::app );
    //myfile2.open("/home/icgel/icgel3/hem2.txt", ios::out | ios::app );
	myfile4<<"kalman"<<endl;
	myfile5<<"odo"<<endl;
	myfile6<<"kinect"<<endl;
	//double tp=0;
 double totalt=0;

	do{

        double tc=ros::Time::now().toSec();
        dtt=tc-tp;
        totalt=totalt+dtt;
        xk=est_x;
        yk=est_y;
        zk=est_z;
        myfile6<<xk<<setw(20)<<yk<<setw(20)<<zk<<endl;
        Vx=drone_navdata.vx*0.001;
        Vy=drone_navdata.vy*0.001;
        Vz=drone_navdata.vz*0.001;
        xf=xf-Vx*dtt;
        yf=yf+Vy*dtt;
        zf=drone_navdata.altd*0.001;
        myfile5<<xf<<setw(20)<<yf<<setw(20)<<zf<<endl;



        if (xk==xp )
        {

          x=x-Vx*dtt;
          y=y+Vy*dtt;
          //z=z+Vz*dtt;
          z=drone_navdata.altd*0.001;

          p00=p00+ (q00+2*a00*p00)*dtt;
          p11=p11+ (q11+2*a11*p11)*dtt;
          p22=p22+ (q22+2*a22*p22)*dtt;

          errorx=x-xdes;
          errory=ydes-y;
          errorz=zdes-z;

          lx=0.1*errorx-0.03*Vx;
          ly=0.1*errory-0.03*Vy;
          lz=0.3*errorz-0.03*Vz;

          move(lx,ly,lz,0,0,0);
          myfile4<<x<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ly<<setw(20)<<z<<setw(20)<<lz<<endl;
          cout<<i<<setw(20)<<"Odoooo"<<setw(20)<<x<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ly<<endl;

        }

        else if (xk!=xp )

        {

          k00=(c00*p00)/(p00*c00*c00+r00);
          k11=(c11*p11)/(p11*c11*c11+r11);
          k22=(c22*p22)/(p22*c22*c22+r22);

          x=x+k00*(xk-x);
          y=y+k11*(yk-y);
          z=z+k22*(zk-z);
          errorx=x-xdes;
          errory=ydes-y;
          errorz=zdes-z;
          lx=0.1*errorx-0.03*Vx;
          ly=0.1*errory-0.03*Vy;
          lz=0.35*errorz-0.03*Vz;
          move(lx,ly,lz,0,0,0);
          p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
          p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
          p00=(k22*k22*r22)+p22*(c22*k22-1)*(c22*k22-1);

          myfile4<<x<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ly<<setw(20)<<z<<setw(20)<<lz<<endl;
          cout<<i<<setw(20)<<"Kalman"<<setw(20)<<x<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ly<<endl;

        }
        /////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////
		////cout<<"Bitchhhhh"        /////////////////////////////////////////////////////////////////////////
		//cout<<"Arsehole"<<est_x<<setw(30)<<t2-tp<<endl;

		/*
		x=est_x*100;
			y=est_y*100;
			z=est_z*100;
			cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
		//cout<<kinect_ready<<endl;

		t1 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0.1;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;
		cout<<iii<<endl;
	velocity_publisher.publish(vel_msg);
    myfile3<<drone_navdata.rotX<<setw(25)<<drone_navdata.vy<<setw(25)<<drone_navdata.ay<<endl;
*/
    xp=xk;
    tp=tc;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();
    i=i+1;
	//iii=iii+1;

	}while(i<2000);
		//(abs(errorx)>0.15 && abs(errory)>0.15 && abs(errorz)>0.15);
	cout<<"land"<<setw(20)<<totalt;
	land();

myfile4.close();
myfile5.close();
myfile6.close();
	//ros::spinOnce();
}


void moveto1(double x, double y, double z)
{

	cout<<x<<" "<<" "<<y<<endl;
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	//double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y, e_z,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0,x_d =0,y_d =0,z_d =0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x, y, z);

	ros::Rate loop_rate(f);

	//ofstream myfile;
	//myfile.open("/home/icgel/mov_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = r0;

	do{

		//double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z = 0;

		if(r > 0.15)
		{
			Vx = (drone_navdata.vx*0.001);
			Vy = (drone_navdata.vy*0.001);
			lx =  - Kp*0.15*(e_x)- Kd*0.03*Vx; //moves fast if velocity is given high
			ly = - Kp*0.15*(e_y)- Kd*0.03*Vy;
			lz = 0;

			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
//		cout<<"Iteration"<<i<<endl;
		//myfile<<endl<<i<<setw(20);


		//myfile<<drone_navdata.header.stamp<<setw(50);

		move(lx, ly, lz, ax, ay, az);

	//	cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;


	///feedback
		//x = K_est_x;
		//y = K_est_y;
		x = x + ((Vx+Vx0)/2)/f;
		y = y + ((Vy+Vy0)/2)/f;
		z = drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		r = getDistance(e_x, e_y, e_z, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((ros::ok()) && kinect_ready==0 );
	hover();
	//hemdu(est_x,est_y,est_z);//to start tau guidance
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();
	//	myfile.close();
	//est_x==0 && est_y==0 && est_z==0
		ROS_INFO("move complete");
		//hover();

}


void k_hold()
{
	ros::Rate loop_rate(10);
	//double z=drone_navdata.altd*0.001;
	do
	{



			hover();
			cout<<"tautautautauautauuuuuuuuuuuuuuuuuut"<<endl;
			ROS_INFO(" tau x, y, z =    %f     %f     %f",est_x,est_y,est_z);
			tau(est_x,est_y,est_z);

/*
		//	hover();

			int iredX= redX;
			int iredY = redY;
			double Vx = drone_navdata.vx;
			double Vy = drone_navdata.vy;
			H = 0;

			cout<<"droneeeeeeeeeeeeeeeeeeeee________cameraaaaaaaa"<<redX<<setw(10)<<redY<<endl;
			onTarget(redX, redY);
*/
			ros::spinOnce();
			loop_rate.sleep();
	}while((ros::ok()));
	ROS_INFO("blablabla");
//	land();



}

void tau(double x0, double y0, double z0)

{

double hemx,hemy,hemz;
int xii=0,yii=0,zii=0;
double ex0, ey0, ez0, x=0, y=0,z=0,r0, i=0,v,phi,iphi, ex,ey,ez,nr,r,a,xdes=1,ydes=1,zdes, tau1,xprev=0,yprev=0,zprev=0,epx=0,epy=0,epz=0,errorx=0,errory=0,errorz=0;
v  =0.2;
int fuck=0;
double Vx0=0, Vy0=0,dt=0;
ex0=est_x-0.35;
ey0=est_y;
ez0=est_z-0.95;
r0=getDistance(ex0, ey0, ez0, 0, 0, 0);
nr = getDistance(ex0, ey0, 0, 0, 0, 0);
iphi = asin(nr/r0);
tau1=-r0/v;
double t0 = ros::Time::now().toSec();
double tprev=t0;
ofstream myfile;
ofstream myfile2;
ofstream myfile3;
	myfile.open("/home/icgel/icgel/hem_1A.txt", ios::out | ios::app );
	myfile<<"AAA"<<endl;
	myfile2.open("/home/icgel/icgel/hem_1AA.txt", ios::out | ios::app );
	myfile2<<"BBB"<<endl;
	myfile3.open("/home/icgel/icgel/hem_1AAA.txt", ios::out | ios::app );
	myfile3<<"CCC"<<endl;
	int aa=0;
	ros::Rate loop_rate(200);

float ttt=0.1;

do
{

double t1=ros::Time::now().toSec();
dt=t1-tprev;
ttt=ttt+0.5;

//0.02

/*
ex=ex0*pow((1+k*(t1-t0)/tau1),(1/k));
ey=ey0*pow((1+k*(t1-t0)/tau1),(1/k));
ez=ez0*pow((1+k*(t1-t0)/tau1),(1/k));
*/
ex=ex0*pow((1+k*(ttt)/tau1),(1/k));
ey=ey0*pow((1+k*(ttt)/tau1),(1/k));
ez=ez0*pow((1+k*(ttt)/tau1),(1/k));


//r=getDistance(ex, ey, ez, 0, 0, 0);
//phi = iphi * pow((r/r0),(1/k));
//a = r*sin(phi)/(r0*sin(iphi));
//xdes=a*ex0 +0.35;
//ydes=a*ey0;

if (xii==0)
{
xdes=ex*100+35;
}
if (yii==0)
{
ydes=ey*100;
}
if (zii==0)
{
zdes=ez*100+95;
}
if ((ydes<2)&&(ydes>-2))
{
	ydes=0;
	yii=yii+1;
}
if ((xdes>34)&&(xdes<36))
{
	xdes=35;
	xii=xii+1;
}
if ((zdes<96))
{
	zdes=95;
	zii=zii+1;
}
//ros::spinOnce();
hemx=drone_navdata.vx*0.001;
hemy=drone_navdata.vy*0.001;


//begin
if (kinect_ready > 0)
{
	x=est_x*100;
	y=est_y*100;
	z=est_z*100;
	cout<<"kinect working"<<setw(20);

}

else if (kinect_ready == 0)

{

	x = x - ((hemx+Vx0)/2)*100*dt;
	y = y + ((hemy+Vy0)/2)*100*dt;
	z = drone_navdata.altd*0.1;
	cout<<"imu working"<<setw(20)<<x<<setw(20)<<y<<setw(20);
	myfile3<<fuck<<setw(20)<<x<<setw(20)<<hemx<<setw(20)<<y<<setw(20)<<hemy<<setw(20)<<z<<setw(20)<<dt<<endl;

}

Vx0=hemx;
Vy0=hemy;
//end


/*drone_navdata.vx
lx = 0.1*(x-xdes)  ;
ly = 0.1* (ydes-y)  ;
lz = 0.3*(zdes-z)  ;
*/

lx = 0.003*(x-xdes) + 0*((x-xdes)-epx)/(t1-tprev);
ly = 0.003*(ydes-y) + 0*((ydes-y)-epy)/(t1-tprev);
lz = 0.0045*(zdes-z) + 0*((zdes-z)-epz)/(t1-tprev);

myfile<<fuck<<setw(20)<<x<<setw(20)<<xdes<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ydes<<setw(20)<<ly<<endl;
myfile2<<fuck<<setw(20)<<z<<setw(20)<<zdes<<setw(20)<<lz<<endl;
tprev= t1;
errorx=x-xdes;
errory=y-ydes;
errorz=z-zdes;
epx=(x-xdes);
epy=(ydes-y);
epz=(zdes-z);
if ((x==xprev )||(y==yprev )||(z==zprev ))
{
hover();
cout<<"Sameeeeeee Readingssssssss"<<setw(30)<<kinect_ready<<setw(30)<<drone_navdata.batteryPercent<<endl;
aa=aa+1;
lx=0;
ly=0;
lz=0;
}

else if ((x != xprev)||(y!=yprev)||(z!=zprev))
{

	aa=0;
	cout<<"its working"<<setw(30)<<drone_navdata.batteryPercent<<endl;
	//cout<<"its workingggggggggg"<<x<<setw(15)<<lx<<setw(20)<<y<<setw(15)<<ly<<setw(20)<<z<<setw(20)<<lz<<endl;

}

if (xii>20 && yii>20 && zii>20)
{
	cout<<"FU landdddddddddddddddd "<<endl;
	land();
	break;
}
if (aa>7)
{
land();
cout<<"Landddddddddddddddddddddddd"<<endl;
break;
}
xprev=x;
yprev=y;
zprev=z;

fuck=fuck+1;
/*
vel_msg.linear.x = lx;
vel_msg.linear.y = ly;
vel_msg.linear.z = lz;

vel_msg.angular.x = 0;
vel_msg.angular.y = 0;
vel_msg.angular.z = 0;
velocity_publisher.publish(vel_msg);*/

cout<<lx<<setw(20)<<ly<<setw(20)<<lz<<endl;
int i=0;

do
{

move(lx, ly, lz, 0, 0, 0);
i=i+1;
}
while(i<10);
i=0;

//while(abs(errorx) > 1 && abs(errory) > 1 && abs(errorz) > 1);

//cout<<est_x<<setw(20)<<est_y<<endl;
ros::spinOnce();
loop_rate.sleep();

}

while(ros::ok());
land();
myfile.close();

}
//&& abs(xdes-35)>5 && abs(ydes)>5
void onTarget(int redX, int redY)
{
	ros::Rate loop_rate(10);

	res = 1;


//	if(redX > 0)

		double z=drone_navdata.altd*0.001;

		double x = (z/foc)*(180-redY);
		double y = (z/foc)*(320-redX);
		cout<<"Desired Pos"<<x<<"&"<<y<<endl;
	//	moveadv(-2,0,0);
//			cout<<"Est Pos"<<est_x<<"&"<<est_y<<endl;

		//	moveto(x,y,z);

		tau1(x,y,z);
		land();
		ros::spinOnce();
		loop_rate.sleep();
//

}
void tau1(double x0, double y0, double z0)
{
double ex0, ey0, ez0, x=0, y=0,z=0,r0, i=0,v,phi,iphi, ex,ey,ez,nr,r,a,xdes,ydes,zdes, tau1,xprev=0,yprev=0,zprev=0,epx=0,epy=0,epz=0;
v = 0.2;
ex0=(z/foc)*(180-redY);;
ey0=(z/foc)*(320-redX);
ez0=drone_navdata.altd*0.001;;

r0=getDistance(ex0, ey0, ez0, 0, 0, 0);
nr = getDistance(ex0, ey0, 0, 0, 0, 0);
iphi = asin(nr/r0);
tau1=-r0/v;
double t0 = ros::Time::now().toSec();
double tprev=t0;
ofstream myfile;
ofstream myfile2;
	myfile.open("/home/icgel/icgel/hem_1B.txt", ios::out | ios::app );
	myfile<<"Entering the camera loop ";
	myfile2.open("/home/icgel/icgel/hem_1BB.txt", ios::out | ios::app );
	myfile2<<"Entering the camera loop ";
	int aa=0;
	ros::Rate loop_rate(15);

do
{
double t1=ros::Time::now().toSec();

ex=ex0*pow((1+k*(t1-t0)/tau1),(1/k));
ey=ey0*pow((1+k*(t1-t0)/tau1),(1/k));
ez=ez0*pow((1+k*(t1-t0)/tau1),(1/k));
//r=getDistance(ex, ey, ez, 0, 0, 0);
//phi = iphi * pow((r/r0),(1/k));
//a = r*sin(phi)/(r0*sin(iphi));
//xdes=a*ex0 ;
//ydes=a*ey0;
xdes=ex;
ydes=ey;
zdes=ez ;

x = (z/foc)*(180-redY);
y = (z/foc)*(320-redX);
z = drone_navdata.altd*0.001;
/*
lx = Kp*0.1*(x-xdes)  ;
ly =  0.1*(y-ydes)  ;
lz =0.3*(zdes-z) ;
*/
myfile<<x<<setw(20)<<xdes<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ydes<<setw(20)<<ly<<endl;
myfile2<<z<<setw(20)<<zdes<<setw(20)<<lz<<endl;

lx = Kp*0.5*(x-xdes) + Kd*0*((x-xdes)-epx)/(t1-tprev);
ly =  Kp*0.5*(y-ydes)+ Kd*0*((y-ydes)-epy)/(t1-tprev);
lz = Kp*1*(zdes-z) + Kd*0*((zdes-z)-epz)/(t1-tprev);
tprev= t1;
if ((x==xprev )||(y==yprev )||(z==zprev ))
{
//hover();
cout<<"hoverrrrrrrrrr"<<aa<<endl;
aa=aa+1;
}
if (aa>4)
{
land();
cout<<"Landddddddddddddddddddddddd"<<endl;
}


if ((x != xprev)||(y!=yprev)||(z!=zprev))
{
	aa=0;
	cout<<"Ganjaaaaaa"<<x<<setw(10)<<y<<setw(10)<<z<<endl;
	move(lx, ly, lz, 0, 0, 0);
}
xprev=x;
yprev=y;
zprev=z;
epx=(x-xdes);
epy=(y-ydes);
epz=(zdes-z);
ros::spinOnce();
loop_rate.sleep();

}
while( z >0.3);
land();
myfile.close();

}




void moveadv(double x, double y, double z)
{

	cout<<x<<" "<<" "<<y<<endl;
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y, e_z,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0,x_d =0,y_d =0,z_d =0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x, y, z);

	//ros::Rate loop_rate(f);

	//ofstream myfile;
	//myfile.open("/home/icgel/mov_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = r0;

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z=0;
		//e_z = drone_navdata.altd*0.001;

		if(r > 0.15)
		{
			Vx = (drone_navdata.vx*0.001);
			Vy = (drone_navdata.vy*0.001);
			lx =  - Kp*0.25*(e_x)- Kd*0.015*Vx; //moves fast if velocity is given high
			ly = - Kp*0.25*(e_y)- Kd*0.015*Vy;
			lz = 0;
			//lz = -((0.05)*(e_z));

			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
			ROS_INFO("moveadv");
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
//		cout<<"Iteration"<<i<<endl;
//		myfile<<endl<<i<<setw(20);


//		myfile<<drone_navdata.header.stamp<<setw(50);

		move(lx, ly, lz, ax, ay, az);

	//	cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;


	///feedback
		x = K_est_x;
		y = K_est_y;
		//x = x + ((Vx+Vx0)/2)/f;
		//y = y + ((Vy+Vy0)/2)/f;
		z = drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		r = getDistance(e_x, e_y, e_z, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		//loop_rate.sleep();


		}while((r > 0.15) && (ros::ok()) && redX==0);
//		myfile.close();

}

void moveto(double x, double y, double z)
{

	cout<<x<<" "<<" "<<y<<endl;
	ax = 0;
	ay = 0;
	//az = 0;
	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y, e_z,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0,x_d =0,y_d =0,z_d =0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x, y, z);

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/mov_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = r0;

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z = 0;

		if(r > 0.2)
		{
			Vx = (drone_navdata.vx*0.001);
			Vy = (drone_navdata.vy*0.001);
			lx =  - Kp*0.1*(e_x)- Kd*0.01*Vx; //moves fast if velocity is given high
			ly = - Kp*0.1*(e_y)- Kd*0.01*Vy;
			lz = 0;

			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
//		cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(20);


		myfile<<drone_navdata.header.stamp<<setw(50);

		move(lx, ly, lz, ax, ay, az);

	//	cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;


	///feedback
		//x = K_est_x;
		//y = K_est_y;
		x = x + ((Vx+Vx0)/2)/f;
		y = y + ((Vy+Vy0)/2)/f;
		z = drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		r = getDistance(e_x, e_y, e_z, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((r > 0.2) && (ros::ok()));
		myfile.close();

}



void min_jerk(double x_d, double y_d, double z0, double Tf)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec();
	double e_x, e_y, e_z,e_x0, e_y0, e_z0, x=0, y=0,z,r, i=0,v;


	double g,x0,u0,v0,ax0,az0,xf,zf,uf,vf,axf,azf, tgo, u1, u2,r0,c1x,c2x,c3x,c1z,c2z,c3z,K = 1, dxg, dxg0,dzg,dzg0,xg,xg0,zg,zg0, zi0;

	e_x0 = x-x_d;
	e_y0 = y-y_d;
	e_z0 = z0;

	r = getDistance(e_x0, e_y0, e_z0, 0, 0, 0);

	//cout<<endl<<r0;

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/jerk_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";


g=9.81;
x0=0;
u0=drone_navdata.vx*0.001;
v0=0;
ax0=0;
az0=-g;
xf=x_d;
zf=0.5;
uf=0;
vf=-0.5;
axf=0;
azf=-g;
dxg0 = 0;
dzg0 = v0;
xg0 = 0;
zg0=z0;
zg =zg0;

do
{
	if(redX > 0)
		{
			double z = drone_navdata.altd*0.001;
			ly = -Kp*(redY - 320)*0.001;
			cout<<"On Target Move forward by "<<redX<<endl;
			lx = -Kp*(redX - 170)*0.001;
			cout<<"On Target Move right by "<<redY<<endl;
 			lz = 0;//-0.01*(z-1);
			//cout<<"On Target Move height by "<<z<<endl;

		}
	else
		{
			double z = drone_navdata.altd*0.001;
			lx = 0;//-Kp*0.001*Vx;
			//cout<<"Target Not found Move forward by "<<Vx<<endl;
			ly = 0;//-Kp*0.001*Vy;
			//cout<<"Target Not found Move right by "<<Vy<<endl;
			lz = 0; //-Kp*(z_new - z)*0.01;
		}

	move(lx,ly, lz, 0, 0, 0);

	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();
//}while((z > 1) && (ros::ok()));
}while(((redY-320)> 25 || abs(redX-180)>25) && (ros::ok()));


	do{
		double t1=ros::Time::now().toSec();

		c1x= (10/(Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)+(4/(Tf*Tf))*(u0-uf+axf*Tf)+(1/(2*Tf))*(ax0-axf);
		//cout<<endl<<"coeff = "<<c1x<<endl;
		c2x= (-15/(Tf*Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(u0-uf+axf*Tf)-(1/(Tf*Tf))*(ax0-axf);
		//cout<<endl<<"coeff = "<<c2x<<endl;
		c3x= (6/(Tf*Tf*Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(u0-uf+axf*Tf)+(1/(2*Tf*Tf*Tf))*(ax0-axf);
		//cout<<endl<<"coeff = "<<c3x<<endl;


		c1z= (10/(Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)+(4/(Tf*Tf))*(v0-vf+(azf+g)*Tf)+(1/(2*Tf))*(az0-azf);
		//cout<<endl<<"coeff = "<<c1z<<endl;
		c2z= (-15/(Tf*Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(v0-vf+(azf+g)*Tf)-(1/(Tf*Tf))*(az0-azf);
		//cout<<endl<<"coeff = "<<c2z<<endl;
		c3z= (6/(Tf*Tf*Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(v0-vf+(azf+g)*Tf)+(1/(2*Tf*Tf*Tf))*(az0-azf);
		//cout<<endl<<"coeff = "<<c3z<<endl;
		tgo=Tf-(i)/f;

		cout<<endl<<tgo<<endl;
		cout<<endl<<zg<<endl;


		u1=20*c3x*tgo*tgo*tgo+12*c2x*tgo*tgo+6*c1x*tgo+axf;
		//cout<<endl<<u1;

		u2=20*c3z*tgo*tgo*tgo+12*c2z*tgo*tgo+6*c1z*tgo+azf;
		//cout<<endl<<u2;

		dxg = dxg0 + u1/f;
		dzg =  dzg0 + (u2+g)/f;

		xg = xg0 + dxg/f;
		zg =  zg0 + (dzg)/f;

		//cout<<endl<<"Time = "<<t1-t0<<endl;

		myfile<<xg<<setw(20);
		//myfile<<yg<<setw(20);
		myfile<<zg<<setw(20);
		myfile<<x<<setw(25);
		myfile<<y<<setw(25);
		myfile<<z<<setw(25);

		//cout<<x<<setw(20)<<xg<<endl<<y<<setw(20)<<yg<<endl<<z<<setw(20)<<zg<<endl;

		//cout<<endl<<"Dist = "<<r<<endl;



		if(r>0.5)
		{
			lx = - Kd*0.3*(drone_navdata.vx*0.001-dxg) - Kp*0.8*(x-xg);
			ly = 0;
			lz = -((0.7)*(z-zg));
			cout<<"Cmd_vel"<<lx<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz =0;
			break;
		}



		i = i+1;
		//cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(25);


		move(lx, ly, lz, ax, ay, az);




		myfile<<drone_navdata.vx<<setw(25);
		myfile<<drone_navdata.vy<<setw(25);
		myfile<<drone_navdata.altd<<setw(25);


		x = x + drone_navdata.vx*0.001/f;
		y = y + drone_navdata.vy*0.001/f;
		z = drone_navdata.altd*0.001;

		dxg0 = dxg;
		dzg0 = dzg;
		xg0 = xg;
		zg0 = zg;

		r = getDistance(x, y, z, x_d, y_d, 0);


		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((zg >= 0.1) && (ros::ok()));
		myfile.close();

}

void k_move(double x_d, double y_d, double z_d)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x=0, e_y=0, e_z=0, x=0, y=0,z =0,r,r0, i=0,z0, Vx, Vy, Vx0=0, Vy0=0;
	z0 = drone_navdata.altd*0.001;


	r0 = getDistance(0, 0, 0, x_d, y_d, 0);

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/kinect_mov.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	z = z0;
	r = 0;   //Must change initial value of r

	do{
		double t1=ros::Time::now().toSec();

		//cout<<endl<<"ALT = "<<z<<endl;

		e_x = x - x_d;
		e_y = y - y_d;
		e_z = z - z_d;

		if(r > tol)
		{
			if (redX==0 || redY==0)
			{
				lx = 0;
				ly = 0;
				lz = 0;

			}
			else
			{
				Vx = (drone_navdata.vx*0.001);
				Vy = (drone_navdata.vy*0.001);
				lx =  - Kp*0.2*(e_x)-Kd*0.025*(Vx);
				ly = - Kp*0.2*(e_y)-Kd*0.025*(Vy);
				lz = 0;//- Kpz*(e_z);
			}

			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx = 0;
			ly = 0;
			lz = 0;
			//break;
		}

		i = i+1;
		//cout<<x<<setw(20)<<y<<setw(20)<<z<<endl;
		//cout<<"Iteration"<<i<<endl;
		myfile<<endl<<i<<setw(20);

		move(lx, ly, lz, ax, ay, az);
		cout<<"co or"<<x<<setw(10)<<y<<setw(10)<<z<<endl;


		x = est_x; 				//x + ((Vx+Vx0)/2)/f;
		y = est_y;				//y + ((Vy+Vy0)/2)/f;
		z = drone_navdata.altd*0.001;
		Vx0 = Vx;
		Vy0 = Vy;

		r = getDistance(e_x, e_y,e_z, 0,0,0);

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((ros::ok()));
		myfile.close();

}



void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.vx	= 	pose_message->vx;
	drone_navdata.vy 	= 	pose_message->vy;
	drone_navdata.vz 	= 	pose_message->vz;
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay 	= 	pose_message->ay;
	drone_navdata.az 	= 	pose_message->az;
	drone_navdata.rotX	= 	pose_message->rotX;
	drone_navdata.rotY	= 	pose_message->rotY;
	drone_navdata.rotZ	= 	pose_message->rotZ;
	drone_navdata.magX	= 	pose_message->magX;
	drone_navdata.magY	= 	pose_message->magY;
	drone_navdata.magZ	= 	pose_message->magZ;
	drone_navdata.altd 	= 	pose_message->altd;
	drone_navdata.tm	= 	pose_message->tm;
	drone_navdata.header	= 	pose_message->header;
	drone_navdata.batteryPercent= pose_message->batteryPercent;
}
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message)
{
	ang.header	= 	imu_message->header;
	ang.angular_velocity	= 	imu_message->angular_velocity;
	ang.linear_acceleration =  imu_message->linear_acceleration;
}
void hem_odo(const nav_msgs::Odometry::ConstPtr & nav_message)
{

	odo.pose = nav_message->pose;
}

void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message)
{
	lat	= 	gps_message->latitude;
	lon 	= 	gps_message->longitude;
	ele 	= 	gps_message->elevation;
}

void new_gpsCallback(const std_msgs::String::ConstPtr& gps1)
{
	if (gps_cntr == 0)
	{
		stringstream(gps1->data.c_str()) >> new_lat;
		gps_cntr=1;
	}

	else if (gps_cntr==1)
	{
		stringstream(gps1->data.c_str()) >> new_lon;
		gps_cntr=0;
	}
}

void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val)
{
	//cout<<"TaRgOt:"<< val->x<< val->y;
	redX = val->x;
	redY = val->y;
}

void EstCallback(const ardrone_test::est_co::ConstPtr& est)
{
  //cout<<"Target:"<< val->x<< val->y;
	est_x = est->z;
	est_y = est->y;
	est_z = est->x;
	kinect_ready = est->w;
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;
}

void INS_K_Callback(const ardrone_test::est_co::ConstPtr& esti)
{
  //cout<<"Target:"<< val->x<< val->y;
	K_est_x = esti->x;
	K_est_y = esti->y;
	K_est_z = esti->z;
	//ROS_INFO("hi = %f, %f ",K_est_x,K_est_y);
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;
}

void testdataCallback(const ardrone_autonomy::vector31::ConstPtr & test )
{
  //cout<<"Target:"<< val->x<< val->y;
	test_x = -test->z;
	test_y = test->y;
	test_z = test->x;
	//ROS_INFO("hi = %f, %f ",K_est_x,K_est_y);
	//cout<<"Target:"<< est_x<< setw(25)<<est_y<<setw(25) << est_z;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr & joy )
{
	lx = joy->axes[0];
	ly = joy->axes[1];
	az = joy->axes[2];
	lz = joy->axes[3];
	to = joy->buttons[8];
	la = joy->buttons[9];
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

double deg2rad(double angle_in_degrees)
{
	return angle_in_degrees*PI/180.0;
}

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2)+pow(z1-z2, 2));
}

void takeoff()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();  // epoch time
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				T_pub_empty.publish(emp_msg); /* launches the drone */
				ros::spinOnce(); // feedback
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		ROS_INFO("ARdrone launched");
		exit(0);
		}//ros::ok loop
	hover();

}

void land()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				L_pub_empty.publish(emp_msg); /* launches the drone */
				ros::spinOnce();
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		ROS_INFO("ARdrone landed");
		exit(0);
		}//ros::ok loop

}

void hover()
{

	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'
	double t1;

	ros::Rate loop_rate(200);

	do{

		t1 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;

	velocity_publisher.publish(vel_msg);

	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();

	}while(t1 <= (t0+10));

	//ros::spinOnce();
}

void move(float lx, float ly, float lz, float ax, float ay, float az )
{

	//defining the linear velocity
	vel_msg.linear.x = lx;
	vel_msg.linear.y = ly;
	vel_msg.linear.z = lz;

	//defining the linear velocity
	vel_msg.angular.x = ax;
	vel_msg.angular.y = ay;
	vel_msg.angular.z = az;

	velocity_publisher.publish(vel_msg);

}

void moveup(double z_d)
{
	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_z, z =0, i=0 ,z0 ,r = 1, x, y, x0, y0, Vx, Vy, e_x, e_y;

	z0 = drone_navdata.altd*0.001;

	ros::Rate loop_rate(f);

	z = z0;
	ofstream myfile;
	myfile.open("/home/icgel/mov_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";

	do{
		e_z = z - z_d;

		double t1=ros::Time::now().toSec();

		if((abs(e_z) > 0.5 && z > 0))
		{
			double z = drone_navdata.altd*0.001;
					lx = 0;
					ly = 0;
					lz = -0.7*(z-z_d);
			cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
		}
		else
		{
			lx =0;
			ly =0;
			lz = 0;
			//break;
		}

		myfile<<endl<<i<<setw(20);
		myfile<<drone_navdata.header.stamp<<setw(50);

		cout<<"iter"<<i<<endl;

		move(lx, ly, lz, 0, 0, 0);


		z = drone_navdata.altd*0.001;

		i = i+1;

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();

		}while(abs(e_z)>0.5 && ros::ok());
		myfile.close();

}


void record()
{
	double i =0, f= 200.0, d2r = 3.14159265/180;
	ofstream myfile;
	myfile.open("/home/icgel/icgel7/recorded_data.txt", ios::out | ios::app );
	myfile<<"gx gy gz ax ay az vxo vyo vzo theta phi psi"<<endl;
	ros::Rate loop_rate(200);
	do{
			move(0.1,0,0,0,0,0);
			myfile<<ang.angular_velocity.x<<setw(20);
			myfile<<ang.angular_velocity.y<<setw(20);
			myfile<<ang.angular_velocity.z<<setw(20);

			//myfile<<drone_navdata.ax*9.81<<setw(20);
			//myfile<<drone_navdata.ay*9.81<<setw(20);
			//myfile<<drone_navdata.az*9.81<<setw(20);
			myfile<<ang.linear_acceleration.x<<setw(20);
			myfile<<ang.linear_acceleration.y<<setw(20);
			myfile<<ang.linear_acceleration.z<<setw(20);

			myfile<<drone_navdata.vx*0.001<<setw(20);
			myfile<<drone_navdata.vy*0.001<<setw(20);
			myfile<<drone_navdata.vz*0.001<<setw(20);

			myfile<<(drone_navdata.rotY)*d2r<<setw(20);//theta-pitch
			myfile<<(drone_navdata.rotX)*d2r<<setw(20);//phi-roll
			myfile<<(drone_navdata.rotZ)*d2r<<endl;

			i = i+1;

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();
		}while(i<1500);
		myfile.close();
		land();
}



void record1()
{


	double i =0, dt = 0, a, b, c, d;
	//float xang = 0, yang= 0, zang=0, exang, eyang, ezang, rotX, rotY, rotZ, angX, angY, angZ;

	double t0=ros::Time::now().toSec();

		ofstream myfile;
		myfile.open("/home/icgel/Cali.txt", ios::out | ios::app );

		ros::Rate loop_rate(f);

		do{
			ROS_INFO("gps: %lf %lf",new_lat,new_lon);
			double t1=ros::Time::now().toSec();

			dt = t1-t0;

			myfile<<endl<<i<<setw(20);
			myfile<<dt<<setw(20);

/*
			myfile<<ang.angular_velocity.x<<setw(30);
			myfile<<ang.angular_velocity.y<<setw(30);
			myfile<<ang.angular_velocity.z<<setw(30);
*/
			myfile<<drone_navdata.ax<<setw(30);
			myfile<<drone_navdata.ay<<setw(30);
			myfile<<drone_navdata.az<<setw(30);

			myfile<<new_lat<<setw(30);
			myfile<<new_lon<<setw(30);

			//myfile<<drone_navdata.tm<<setw(40);
			//myfile<<drone_navdata.altd*0.001;

			i = i+1;

			t0 = t1;

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();
		}while(ros::ok());
		myfile.close();

}



void yawing(double angle)
{
	angle = angle + drone_navdata.rotZ;

		double error;
		ros::Rate loop_rate(10);
		do{

			//velocity_publisher.publish(vel_msg);

			error = (angle - drone_navdata.rotZ);
			cout<< "Yawwwwwwwwwwwwwww"<<drone_navdata.rotZ<<endl;
			vel_msg.angular.z = 0.1 * deg2rad(error);

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();

		}while((abs(error) > 5) && (ros::ok()));

		vel_msg.angular.z = 0;
		velocity_publisher.publish(vel_msg);
}

void validate(double x_d, double y_d, double z_d)
{
	ax = 0;
	ay = 0;
	//az = 0;

	//Measuring t : current time
	double t1, t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_x, e_y = 0, e_z, error, i = 0;

	ros::Rate loop_rate(f);

	ofstream myfile;
	myfile.open("/home/icgel/vali.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";


	do{
		t1=ros::Time::now().toSec();

	   //e_x = drone_navdata.vx* (t1-t0)*0.01 - x_d;//ASHU convert to inertial frame
	   //e_y = drone_navdata.vy* (t1-t0)*0.01 - y_d;

	   //double t2=ros::Time::now().toSec();
	   //cout<<"Time Stamps between the data = "<<t2-t1<<endl;

		//error = getDistance(0, 0, e_x, e_y);// only for tolernace

		myfile<<endl<<i<<setw(20);
		myfile<<lat<<setw(20);
		myfile<<lon<<setw(20);
		myfile<<ele<<setw(20);
		myfile<<drone_navdata.ax<<setw(20);
		myfile<<drone_navdata.ay<<setw(20);
		myfile<<drone_navdata.az<<setw(20);
		myfile<<ang.angular_velocity.x<<setw(20);
		myfile<<ang.angular_velocity.y<<setw(20);
		myfile<<ang.angular_velocity.z;

		//myfile<<drone_navdata.vx<<endl;
		//myfile<<drone_navdata.vy<<endl;
		//myfile<<drone_navdata.vz<<endl;


	    if (ros::ok())
		{
	    	//myfile<<t1-t0<<setw(10);
	    	cout<<"time = "<<t1-t0<<endl;
	    	lx = -Kp*2*(drone_navdata.vx*0.001-x_d);
	    	ly = -Kp*(drone_navdata.vy*0.001);
	    	//lx = 0;
	    	//ly =0;
	    	lz = 0;

//			myfile<<lx<<setw(10);
	//		cout<<"u_phi = "<<lx<<endl;
	    				//lz = Kp * e_z;
	     }
		else
		{
	  //  	myfile<<t1-t0<<setw(10);
			lx = 0;
			ly =0;
			lz = 0;
		//	myfile<<ly<<setw(10);
			//cout<<"u_phi = "<<ly<<endl;

		}


		//t0 = t1;

		i = i +1;

		cout<<"iteration"<<i<<endl;;


		move(lx, ly, lz, ax, ay, az );
		//myfile<<endl<<vel_msg.linear.x<<endl;
	    //myfile<<endl<<vel_msg.linear.y<<endl;


		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


	}while(((t1-t0 < 120)) && (ros::ok()));
	myfile.close();


}





