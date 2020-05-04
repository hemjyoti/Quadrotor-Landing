#include "ros/ros.h"
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
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
#include "std_msgs/Bool.h"
// class variables
std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class
geometry_msgs::Twist vel_msg2;
// Variables for Publishing
ros::Publisher T_pub_empty;				//take off publisher
ros::Publisher L_pub_empty;				//landing publisher
ros::Publisher velocity_publisher;		// velocity publisher
ros::Publisher velocity_publisher2;
ros::Publisher Buzz;
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
ros::Subscriber push_subscriber;
//ros::Subscriber filter_subscriber;		// tum estimation subscriber
ros::Subscriber gdrone_sub;
ros::Subscriber grobot_sub;
// Variables for Service
ros::ServiceClient client1;		// ardrone camera service
//#define N 4
#define N1 2
#define N 6
using namespace std;
bool rabha;
float lx, ly, lz, ax, ay, az,foc = 685, angx,angy,angz, est_x, est_y, est_z,K_est_x, K_est_y, K_est_z,test_x,test_y,test_z;
int to, la, f = 200 , redX, redY, res =0, H=0,I=0,J=0;
double hk = 0.3, k =0.49, z, tf, ang_time,hem; // k value for tau
double lat, lon, ele;
double xD=7,yD=4,zD=1;
//gps variables///
double new_lat, new_lon;
bool gps_cntr = 0;
//static float pos_x = 0, pos_y = 0, pos_z = 0;
const double PI = 3.14159265359;
double Kp = 0.2, Kd = 1,Kpz;// Gain for the controller
double k1 = 2.438, k2 = 0.779, k3 = 1.234, k4 = 0.221;// Gain for the controller
double tol = 0.4;// error tolerance
int kinect_ready=0;
//float inv[N][N];
double inv1[N1][N1];
//class instances
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;
nav_msgs::Odometry odo;
tum_ardrone::filter_state posisi;
nav_msgs::Odometry gdrone;
nav_msgs::Odometry grobot;
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
void push(const std_msgs::Bool::ConstPtr& pul);
void hem_gdrone(const nav_msgs::Odometry::ConstPtr & nav_msg);
void hem_grobot(const nav_msgs::Odometry::ConstPtr & nav_msg2);

// basic func
void fBuzz();
void hover();		// hovering
void takeoff();		// take off
void land();		//landing
void move(float lx, float ly, float lz, float ax, float ay, float az ); // publishing command velocity
void move2(float lx, float ly, float lz, float ax, float ay, float az );
// Basic Navigation func
void yawing(double angle);				// yawing movements

//Advanced Navigation function
void moveadv(double x_d, double y_d, double z_d);
void moveto(double x_d, double y_d, double z_d);
void moveto1(double x_d, double y_d, double z_d);//with kinect addition to moveto
void k_move(double x_d, double y_d, double z_d);
void k_hold();
void moveup(double z_d);
void validate(double x_d, double y_d, double z_d);
void tau(double x_d, double y_d, double z0);
void tauc(double x_d, double y_d, double z0);
void taug(double x_d, double y_d, double z0, double td);
//void tau1(double x0, double y0, double z0, double x1, double y1, double z1,double xof, double yof);
void min_jerk(double x_d, double y_d, double z0, double tf);
//void itau(double x_d, double y_d, double z0);
void onTarget(int redX, int redY);
// Miscellaneous func
int getch();								//similar to getting input
double deg2rad(double angle_in_degrees);	//degree to radian converter
double last_ten_average(double arr[], int integer);// take avg of last ten values of arr from integer
double last_ten_average2(double arr[], int integer);// take avg of last ten values of arr from integer

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2);
void compare_tum_position_vals();
void hemdu(double xdd,double ydd,double zdd);void gps_odom_ekf();
void hem_gps();
void imu_ekf(double x_d, double y_d, double z_d);
void imugps_tau(double x_d, double y_d, double z_d);
void imukinect_taug(double x_d, double y_d, double z_d, double td);
void check_kinect();
float a[4][4]={2,2,3,4,2,4,1,2,6,4,9,7,2,8,8,6};
double inv[4][4];
static void matinv(double *a,double *inv);
void getCofactor(double A[N][N], double temp[N][N], int p, int q, int n);
double determinant(double A[N][N], int n);
void adjoint(double A[N][N],double adj[N][N]);
bool inverse(double A[N][N], double inverse[N][N]);
template<class T>
void display(T A[N][N]);
//void getCofactor(int A[N][N], int temp[N][N], int p, int q, int n);
//int determinant(int A[N][N], int n);
//void adjoint(int A[N][N],int adj[N][N]);
//bool inverse(double A[N][N], float inverse[N][N]);

//void hem_gps_2(int &x, int &y);
int adj[N][N];  // To store adjoint of A[][]
int adj1[N][N];
//float inv[N][N]; // To store inverse of A[][]


int main(int argc, char **argv)
{
	//Initiate the ROS

	ros::init(argc, argv, "waypuoint_nav");
	ros::NodeHandle n; 			// Nodehandle is like a class where you are operating

	// Publishing the data


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100); // initialize to send the Control Input
    velocity_publisher2= n.advertise<geometry_msgs::Twist>("/sim_p3at/cmd_vel",100);
	T_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);  //initialize to send the take off command  /* Message queue length is just 1 */

	L_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/land", 1); //initialize to send the land command /* Message queue length is just 1 */
    Buzz= n.advertise<std_msgs::Empty>("toggle_led", 1);
	// Subscribing the data
    gdrone_sub=n.subscribe("/ground_truth/state", 10, hem_gdrone);
    grobot_sub=n.subscribe("/sim_p3at/odom", 10, hem_grobot);

	pose_subscriber = n.subscribe("/ardrone/navdata", 200, poseCallback);	//initialize to receive processed sensor data
	imu_subscriber = n.subscribe("/ardrone/imu", 200, imuCallback); //initialize to receive raw sensor data
	gps_subscriber = n.subscribe("/ardrone/navdata_gps", 10, gpsCallback); //initialize to receive gps data
	est_sub = n.subscribe("Coordinate", 100, EstCallback); //initialize to receive ground cam target data
	INS_K_est = n.subscribe("/estimation_data", 200, INS_K_Callback); //initialize to receive kinect+ uav sensor estimation sensor data
	test_sub = n.subscribe("/test", 100, testdataCallback);
    odo_subscriber = n.subscribe("/ardrone/odometry", 200, hem_odo);
    push_subscriber = n.subscribe("pushed", 200, push);
	num_sub = n.subscribe("/chatter1", 100, NumCallback);
	tum_pose = n.subscribe("/ardrone/predictedPose", 100,tum_posecallback );
	joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
	client1 = n.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel", 1);
	ardrone_autonomy::CamSelect srv;

				srv.request.channel = 0;
				if (client1.call(srv))
				{
					ROS_INFO("Called");
				}
				else
				{
					ROS_INFO("Nope");
				}

	// Calling toggle cam service
/*for(i=0;i<100;i++)
	{
	fBuzz();
    cout<<"motherfuckerrrrrrrrrrrrrrrrrrrr";
	}
	client1 = n.serviceClient<ardrone_autonomy::CamSelect>("/ardrone/setcamchannel", 1);

	ardrone_autonomy::CamSelect srv;

	srv.request.channel = 0;
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

//				int A[N][N] = { {1, 2, 3},
//				                    {2, 5, 1},
//				                    {7, 4, 9}};




while(ros::ok())
{

	cout<<"Press any key"<<endl;
    cout<<"t -> takeoff"<<endl;
    cout<<"l -> land"<<endl;
    cout<<"n -> Record data"<<endl;
    cout<<"y -> yaw"<<endl;
    cout<<"h -> hover"<<endl;
    cout<<"m -> move up"<<endl;
    cout<<"j -> Min_jerk"<<endl<<endl;
    cout<<"e -> move using imu"<<endl;
    //Hemjyothi and Kaustubhs functions
    cout<<"q -> slam and imu based indoor tau guided landing"<<endl;
    cout<<"r -> imu ekf and gps based outdoor tau guided landing "<<endl;
    cout<<"o -> check kinect"<<endl;
    cout<<"w -> kalman gps and odometry"<<endl;
    cout<<"a -> slam and imu based indoor TAU-G guided landing"<<endl;
    cout<<"v -> old Validate"<<endl;
   // ROS_INFO("%d",kinect_ready);spjivhltny

    int c = getch();   // call your non-blocking input function

    double x, y, z;			// for reference value

    switch (c)
    {
    case 'm':
    	moveup(0.2);
    	break;
    case 'e':
    	fBuzz();
    	//imu_ekf(5,5,5);
    	break;
    case 'r':
		//imugps_tau(30,5,1);


		imukinect_taug(7,-7,0.4,15);

		break;
    case 'Z':

    	   cout<<drone_navdata.rotZ<<endl;
    	   break;
	case 'o':
		//compare_tum_position_vals();
		check_kinect();

			break;
	case 'a':
		tau(7,5,1.5);
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

	case 'q':
	//	cout<<atan2(1, -2);
//		moveup(4);
//		cout<<" I am doneeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
//
//		hover();
//imukinect_taug(2, 2,  1,10);
//		matinv((float *)a,(float *)inv);
//		cout<< inv[0][0]<<endl;
taug(7.3,5,0.5,15 );
//		min_jerk(2,2,2,2);
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

void taug(double x1, double y1, double z0, double td1)
{
double hemoo=0;
ros::Rate loop_rate(100);
ofstream myfile, myfile2, myfile3, myfile4;
myfile.open("/home/icgel/1cgel/25 apr/taug_tum.txt", ios::out | ios::app );
//myfile2.open("/home/icgel/1cgel/25 feb/taug_tumA.txt", ios::out | ios::app );
//myfile3.open("/home/icgel/1cgel/25 feb/taug_tumB.txt", ios::out | ios::app );
//myfile4.open("/home/icgel/1cgel/25 feb/taug_tumC.txt", ios::out | ios::app );


myfile<<"a"<<setw(20)<<"b"<<setw(20)<<"c"<<setw(20)<<"d"<<setw(20)<<"e"<<setw(20)<<"f"<<setw(20)<<"g"<<setw(20)<<"h"<<endl;
myfile2<<"a"<<setw(20)<<"b"<<setw(20)<<"c"<<setw(20)<<"d"<<setw(20)<<"e"<<setw(20)<<"f"<<setw(20)<<"g"<<setw(20)<<"h"<<endl;
myfile3<<"a"<<setw(20)<<"b"<<setw(20)<<"c"<<setw(20)<<"d"<<setw(20)<<"e"<<setw(20)<<"f"<<setw(20)<<"g"<<setw(20)<<"h"<<setw(20)<<"i"<<setw(20)<<"j"<<setw(20)<<"k"<<setw(20)<<"l"<<endl;
myfile4<<"a"<<setw(20)<<"b"<<setw(20)<<"c"<<setw(20)<<"d"<<setw(20)<<"e"<<setw(20)<<"f"<<setw(20)<<"g"<<setw(20)<<"h"<<setw(20)<<"i"<<setw(20)<<"j"<<setw(20)<<"k"<<setw(20)<<"l"<<endl;
double r, rd, rdd, rdes, rdesd, rdesdd, x, xd, xdd, xdes, xdesd, xdesdd, y, yd, ydd, ydes, ydesd, ydesdd, z=2, zd, zdd, zdes, zdesd, zdesdd;
double p, pd, pdd, pdes, pdesd, pdesdd, t, td, tdd, tdes, tdesd, tdesdd;
double k=.1, kphi=5.25, ktheta=0.5, kz=3;

//double k=1, kphi=6 , ktheta=0.5, kz=3;

double E;
//double k=0.4, kphi=0.2, ktheta=0.1;

double tauf=-5;
double taufp=-4;
double xdt, ydt, zdt;
double k1=k*tauf;
double k2=kphi*taufp;
double vxdt, vydt, vzdt;
double X, Y;
double lx, ly, lz;

double C, D;
moveup(10);

int i=1;
int jj=1;

double fufu=0;
while (jj<300)

{

ros::Rate loop_rate(200);
move(0.25, 0.25, -0.3,0,0,0);
cout<<jj;

jj=jj+1;
ros::spinOnce();
loop_rate.sleep();

}
double dupx;
double dupy;
int count=1;
double RR0=0, TT0=0;
double dafu=0,dipa=0, stage=1, po=6;
do
	{

if (i>5)
{
//        move2(0.125,0,0,0,0,0);

//        			if (abs(r)<3)
//               	               {
//               	               	k1=(1/2.8)*(r-3)-1;
//               	               	if (k1<-2)
//               	               	{
//               	               		k1=-2;
//               	               	}
//               	               }
//               	        else if (abs(r)>3)
//               	        {
//               	        	k1=-1;
//               	        }
        if ( stage == 1 )
        {
        	 	 	x=gdrone.pose.pose.position.x;
        			y=gdrone.pose.pose.position.y;
        			z=gdrone.pose.pose.position.z-2;
        			xdes=0;
        			ydes=0;
        			zdes=0;
        			rdes=sqrt((xdes)*(xdes)+(ydes)*(ydes)+(zdes)*(zdes));
        	        r=sqrt((x)*(x)+(y)*(y)+(z)*(z));
//        	        rdes=0;
        			xd=gdrone.twist.twist.linear.x;
        			yd=gdrone.twist.twist.linear.y;
        	        zd=gdrone.twist.twist.linear.z;
        	        pdes=180*3.14/180;
        	        double hjh=3.14-(pdes);
        	        xdt=cos(hjh)*x-sin(hjh)*y;
        	        ydt=sin(hjh)*x+cos(hjh)*y;
        	        vxdt=cos(hjh)*xd-sin(hjh)*yd;
        	        vydt=sin(hjh)*xd+cos(hjh)*yd;
        		    xdesd=0;
        		    ydesd=0;
        		    zdesd=0;
        	        rd= (((x)*(xd)+(y)*(yd)+(z)*(zd))/r);
        	        rdesd=0;
        	        rdesdd=0;
        				    if (abs(x-xdes) > 0.2 || abs(y-ydes)> 0.2)
        				    {
//        				    	cout<<"one"<<setw(20);
        //							    r=sqrt((mitx)*(mitx)+(mity)*(mity));

        			//				    		xd=drone_navdata.vx*0.001;
        			//				    		yd=drone_navdata.vy*0.001;
        			//				            zd=drone_navdata.vz*0.001;

        			//				            if ((abs(r-rdes)< 5) || si==1)
        			//				            {
        			//				            	si=1;
        			//				            	k1=k1-0.002;
        			//				            }
        			//				            if (k1<-0.75)
        			//				            {
        			//				            	k1=-.75;
        			//				            }

        			//				            k1=kafu;
        							            rdd=rdesdd+(rd-rdesd)*(rd-rdesd)/(r-rdes)+k1*(r-rdes)-k*(rd-rdesd);
        							            double luk=0;
        							            luk=k1;
        							            //0.65 , 1  , 0.7
        			//				            if (rdd>-0.5)
        			//				            {
        			//				            	rdd=-0.5;
        			//				            	luk=(-3-(rd)*rd/(r-rdes)+k*(rd))/(r-rdes);
        			//				            }

        							            //cout<<r<<endl;
        							         p=atan2(y,x);

        							         pd=((x*yd-y*xd)/(x*x+y*y));
        							         pdesd=0;
        							         pdesdd=0;
        							         t=atan2(z,sqrt(x*x+y*y));
        							         tdes=90*3.14/180;
        							         tdesd=0;
        							         zdd=0;
        							         pdd=pdesdd+(kphi*(k1-k*(rd-rdesd)/(r-rdes))*(p-pdes)*(p-pdes)+(pd-pdesd)*(pd-pdesd))/(p-pdes);
        			//        				 pdd=0;
        							         if (abs(ydt)<0.2)
        							         {
        							        	 pdd=0;
        							        			 cout<<"two"<<setw(20);
        							         }
        							         else
        							         {
        							        	 cout<<"one"<<setw(20);
        							         }

double Xtu,Ytu;
        							         Xtu=rdd*r+rd*rd-xd*xd-yd*yd;
        							         Ytu=pdd*(x*x+y*y)+2*((x*yd-y*xd)*(x*xd+y*yd))/(x*x+y*y);
        							         xdd=(x*Xtu-y*Ytu)/(x*x+y*y);
        							         ydd=(y*Xtu+x*Ytu)/(x*x+y*y);
        							         C=tdesd+ktheta*(rd-rdesd)*(t-tdes)/(r-rdes);
        							    //     C=tdes+((TT0)*(pow((r-rd),(1/ktheta -1)))/(ktheta*(pow((RR0),(1/ktheta)))) );
        							         D=(C*(x*x+y*y+z*z)+z*(x*xd+y*yd)/sqrt(x*x+y*y))/sqrt(x*x+y*y);
//        							    D= -ktheta*(z-zdes)*abs((rd-rdesd)/(r-rdes));

        							    //if (abs(r)<4)
        							    //{
        							    // 	 tauf=tauf-0.1;
        							    //	  if (tauf<-10)
        							    //	  {
        							    //	 	 tauf=-10;
        							    //	  }
        							    //}

        							    //          D=(z*(x*xd+y*yd)/sqrt(x*x+y*y))/sqrt(x*x+y*y);
        							    //     E=zdesd+(z-zdes)*kz*(rd-rdesd)/(r-rdes);
        							         double ply=-ydd/10;

        							         //     cout<<ply<<setw(20);
        							         if (ply>1)
        							         {
        							        	 ply=1;
        							         }
        							         if (ply<-1)
        							         {
        							        	 ply=-1;
        							         }
        							         ly=-asin(ply);
        							         double plx=xdd/(10*cos(ly));
        							    //     cout<<plx<<endl;
        							         if (plx>1)
        							            {
        							           	 plx=1;
        							            }
        							            if (plx<-1)
        							            {
        							           	 plx=-1;
        							            }
        							         lx=asin(plx);
        							    //     if (abs(x-xdes)<0.15)
        							    //         		 {
        							    //         	 lx=0;
        							    //         		 }
        							    //          if (abs(y-ydes)<0.15)
        							    //          {
        							    //         	 ly=0;
        							    //          }
        							         lz=D;
        							         dupx=x;
        							         dupy=y;
        							         move(lx,ly,lz,0,0,0);
        							              myfile<<x<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ly<<setw(20)<<z+2<<setw(20)<<lz<<setw(20)<<xdt<<setw(20)<<ydt<<endl;
        							         //     myfile<<r<<setw(20)<<luk<<endl;
        							         //     myfile<<x<<setw(20)<<y<<setw(20)<<z<<setw(20)<<lx<<setw(20)<<ly<<setw(20)<<lz<<setw(20)<<luk<<endl;
        							              cout<<ydt<<setw(20)<<x-xdes<<setw(20)<<lx<<setw(20)<<y-ydes<<setw(20)<<ly<<setw(20)<<z-zdes<<setw(20)<<lz<<endl;

        							    }
        				    else
        				    {
        				    	stage=2;
//        				    dipa=1;
        				    }
        }
        if (stage==2)
        		{
//			z=gdrone.pose.pose.position.z;
        	x=gdrone.pose.pose.position.x;
        	        		                		        			y=gdrone.pose.pose.position.y;
        	        		                		        			z=gdrone.pose.pose.position.z;
        	        		                		        			xdes=0;
        	        		                		        			ydes=0;
        	        		                		        			zdes=0.5;
        	        		                		        			rdes=sqrt((xdes)*(xdes)+(ydes)*(ydes)+(zdes)*(zdes));
        	        		                		        	        r=sqrt((x)*(x)+(y)*(y)+(z)*(z));
        	        		                		//        	        rdes=0;
        	        		                		        			xd=gdrone.twist.twist.linear.x;
        	        		                		        			yd=gdrone.twist.twist.linear.y;
        	        		                		        	        zd=gdrone.twist.twist.linear.z;
        	        		                		        		    xdesd=0;
        	        		                		        		    ydesd=0;
        	        		                		        		    zdesd=0;
        	        		                		        	        rd= (((x)*(xd)+(y)*(yd)+(z)*(zd))/r);
        	        		                		        	        rdesd=0;
        	        		                		        	        rdesdd=0;
        	if (abs(z-zdes)>0.25)
        	{

        	cout<<"three"<<setw(20)<<dipa;


        			 							    //  k=k+0.3;
        			 							    //  if (k>10)
        			 							    //  {
        			 							    // 	 k=10;
        			 							    //  }
        			 							     	 fufu=1;
        			 							     	 D=-0.1*(z-zdes);
        			 							    // 	 D=-0.125*(z-zdes);

        			  lz=D;
//        			  							         if (abs(z)<1.1)
//        			  							         {
//        			  							        	 tu=tu+1;
//        			  							         }
        			  							         lx=-0.4*(x-dupx);
        			  							         ly=-0.4*(y-dupy);
//        			  							         ly=0;
        			  							       move(lx,ly,lz,0,0,0);
        			  							            myfile<<x<<setw(20)<<lx<<setw(20)<<y<<setw(20)<<ly<<setw(20)<<z<<setw(20)<<lz<<endl;
        			  							       //     myfile<<r<<setw(20)<<luk<<endl;
        			  							       //     myfile<<x<<setw(20)<<y<<setw(20)<<z<<setw(20)<<lx<<setw(20)<<ly<<setw(20)<<lz<<setw(20)<<luk<<endl;
        			  							            cout<<x-xdes<<setw(20)<<lx<<setw(20)<<y-ydes<<setw(20)<<ly<<setw(20)<<z-zdes<<setw(20)<<lz<<endl;
        	}
        	else if (abs(z-zdes)<0.25)
        	{
        	dipa=1;
        	cout<<"fuck"<<endl;
        	}
        		}



//     cout<<X<<setw(20)<<rdd<<setw(20)<<x<<setw(20)<<lx<<setw(20)<<luk<<endl;
}
i=i+1;
count=count+1;
  //   move(lx,ly,lz,0,0,0);
//     cout<<X<<setw(20)<<Y<<setw(20)<<lx<<setw(20)<<ly<<setw(20)<<lz<<endl;
       ros::spinOnce();
		loop_rate.sleep();
		}while(ros::ok() && dipa==0);
//&& buduk==0
//			fBuzz();
			cout<<"land";
		//	myfile.close();
			land();


		}




// Function to get cofactor of A[p][q] in temp[][]. n is current
// dimension of A[][]
void getCofactor(double A[N][N], double temp[N][N], int p, int q, int n)
{
	int i = 0, j = 0;

	// Looping for each element of the matrix
	for (int row = 0; row < n; row++)
	{
		for (int col = 0; col < n; col++)
		{
			// Copying into temporary matrix only those element
			// which are not in given row and column
			if (row != p && col != q)
			{
				temp[i][j++] = A[row][col];

				// Row is filled, so increase row index and
				// reset col index
				if (j == n - 1)
				{
					j = 0;
					i++;
				}
			}
		}
	}
}

/* Recursive function for finding determinant of matrix.
n is current dimension of A[][]. */
double determinant(double A[N][N], int n)
{
	double D = 0; // Initialize result

	// Base case : if matrix contains single element
	if (n == 1)
		return A[0][0];

	double temp[N][N]; // To store cofactors

	int sign = 1; // To store sign multiplier

	// Iterate for each element of first row
	for (int f = 0; f < n; f++)
	{
		// Getting Cofactor of A[0][f]
		getCofactor(A, temp, 0, f, n);
		D += sign * A[0][f] * determinant(temp, n - 1);

		// terms are to be added with alternate sign
		sign = -sign;
	}

	return D;
}

// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint(double A[N][N],double adj[N][N])
{
	if (N == 1)
	{
		adj[0][0] = 1;
		return;
	}

	// temp is used to store cofactors of A[][]
	int sign = 1;
        double temp[N][N];

	for (int i=0; i<N; i++)
	{
		for (int j=0; j<N; j++)
		{
			// Get cofactor of A[i][j]
			getCofactor(A, temp, i, j, N);

			// sign of adj[j][i] positive if sum of row
			// and column indexes is even.
			sign = ((i+j)%2==0)? 1: -1;

			// Interchanging rows and columns to get the
			// transpose of the cofactor matrix
			adj[j][i] = (sign)*(determinant(temp, N-1));
		}
	}
}

// Function to calculate and store inverse, returns false if
// matrix is singular
bool inverse(double A[N][N], double inverse[N][N])
{
	// Find determinant of A[][]
	double det = determinant(A, N);
	if (det == 0)
	{
		cout << "Singular matrix, can't find its inverse";
		return false;
	}

	// Find adjoint
	double adj[N][N];
	adjoint(A, adj);

	// Find Inverse using formula "inverse(A) = adj(A)/det(A)"
	for (int i=0; i<N; i++)
		for (int j=0; j<N; j++)
			inverse[i][j] = adj[i][j]/double(det);

	return true;
}

// Generic function to display the matrix. We use it to display
// both adjoin and inverse. adjoin is integer matrix and inverse
// is a float.
template<class T>
void display(T A[N][N])
{
	for (int i=0; i<N; i++)
	{
		for (int j=0; j<N; j++)
			cout << A[i][j] << " ";
		cout << endl;
	}
}

// Generic function to display the matrix.  We use it to display
// both adjoin and inverse. adjoin is integer matrix and inverse
// is a float.
// dimension of A[][]
static void matinv(double *a,double *inv)
{
    double det=((*(a+0))*(*(a+5))*(*(a+10))*(*(a+15))) + ((*(a+0))*(*(a+6))*(*(a+11))*(*(a+13))) + ((*(a+0))*(*(a+7))*(*(a+9))*(*(a+14)))
             + ((*(a+1))*(*(a+4))*(*(a+11))*(*(a+14))) + ((*(a+1))*(*(a+6))*(*(a+8))*(*(a+15))) + ((*(a+1))*(*(a+7))*(*(a+10))*(*(a+12)))
             + ((*(a+2))*(*(a+4))*(*(a+9))*(*(a+15))) + ((*(a+2))*(*(a+5))*(*(a+11))*(*(a+12))) + ((*(a+2))*(*(a+7))*(*(a+8))*(*(a+13)))
             + ((*(a+3))*(*(a+4))*(*(a+10))*(*(a+13))) + ((*(a+3))*(*(a+5))*(*(a+8))*(*(a+14))) + ((*(a+3))*(*(a+6))*(*(a+9))*(*(a+12)))
             - ((*(a+0))*(*(a+5))*(*(a+11))*(*(a+14))) - ((*(a+0))*(*(a+6))*(*(a+9))*(*(a+15))) - ((*(a+0))*(*(a+7))*(*(a+10))*(*(a+13)))
             - ((*(a+1))*(*(a+4))*(*(a+10))*(*(a+15))) - ((*(a+1))*(*(a+6))*(*(a+11))*(*(a+12))) - ((*(a+1))*(*(a+7))*(*(a+8))*(*(a+14)))
             - ((*(a+2))*(*(a+4))*(*(a+11))*(*(a+13))) - ((*(a+2))*(*(a+5))*(*(a+8))*(*(a+15))) - ((*(a+2))*(*(a+7))*(*(a+9))*(*(a+12)))
             - ((*(a+3))*(*(a+4))*(*(a+9))*(*(a+14))) - ((*(a+3))*(*(a+5))*(*(a+10))*(*(a+12))) - ((*(a+3))*(*(a+6))*(*(a+8))*(*(a+13)));

    double b11=((*(a+5))*(*(a+10))*(*(a+15))) + ((*(a+6))*(*(a+11))*(*(a+13))) + ((*(a+7))*(*(a+9))*(*(a+14))) -
               ((*(a+5))*(*(a+11))*(*(a+14))) - ((*(a+6))*(*(a+9))*(*(a+15))) - ((*(a+7))*(*(a+10))*(*(a+13)));
    double b12=((*(a+1))*(*(a+11))*(*(a+14))) + ((*(a+2))*(*(a+9))*(*(a+15))) + ((*(a+3))*(*(a+10))*(*(a+13))) -
               ((*(a+1))*(*(a+10))*(*(a+15))) - ((*(a+2))*(*(a+11))*(*(a+13))) - ((*(a+3))*(*(a+9))*(*(a+14)));
    double b13=((*(a+1))*(*(a+6))*(*(a+15))) + ((*(a+2))*(*(a+7))*(*(a+13))) + ((*(a+3))*(*(a+5))*(*(a+14))) -
               ((*(a+1))*(*(a+7))*(*(a+14))) - ((*(a+2))*(*(a+5))*(*(a+15))) - ((*(a+3))*(*(a+6))*(*(a+13)));
    double b14=((*(a+1))*(*(a+7))*(*(a+10))) + ((*(a+2))*(*(a+5))*(*(a+11))) + ((*(a+3))*(*(a+6))*(*(a+9))) -
               ((*(a+1))*(*(a+6))*(*(a+11))) - ((*(a+2))*(*(a+7))*(*(a+9))) - ((*(a+3))*(*(a+5))*(*(a+10)));
    double b21=((*(a+4))*(*(a+11))*(*(a+14))) + ((*(a+6))*(*(a+8))*(*(a+15))) + ((*(a+7))*(*(a+10))*(*(a+12))) -
               ((*(a+4))*(*(a+10))*(*(a+15))) - ((*(a+6))*(*(a+11))*(*(a+12))) - ((*(a+7))*(*(a+8))*(*(a+14)));
    double b22=((*(a+0))*(*(a+10))*(*(a+15))) + ((*(a+2))*(*(a+11))*(*(a+12))) + ((*(a+3))*(*(a+8))*(*(a+14))) -
               ((*(a+0))*(*(a+11))*(*(a+14))) - ((*(a+2))*(*(a+8))*(*(a+15))) - ((*(a+3))*(*(a+10))*(*(a+12)));
    double b23=((*(a+0))*(*(a+7))*(*(a+14))) + ((*(a+2))*(*(a+4))*(*(a+15))) + ((*(a+3))*(*(a+6))*(*(a+12))) -
               ((*(a+0))*(*(a+6))*(*(a+15))) - ((*(a+2))*(*(a+7))*(*(a+12))) - ((*(a+3))*(*(a+4))*(*(a+14)));
    double b24=((*(a+0))*(*(a+6))*(*(a+11))) + ((*(a+2))*(*(a+7))*(*(a+8))) + ((*(a+3))*(*(a+4))*(*(a+10))) -
               ((*(a+0))*(*(a+7))*(*(a+10))) - ((*(a+2))*(*(a+4))*(*(a+11))) - ((*(a+3))*(*(a+6))*(*(a+8)));
    double b31=((*(a+4))*(*(a+9))*(*(a+15))) + ((*(a+5))*(*(a+11))*(*(a+12))) + ((*(a+7))*(*(a+8))*(*(a+13))) -
               ((*(a+4))*(*(a+11))*(*(a+13))) - ((*(a+5))*(*(a+8))*(*(a+15))) - ((*(a+7))*(*(a+9))*(*(a+12)));
    double b32=((*(a+0))*(*(a+11))*(*(a+13))) + ((*(a+1))*(*(a+8))*(*(a+15))) + ((*(a+3))*(*(a+9))*(*(a+12))) -
               ((*(a+0))*(*(a+9))*(*(a+15))) - ((*(a+1))*(*(a+11))*(*(a+12))) - ((*(a+3))*(*(a+8))*(*(a+13)));
    double b33=((*(a+0))*(*(a+5))*(*(a+15))) + ((*(a+1))*(*(a+7))*(*(a+12))) + ((*(a+3))*(*(a+4))*(*(a+13))) -
               ((*(a+0))*(*(a+7))*(*(a+13))) - ((*(a+1))*(*(a+4))*(*(a+15))) - ((*(a+3))*(*(a+5))*(*(a+12)));
    double b34=((*(a+0))*(*(a+7))*(*(a+9))) + ((*(a+1))*(*(a+4))*(*(a+11))) + ((*(a+3))*(*(a+5))*(*(a+8))) -
               ((*(a+0))*(*(a+5))*(*(a+11))) - ((*(a+1))*(*(a+7))*(*(a+8))) - ((*(a+3))*(*(a+4))*(*(a+9)));
    double b41=((*(a+4))*(*(a+10))*(*(a+13))) + ((*(a+5))*(*(a+8))*(*(a+14))) + ((*(a+6))*(*(a+9))*(*(a+12))) -
               ((*(a+4))*(*(a+9))*(*(a+14))) - ((*(a+5))*(*(a+10))*(*(a+12))) - ((*(a+6))*(*(a+8))*(*(a+13)));
    double b42=((*(a+0))*(*(a+9))*(*(a+14))) + ((*(a+1))*(*(a+10))*(*(a+12))) + ((*(a+2))*(*(a+8))*(*(a+13))) -
               ((*(a+0))*(*(a+10))*(*(a+13))) - ((*(a+1))*(*(a+8))*(*(a+14))) - ((*(a+2))*(*(a+9))*(*(a+12)));
    double b43=((*(a+0))*(*(a+6))*(*(a+13))) + ((*(a+1))*(*(a+4))*(*(a+14))) + ((*(a+2))*(*(a+5))*(*(a+12))) -
               ((*(a+0))*(*(a+5))*(*(a+14))) - ((*(a+1))*(*(a+6))*(*(a+12))) - ((*(a+2))*(*(a+4))*(*(a+13)));
    double b44=((*(a+0))*(*(a+5))*(*(a+10))) + ((*(a+1))*(*(a+6))*(*(a+8))) + ((*(a+2))*(*(a+4))*(*(a+9))) -
               ((*(a+0))*(*(a+6))*(*(a+9))) - ((*(a+1))*(*(a+4))*(*(a+10))) - ((*(a+2))*(*(a+5))*(*(a+8)));

    *(inv+0)=b11/det;*(inv+1)=b12/det;*(inv+2)=b13/det;*(inv+3)=b14/det;
    *(inv+4)=b21/det;*(inv+5)=b22/det;*(inv+6)=b23/det;*(inv+7)=b24/det;
    *(inv+8)=b31/det;*(inv+9)=b32/det;*(inv+10)=b33/det;*(inv+11)=b34/det;
    *(inv+12)=b41/det;*(inv+13)=b42/det;*(inv+14)=b43/det;*(inv+15)=b44/det;
}


void min_jerk(double x_d, double y_d, double z0, double Tf)
{
	ax = 0;
	ay = 0;
	az = 0;

	//az = 0;

	//Measuring t : current time
	double t0 = ros::Time::now().toSec();
	double e_x, e_y, e_z,e_x0, e_y0, e_z0, x=0, y=0,z,r, i=0,v;


	double g,x0,y0, u0,uy0,v0,ax0,ay0,az0,xf,yf, zf,uf,uyf,vf,axf,ayf,azf, tgo, u1, uy1, u2,r0,c1x,c2x,c3x,c1y, c2y, c3y, c1z,c2z,c3z,Kc = 1, dxg, dxg0, dyg, dyg0, dzg,dzg0,xg,xg0,yg, yg0, zg,zg0, zi0;
	//cout<<endl<<r0;

	ros::Rate loop_rate(200);

	ofstream myfile;
	myfile.open("/home/icgel/jerk_test.txt", ios::out | ios::app );
	//myfile<<"Entering the loop";


g=9.81;


v0=0;
ax0=0;
ay0=0;
az0=-g;
xf=0;
yf=0;
zf=0.5;
uf=0;
uyf=0;
vf=0;
axf=0;
ayf=0;
azf=-g;
dxg0 = 0;
dzg0 = v0;
xg0 = 0;
zg0=z0;
zg =zg0;
double Vx=0, Vy=0, Vz=0;
double Vxdot,Vydot,Vzdot;
double C=-0.25/0.38;
double CTP = -0.1/0.38;
double d2r = 3.14159265/180.0;
double theta0,phi0,psi0;
int hhh=getch();
double P[6][6]={{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
  double H1[4][6]={{0,0,1,0,0,0},{0,0,0,C,0,0},{0,0,0,0,C,0} ,{0,0,0,0,0,0}   };
 double H2[6][6]={{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,C,0,0},{0,0,0,0,C,0},{0,0,0,0,0,0}};
  double K[6][6];
  double KK[6][4]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  double ident[6][6]={{1,0,0,0,0,0},
                       {0,1,0,0,0,0},
                       {0,0,1,0,0,0},
                       {0,0,0,1,0,0},
                       {0,0,0,0,1,0},
                       {0,0,0,0,0,1}};
     //double A[6][6];
  //double td0=td;
  double qu=3;
  double rui=3;
  double rus=3;
  double rh=0.1;
     double Q[6][6]={{qu,0,0,0,0,0},
  		            {0,qu,0,0,0,0},
  		            {0,0,qu,0,0,0},
  		            {0,0,0,qu,0,0},
  		            {0,0,0,0,qu,0},
  		            {0,0,0,0,0,qu}};
    double R1[6][6]={{rus,0,0,0,0,0},{0,rus,0,0,0,0},{0,0,rh,0,0,0},{0,0,0,rus,0,0},{0,0,0,0,rus,0},{0,0,0,0,0,rh}};
     double R2[4][4]={rh,0,0,0, 0,rui,0,0,0,0,rui,0,0,0,0,rh};
     double J[4][4];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J1[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J2[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  };
     double J3[6][6];//={  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
     double J33[6][4];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J4[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J44[4][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J5[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J55[4][4];//={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
     double J6[6][6];//={ 1,2,3,4,3,7,1,8,5,78,4,7,3,4,2,1 };
//       double J6[4][4]={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J66[4][4];//={0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
     double J7[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J8[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 ,0,0,0,0 };
     double J9[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J10[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J15[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J16[6][6];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J11[6][4];//={ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
     double J12[6][6];//={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  };
     double Y[4][1];//={0,0,0,0};
     double Y1[6][1];//={0,0,0,0};
     double YY[6][1];//={0,0,0,0,0,0};
     double Y2[4][1];//={0,0,0,0};
     double HX1[4][1];//={0,0,0,0};
     double HX2[4][1];//={0,0,0,0};

		double adj[N][N]; // To store adjoint of A[][]

		double inv[N][N];
		double invh[4][4];// To store inverse of A[][]

		do
             {


								{
						ros::Rate loop_rate(20);//loop 50 hz
						theta0=(drone_navdata.rotY)*d2r;
						phi0=(drone_navdata.rotX)*d2r;
						psi0=(drone_navdata.rotZ)*d2r;
						 double oioi=drone_navdata.altd*0.001;
//							    double oioi=0.5;
								double hx=oioi*cos(theta0);
								double hy=oioi*cos(phi0);
								double Bx=atan(tan(35*d2r/2)*(180-redX)/180);
								double By=atan(tan(45*d2r/2)*(redY-320)/320);
								double hemaX=hx*tan(theta0+Bx);
								double hemaY=hy*tan(phi0+By);
						x0=hemaX;
						y0=hemaY;
						z0=drone_navdata.altd*0.001-0.7;
						u0=drone_navdata.vx*0.001;
						uy0=drone_navdata.vy*0.001;
						loop_rate.sleep();
						ros::spinOnce();

								}
				}
		while(redX==-1);
		cout<<"Yeahhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh"<<endl;
       double hr=sqrt(x0*x0+y0*y0+z0*z0);
//       Tf=2*hr;
       Tf=8;
       double rtt=0;
       double gt=0;
      // ros::Rate loop_rate(200);
    	do
{
	double theta=(drone_navdata.rotY)*d2r-theta0 ,phi=(drone_navdata.rotX)*d2r - phi0, psi=(drone_navdata.rotZ)*d2r-psi0;;
		//double ax = drone_navdata.ax-ax0, ay = drone_navdata.ay-ay0, az = ang.linear_acceleration.z;

	 double oioi=drone_navdata.altd*0.001;
	//    double oioi=0.5;
		double hx=oioi*cos(theta);
		double hy=oioi*cos(phi);
		double Bx=atan(tan(35*d2r/2)*(180-redX)/180);
		double By=atan(tan(45*d2r/2)*(redY-320)/320);
		double hemaX=hx*tan(theta+Bx);
		double hemaY=hy*tan(phi+By);
		//cout<<hemaX<<setw(20)<<hemaY<<endl;
		x=x+ (cos(psi)*cos(theta)*Vx+(cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi))*Vy + (- sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))*Vz)/200;
		    y=y+(cos(theta)*sin(psi)* Vx +(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*Vy  +(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*Vz )/200;
		    z=z+((sin(theta))*Vx - ( -cos(theta)*sin(phi))*Vy + (cos(phi)*cos(theta))*Vz )/200;
		    Vxdot=9.66*(sin(theta))+C*Vx;
		    Vydot=-9.66*(sin(phi)*cos(theta))+C*Vy;
		    Vzdot=9.66*cos(theta)*cos(phi) + CTP;
	Vx=Vx+Vxdot/200;
	Vy=Vy+Vydot/200;
	Vz=Vz+Vzdot/200;
	double A[6][6]={0,0,0,cos(psi)*cos(theta),cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), - sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta) ,
	    0,0,0,cos(theta)*sin(psi),cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta),   cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta) ,
	    0,0,0,  sin(theta),-cos(theta)*sin(phi) , cos(phi)*cos(theta) ,
	    0,0,0,C,0,0,
	    0,0,0,0,C,0,
	    0,0,0,0,0,0};
	for (int ji=0; ji<6; ji++)
	    {
	    	for (int li=0;li<6;li++)
	    	{
	              J1[ji][li]=0;
	              J2[ji][li]=0;
	    		for (int pi=0;pi<6;pi++)
	    		{
	    			J1[ji][li]=J1[ji][li]+(A[ji][pi] * P[pi][li]);  //AP
	    			J2[ji][li]=J2[ji][li]+(P[ji][pi] * A[li][pi]);   // PA'
	    		}
	           P[ji][li]=P[ji][li]+(J1[ji][li]  + J2[ji][li]+ Q[ji][li] )/200;
	    	}
	    }
		if (redX ==-1 & rtt != 0 )
		{
			  double cx=-0.27/0.38, cy=-0.27/0.38;

			      	for (int ji=0; ji<6; ji++)
			      	    {
			      	    	for (int li=0;li<4;li++)
			      	    	{
			                      J33[ji][li]=0;
			       	    		for (int pi=0;pi<6;pi++)
			      	    		{
			      	    			J33[ji][li]=J33[ji][li]+(P[ji][pi]*H1[li][pi]);//PC'
			       	    		}
			       	    	}
			      	    }
			      	for (int ji=0; ji<4; ji++)
			      	    	    {
			      	    	    	for (int li=0;li<6;li++)
			      	    	    	{
			      	                    J44[ji][li]=0;

			      	    	    		for (int pi=0;pi<6;pi++)
			      	    	    		{
			      	    	    			J44[ji][li]=J44[ji][li]+(H1[ji][pi]*P[pi][li]);//CP
			      	    	    		}
			      	    	    	}
			      	    	    }
			      	for (int ji=0; ji<4; ji++)
			      	    	    {
			      	    	    	for (int li=0;li<4;li++)
			      	    	    	{
			      	                    J55[ji][li]=0;
			  //
			      	                    J66[ji][li]=0;
			      	    	    		for (int pi=0;pi<6;pi++)
			      	    	    		{
			      	    	    			J55[ji][li]=J55[ji][li]+(J44[ji][pi]*H1[li][pi]);//CPC'

			      	    	    		}
			      	    	          	J66[ji][li]=J55[ji][li]+R2[ji][li];//CPC' + R
			      	    	    	}
			      	    	    }
			          	 matinv((double *)J66,(double *)invh);
			         					    for (int ji=0; ji<6; ji++)
			      					        	    {
			      					        	    	for (int li=0;li<4;li++)
			      					        	    	{
			      					                        KK[ji][li]=0;
			      					        	    		for (int pi=0;pi<4;pi++)
			      					        	    		{
			      					        	    			KK[ji][li]=KK[ji][li]+(J33[ji][pi]*invh[pi][li]);
			      					        	    		}
			      					    //

			      					        	    	}
			      					        	    }
			      					 //   myfile2<<KK[0][0]<<setw(20);
			      					    Y2[0][0]=drone_navdata.altd*0.001-z;
			      					    Y2[1][0]=ax*10-cx*Vx;
			      					    Y2[2][0]=ay*10-cy*Vy;
			      					    Y2[3][0]=az-CTP;
			//myfile<<ax<<setw(20)<<ay<<setw(20);
			      	for (int ji=0; ji<6; ji++)
			    {
			      		YY[ji][0]=0;
			         for (int li=0;li<4;li++)
			     {
			     YY[ji][0]=YY[ji][0]+KK[ji][li]*Y2[li][0];
			     }
			    }
			      	x=x+YY[0][0];
			      	y=y+YY[1][0];
			      	z=z+YY[2][0];
			      //	cout<<x<<setw(20)<<y<<endl;
			      	Vx=Vx+YY[3][0];
			      	Vy=Vy+YY[4][0];
			      	Vz=Vz+YY[5][0];
			//        	if (abs(Vx-prev_vx) >0.5)
			//        	{
			//        		//cout<<"PADDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDS"<<endl;
			//        		Vx=prev_vx;
			//        	}
			//        	if (abs(Vy-prev_vy) > 0.5)
			//        	{
			//        		Vy=prev_vy;
			//        	}
			      	//cout<<"imuuuuuu"<<x<<setw(20)<<y<<endl;
			      	//cout<<drone_navdata.vx*0.001<<setw(20)<<Vx<<endl;
			      	myfile<<(theta)<<setw(20)<<(phi)<<setw(20)<<psi<<setw(20)<<ax<<setw(20)<<ay<<setw(20)<<az<<setw(20)<<Vx<<setw(20)<<Vy<<setw(20)<<drone_navdata.vx*0.001<<setw(20)<<drone_navdata.vy*0.001<<setw(20)<<drone_navdata.vz*0.001<<setw(20)<<drone_navdata.altd*0.001<<endl;
			      	//myfile2<<
			      	//myfile2<<x<<setw(20)<<y<<setw(20)<<P[0][0]<<setw(20)<<P[1][1]<<endl;
			      	for (int ji=0; ji<6; ji++)
			    {
			          for (int li=0;li<6;li++)
			   {
			          J8[ji][li]=0;
			          for (int pi=0;pi<4;pi++)
			      	   {
			          	J8[ji][li]=J8[ji][li]+(KK[ji][pi]*H1[pi][li]); //KC
			      	   }
			          J8[ji][li]=ident[ji][li]-J8[ji][li];

			   }
			    }

			      	for (int ji=0; ji<6; ji++)
			      	  {
			      	        for (int li=0;li<6;li++)
			      	 {
			      	        J9[ji][li]=0;
			      	        for (int pi=0;pi<6;pi++)
			      	        {
			      	        	J9[ji][li]=J9[ji][li]+(J8[ji][pi]*P[pi][li]);
			      	        }
			      	 }
			      	  }
			      	for (int ji=0; ji<6; ji++)
			      	    	  {
			      	    	        for (int li=0;li<6;li++)
			      	    	 {
			      	    	        J10[ji][li]=0;
			      	    	        for (int pi=0;pi<6;pi++)
			      	    	        {
			      	    	        	J10[ji][li]=J10[ji][li]+(J9[ji][pi]*J8[li][pi]);
			      	    	        }
			      	    	 }
			      	    	  }

			      	for (int ji=0; ji<6; ji++)
			      	    	    	  {
			      	    	    	        for (int li=0;li<4;li++)
			      	    	    	 {
			      	    	    	        J11[ji][li]=0;
			      	    	    	        for (int pi=0;pi<4;pi++)
			      	    	    	        {

			      	    	    	        	J11[ji][li]= J11[ji][li]+ (KK[ji][pi]*R2[pi][li]);

			      	    	    	        }
			      	    	    	 }
			      	    	    	  }
			      	for (int ji=0; ji<6; ji++)
			      	    	    	    	  {
			      	    	    	    	        for (int li=0;li<6;li++)
			      	    	    	    	 {
			      	    	    	    	        J12[ji][li]=0;
			      	    	    	    	        for (int pi=0;pi<4;pi++)
			      	    	    	    	        {
			      	    	    	    	        	J12[ji][li]=J12[ji][li]+(J11[ji][pi]*KK[li][pi]);

			      	    	    	    	        }
			      	    	    	    	        P[ji][li]=J10[ji][li]+J12[ji][li];

			      	    	    	    	 }
			      	    	    	    	  }
			      	//myfile2<<P[0][0]<<endl;
			  //    	for (int ji=0; ji<4; ji++)
			  //    	    	  {
			  //    	    	        for (int li=0;li<4;li++)
			  //    	    	 {
			  //    	    	  P[ji][li]=J9[ji][li];
			  //    	    	 }
			  //    	    	 }
		}
		else if (redX != -1)
		{
			   double cx=-0.25/0.38, cy=-0.25/0.38;
			  	qu=3;
			  	        double Q[6][6]={{qu,0,0,0,0,0},{0,qu,0,0,0,0},{0,0,qu,0,0,0},{0,0,0,qu,0,0},{0,0,0,0,qu,0},{0,0,0,0,0,qu}};
			  	//cout<<"slammmmmmmmmmmmmmmmmmmmmmmmmmmmmmm";
			  	for (int ji=0; ji<6; ji++)
			  	    {
			  	    	for (int li=0;li<6;li++)
			  	    	{
			                  J3[ji][li]=0;
			                  J4[ji][li]=0;
			  	    		for (int pi=0;pi<4;pi++)
			  	    		{
			  	    			J3[ji][li]=J3[ji][li]+(P[ji][pi]*H2[li][pi]);//PC'
			  	    			J4[ji][li]=J4[ji][li]+(H2[ji][pi]*P[pi][li]);//CP

			  	    		}
			//

			  	    	}
			  	    }
			  	for (int ji=0; ji<6; ji++)
			  	    	    {
			  	    	    	for (int li=0;li<6;li++)
			  	    	    	{
			  	                    J5[ji][li]=0;
			  	                    K[ji][li]=0;
			  	                    J6[ji][li]=0;
			  	    	    		for (int pi=0;pi<6;pi++)
			  	    	    		{
			  	    	    			J5[ji][li]=J5[ji][li]+(J4[ji][pi]*H2[li][pi]);//CPC'

			  	    	    		}
			  	    	           	J6[ji][li]=J5[ji][li]+R1[ji][li];//CPC' + R

			  	    	    	}
			  	    	    }
			//    	J6={1,2,3,4,3,7,1,8,5,78,4,7,3,4,2,1};
			  	// cout << "\nThe Inverse is :\n";
			//    					    if (inverse(J6, inv))
			//    					        cout<<inv[2][2]<<endl;
			  	inverse(J6, inv);
			  	// matinv((double *)J6,(double *)inv);
			  	//cout<<"INVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV"<<inv[0][0]<<"INVVVVVVVVVVVVVV";
			  	// matinv((float *)J6,(float *)inv);
			  	 //cout<<"INVVVVVVVVVVVVV"<<J6[0][0];
			  					    for (int ji=0; ji<6; ji++)
			  					        	    {
			  					        	    	for (int li=0;li<6;li++)
			  					        	    	{
			  					                        K[ji][li]=0;
			  					        	    		for (int pi=0;pi<6;pi++)
			  					        	    		{
			  					        	    			K[ji][li]=K[ji][li]+(J3[ji][pi]*inv[pi][li]);

			  					        	    		}
			  					    //

			  					        	    	}
			  					        	    }

			  					    Y1[0][0]=x0-hemaX-x;

			  					    Y1[1][0]=y0-hemaY-y;
			  					    Y1[2][0]=drone_navdata.altd*0.001-z;
			  					   Y1[3][0]=ax*10-cx*Vx;
			  					    Y1[4][0]=ay*10-cy*Vy;
			  					    Y1[5][0]=az-CTP;
			  	for (int ji=0; ji<6; ji++)
			{
			  		YY[ji][0]=0;
			     for (int li=0;li<6;li++)
			 {
			 YY[ji][0]=YY[ji][0]+K[ji][li]*Y1[li][0];
			 }
			}
			  	x=x+YY[0][0];
			  	y=y+YY[1][0];
			  	z=z+YY[2][0];
			  	Vx=Vx+YY[3][0];
			  	Vy=Vy+YY[4][0];
			  	Vz=Vz+YY[5][0];
//			  	myfile2<<(theta)<<setw(20)<<(phi)<<setw(20)<<psi<<setw(20)<<ax<<setw(20)<<ay<<setw(20)<<az<<setw(20)<<x0-hemaX<<setw(20)<<y0-hemaY<<setw(20)<<drone_navdata.vx*0.001<<setw(20)<<drone_navdata.vy*0.001<<setw(20)<<drone_navdata.vz*0.001<<setw(20)<<drone_navdata.altd*0.001<<setw(20)<<x<<setw(20)<<y<<setw(20)<<z<<endl;

			  	//cout<<"SLammmmmmmmmmmmmmm"<<Vx<<setw(20)<<drone_navdata.vx*0.001<<endl;
			  	for (int ji=0; ji<6; ji++)
			{
			      for (int li=0;li<6;li++)
			{
			      J8[ji][li]=0;
			      for (int pi=0;pi<6;pi++)
			  	   {
			      	J8[ji][li]=J8[ji][li]+(K[ji][pi]*H2[pi][li]);//CP
			  	   }
			      J8[ji][li]=ident[ji][li]-J8[ji][li];

			}
			}
			  	for (int ji=0; ji<6; ji++)
			  	  {
			  	        for (int li=0;li<6;li++)
			  	 {
			  	        J9[ji][li]=0;
			  	        for (int pi=0;pi<6;pi++)
			  	    	   {
			  	        	J9[ji][li]=J9[ji][li]+(J8[ji][pi]*P[pi][li]);
			  	    	   }

			  	 }
			  	  }
			  	for (int ji=0; ji<6; ji++)
			  	    	  {
			  	    	        for (int li=0;li<6;li++)
			  	    	 {
			  	    	        J10[ji][li]=0;
			  	    	        for (int pi=0;pi<6;pi++)
			  	    	    	   {
			  	    	        	J10[ji][li]=J10[ji][li]+(J9[ji][pi]*J8[li][pi]);
			  	    	    	   }

			  	    	 }
			  	    	  }
			  	for (int ji=0; ji<6; ji++)
			  	    	    	  {
			  	    	    	        for (int li=0;li<6;li++)
			  	    	    	 {
			  	    	    	        J15[ji][li]=0;
			  	    	    	        for (int pi=0;pi<6;pi++)
			  	    	    	    	   {
			  	    	    	        	J15[ji][li]=J15[ji][li]+(K[ji][pi]*R1[pi][li]);
			  	    	    	    	   }
			  	    	    	 }
			  	    	    	  }
			  	for (int ji=0; ji<6; ji++)
			  	    	    	    	  {
			  	    	    	    	        for (int li=0;li<6;li++)
			  	    	    	    	 {
			  	    	    	    	        J16[ji][li]=0;
			  	    	    	    	        for (int pi=0;pi<6;pi++)
			  	    	    	    	    	   {
			  	    	    	    	        	J16[ji][li]=J16[ji][li]+(J15[ji][pi]*K[li][pi]);
			  	    	    	    	    	   }
			  	    	    	    	        J16[ji][li]=J16[ji][li]+J10[ji][li];
			  	    	    	    	 }
			  	    	    	    	  }

			  	for (int ji=0; ji<6; ji++)
			  	    	  {
			  	    	        for (int li=0;li<6;li++)
			  	    	 {
			  	    	  P[ji][li]=J16[ji][li];
			  	    	 }
			  	    	 }

		}
			c1x= (10/(Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)+(4/(Tf*Tf))*(u0-uf+axf*Tf)+(1/(2*Tf))*(ax0-axf);
			//cout<<endl<<"coeff = "<<c1x<<endl;
			c2x= (-15/(Tf*Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(u0-uf+axf*Tf)-(1/(Tf*Tf))*(ax0-axf);
			//cout<<endl<<"coeff = "<<c2x<<endl;
			c3x= (6/(Tf*Tf*Tf*Tf*Tf))*(x0-xf+uf*Tf-(axf/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(u0-uf+axf*Tf)+(1/(2*Tf*Tf*Tf))*(ax0-axf);
			//cout<<endl<<"coeff = "<<c3x<<endl;
			c1y= (10/(Tf*Tf*Tf))*(y0-yf+uyf*Tf-(ayf/2)*Tf*Tf)+(4/(Tf*Tf))*(uy0-uyf+ayf*Tf)+(1/(2*Tf))*(ay0-ayf);
						//cout<<endl<<"coeff = "<<c1x<<endl;
			c2y= (-15/(Tf*Tf*Tf*Tf))*(y0-yf+uyf*Tf-(ayf/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(uy0-uyf+ayf*Tf)-(1/(Tf*Tf))*(ay0-ayf);
						//cout<<endl<<"coeff = "<<c2x<<endl;
			c3y= (6/(Tf*Tf*Tf*Tf*Tf))*(y0-yf+uyf*Tf-(ayf/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(uy0-uyf+ayf*Tf)+(1/(2*Tf*Tf*Tf))*(ay0-ayf);
						//cout<<endl<<"coeff = "<<c3x<<endl;
			c1z= (10/(Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)+(4/(Tf*Tf))*(v0-vf+(azf+g)*Tf)+(1/(2*Tf))*(az0-azf);
			//cout<<endl<<"coeff = "<<c1z<<endl;
			c2z= (-15/(Tf*Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)-(7/(Tf*Tf*Tf))*(v0-vf+(azf+g)*Tf)-(1/(Tf*Tf))*(az0-azf);
			//cout<<endl<<"coeff = "<<c2z<<endl;
			c3z= (6/(Tf*Tf*Tf*Tf*Tf))*(z0-zf+vf*Tf-((azf+g)/2)*Tf*Tf)+(3/(Tf*Tf*Tf*Tf))*(v0-vf+(azf+g)*Tf)+(1/(2*Tf*Tf*Tf))*(az0-azf);
			//cout<<endl<<"coeff = "<<c3z<<endl;
			tgo=Tf-(i)/f;

			//cout<<endl<<tgo<<endl;
			//cout<<endl<<zg<<endl;

            u1=c1x*tgo*tgo*tgo + c2x*tgo*tgo*tgo*tgo+ c3x*tgo*tgo*tgo*tgo*tgo+xf-uf*tgo+axf*tgo*tgo/2;
			//u1=20*c3x*tgo*tgo*tgo+12*c2x*tgo*tgo+6*c1x*tgo+axf;
			//cout<<endl<<u1;
            uy1=c1y*tgo*tgo*tgo + c2y*tgo*tgo*tgo*tgo+ c3y*tgo*tgo*tgo*tgo*tgo+yf-uyf*tgo+ayf*tgo*tgo/2;
            u2=c1z*tgo*tgo*tgo + c2z*tgo*tgo*tgo*tgo+ c3z*tgo*tgo*tgo*tgo*tgo+zf-vf*tgo+azf*tgo*tgo/2;
			//cout<<endl<<u2;

//			dxg = dxg0 + u1/f;
//			dzg =  dzg0 + (u2+g)/f;
//
//			xg = xg0 + dxg/f;
//			zg =  zg0 + (dzg)/f;

			//cout<<endl<<"Time = "<<t1-t0<<endl;

			myfile<<xg<<setw(20);
			//myfile<<yg<<setw(20);
			myfile<<zg<<setw(20);
			myfile<<x<<setw(25);
			myfile<<y<<setw(25);
			myfile<<z<<setw(25);
lx=0.1*(x-u1);
ly=0.1*(y-u2);
lz=0.1*(u2-drone_navdata.altd*0.001);
//move(lx,lx,lz,0,0,0);
	cout<<x<<setw(20)<<y<<endl;
			i = i+1;
			//cout<<"Iteration"<<i<<endl;
			myfile<<endl<<i<<setw(25);
rtt=rtt+1;

			//move(lx, ly, lz, ax, ay, az);




		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();


		}while((ros::ok()));
//(zg >= 0.1) &&
		myfile.close();

}

void imukinect_taug(double x0, double y0, double z0, double td)
{

	double ff=1;
	double  d2r = 3.14159265/180.0, Kpx = 0.35, Kz=0.7,Kpy=0.35, Kdx = 0.1, Kdy = 0.1, Kdz=2.1, Kpz=1.2*Kdz;
	double roo=4;
	double hemx,hemy,hemz;
	double xkp=est_x, ykp=est_y, zkp=est_z,xk, yk, zk;
	double rox=1,roy=1;
	double xof, yof, zof;
	int xii=0,yii=0,zii=0;
	double ex0, ey0, ez0,r0, i=0,v,phi,iphi, ex=1,ey=1,ez=3,nr,r,a,xdes=1,ydes=1,zdes, tau1,xprev=0,yprev=0,zprev=0,epx=0,epy=0,epz=0,errorx=0,errory=0,errorz=0;
	//obtaining initial velocity
	//v = sqrt(pow(posisi.dx,2) + pow(posisi.dy,2));
	v=0.4;
	double ex_dot,ey_dot,xdes_dot,ydes_dot,xdes_ddot,ydes_ddot,zdes_ddot;
	double ex0_dot=v,ey0_dot=v;
	double dt=0;
	double Vx=0,Vy=0,xo = 0,yo = 0, x=0,y=0,z=drone_navdata.altd*0.001,Vx0=0,Vy0=0,cx=-0.23/0.38,cy=-0.67/0.38,f=200.0;
	double Vx_dot,Vy_dot,Vx_dot0=0,Vy_dot0=0;
	double phi_dot,theta_dot,phi_dot0 = 0, theta_dot0 = 0;
	double p00=1,p11=1,a00=1+cx*(1/f),a11=1+cy*(1/f);
	double q00=0.0001,q11=0.0001;//obtained from analyzing values when hovering at a place
	double r00=3,r11=3;
	double c00=cx, c11=cy, k00=0, k11=0;
	double Vxo,Vyo,Vxo_i=0,Vyo_i=0;
	int oho=0;
	ex0=x0;
	ey0=y0;
	for(int ioa=0;ioa<3;ioa++)
	{
	ros::Rate loop_rate(200);
	ez0=drone_navdata.altd*0.001-z0;
	ros::spinOnce();
	loop_rate.sleep();
	}
	double gd=0;
	r0=getDistance(ex0, ey0, ez0, 0, 0, 0);
	nr = getDistance(ex0, ey0, 0, 0, 0, 0);
	iphi = asin(nr/r0);
	tau1=-r0/v;
	double xddh,yddh,zddh;
	double t0 = ros::Time::now().toSec();
	double tprev=t0;
	ofstream myfile, myfile2;
	myfile.open("/home/icgel/1cgel_hem/taug_imu.txt", ios::out | ios::app );
	myfile<<"ax"<<setw(20)<<"vodox"<<setw(20)<<"ha_mean"<<setw(20)<<"vxa"<<setw(20)<<"xa"<<setw(20)<<"xdes"<<setw(20)<<"X"<<setw(20)<<"lx1"<<setw(20)<<"VX"<<setw(20)<<" height"<<endl;
	myfile2.open("/home/icgel/1cgel_hem/taug_imuA.txt", ios::out | ios::app );
	myfile2<<"ay"<<setw(20)<<"vodoy"<<setw(20)<<"hb_mean"<<setw(20)<<"vya"<<setw(20)<<"ya"<<setw(20)<<"ydes"<<setw(20)<<"Y"<<setw(20)<<"ly1"<<setw(20)<<"VY"<<endl;
	int aa=0;
	double vxa=0,xa=0;
	double vya=0,ya=0;
	ros::Rate loop_rate(200);
	double vox,voy,voz;
	float ttt=0.1;
	double m=0.38,T,g=9.66,zdes_dot,zdd_mean;
	//double ax0 = ang.linear_acceleration.x, ay0 = ang.linear_acceleration.y, az0 = ang.linear_acceleration.z;
	int ii=0;
    double ha[10000],hb[10000],ha_mean,hb_mean;
	double theta0=(drone_navdata.rotY)*d2r,phi0=(drone_navdata.rotX)*d2r;
	double tp=ros::Time::now().toSec();

	/*ros::NodeHandle n;
	ros::Publisher velox= n.advertise<std_msgs::Float64>("velax",1000);
			ros::Publisher veloy= n.advertise<std_msgs::Float64>("velay",1000);
*/
	double ax0 = drone_navdata.ax, ay0 = drone_navdata.ay;
		do{
		ttt=ttt+0.007;
		double t1=ros::Time::now().toSec();
		i=i+1;
		ii=ii+1;
		//double ax = ang.linear_acceleration.x, ay = ang.linear_acceleration.y;az = ang.linear_acceleration.z;

		double ax = drone_navdata.ax, ay = drone_navdata.ay;
		az = drone_navdata.az;
		double theta=(drone_navdata.rotY)*d2r-theta0,phi=(drone_navdata.rotX)*d2r-phi0;
		ha[ii]=sin(theta);
		hb[ii]=cos(theta)*sin(phi);
		if(ii<10)
				{
				ha_mean = ha[ii];
				hb_mean = hb[ii];
				}
			else
				{
				ha_mean = last_ten_average(ha,ii);
				hb_mean = last_ten_average(hb,ii);
				}

		Vx_dot = 9.66*ha_mean+cx*Vx;
		Vy_dot =-9.66*hb_mean+cy*Vy;
		Vx=Vx+Vx_dot/f;
		Vy=Vy+Vy_dot/f;
		p00=p00+ (q00+2*a00*p00)/f;
		p11=p11+ (q11+2*a11*p11)/f;
		cout<<"covarianceeeeeeeeeeee"<<p00<<endl;
		//kalman gain
		k00=(c00*p00)/(p00*c00*c00+r00);
		k11=(c11*p11)/(p11*c11*c11+r11);
		//update equations!
		Vx=Vx+k00*((ax-ax0)*10-cx*Vx);
		Vy=Vy+k11*((ay-ay0)*10-cy*Vy);
		double vodox=drone_navdata.vx*0.001;
		double vodoy=drone_navdata.vy*0.001;
//		if (abs(vodox)<0.04)
//		{
//			Vx=0;
//		}
//        if (abs(vodoy)<0.04)
//				{
//					Vy=0;
//				}
//        if (abs(vodox-Vx)>0.9)
//        {
//        	Vx=vodox;
//        }
//        if (abs(vodoy-Vy)>0.9)
//                {
//                	Vy=vodoy;
//                }
		Vx_dot0 = Vx_dot;
		Vy_dot0 = Vy_dot;
		p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
		p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
		double aax=-(ax-ax0);
		double aay=-(ay-ay0);
		/*
		vxa=(100000)*(-2.4791*pow((aax),5) + 0.3745*pow((aax),4)-0.017*pow((aax),3)-0.0012*pow((aax),2)+0.0002*pow((aax),1));
		xa=xa+vxa/200;
		vya=(100000)*(-2.4791*pow((aay),5) + 0.3745*pow((aay),4)-0.017*pow((aay),3)-0.0012*pow((aay),2)+0.0002*pow((aay),1));
	    ya=ya+vya/200;
        */
		x = x + (Vx)*0.005;//ekfs axes are opposite to drones axes(thats why)
		y = y + (Vy)*0.005;

/*
		x=x+vodox/f;
		y=y+vodoy/f;
*/

		z = drone_navdata.altd*0.001;
		Vx0=Vx;
		Vy0=Vy;

		if (ex>0.05 ){
		//ex=ex0*pow((1+k*(t1-t0)/tau1x),(1/k));
		ex=(ex0/pow(td,(2/k)))*pow((td*td-ttt*ttt),(1/k));
		xdes=ex0-ex;
		ex_dot=(-2*ex0*ttt)/(k*pow(td,(2/k)))*pow((td*td-ttt*ttt),(1/k -1));
		xdes_dot=-ex_dot;
		xdes_ddot=((2*ex0*ttt)/(k*pow(td,(2/k))))*(((2/k)-1)*ttt*ttt- td*td  )*pow((td*td-ttt*ttt),((1/k)-2));
		}
		else if (ex < 0.05){
		xdes=x0;
		ex=0.05;
		xdes_dot = 0;
		xdes_ddot=0;
		}
		if (abs(ey) > 0.05){
		//ey=ey0*pow((1+k*(t1-t0)/tau1y),(1/k));
		ey=(ey0/pow(td,(2/k)))*pow((td*td-ttt*ttt),(1/k));
		ydes=ey0-ey;
		ey_dot=(-2*ey0*ttt)/(k*pow(td,(2/k)))*pow((td*td-ttt*ttt),(1/k-1));
		ydes_ddot=((2*ey0*ttt)/(k*pow(td,(2/k))))*(((2/k)-1)*ttt*ttt- td*td  )*pow((td*td-ttt*ttt),((1/k)-2));
		ydes_dot=-ey_dot;
		}
		else if (abs(ey) < 0.05){
		ydes=y0;
		ey=0.05;
		ydes_dot = 0;
		ydes_ddot=0;}
		if (ez>0.05)
	{
		ez=(ez0/pow(td,(2/k)))*pow((td*td-ttt*ttt),(1/k));
		zdes_dot=((2*ez0*ttt)/(k*pow(td,(2/k))))*pow((td*td-ttt*ttt),(1/k-1));
		zdes_ddot=((2*ez0*ttt)/(k*pow(td,(2/k))))*(((2/k)-1)*ttt*ttt- td*td  )*pow((td*td-ttt*ttt),((1/k)-2));
		zdes=ez+z0;

	}
	else if (ez<0.05)
	{
		ez=0.05;
		zdes=z0;
		zdes_dot=0;
		zdes_ddot=0;
	}
if (xdes_ddot>0)
{
xdes_ddot=0;
}
if (ydes_ddot>0)
{
ydes_ddot=0;
}
if (zdes_ddot>0)
{
zdes_ddot=0;
}
if (xdes_dot<0)
{
xdes_dot=0;
}
if (ydes_dot<0)
{
ydes_dot=0;
}
if (zdes_dot<0)
{
zdes_dot=0;
}
if (kinect_ready !=0 && oho==0)
	{
	    hover();
		oho=1;
		xof=(x)-(x0-0.3-est_x);
		yof=(y)-(y0-est_y);
		xkp=est_x;
		ykp=est_y;
		zkp=est_z;
		x=x0-0.3-est_x;
		y=y0-est_y;
//		cout<<"enters the kinecTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"<<endl;
//		cout<<"offsets -"<<xof<<" "<<yof<<endl;
		myfile<<"kinect started hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<endl;
		//cout<<"landed--";
		//land();myfile.close();
	}
	if (oho==0)
	{
//	cout<<"iMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMu"<<endl;
	}

	else if (oho==1)
	{
		xk=est_x;
		yk=est_y;
		zk=est_z;
	if (xk==xkp||yk==ykp || zk==zkp)
	{
		x=x-xof;
		y=y-yof;
		z=drone_navdata.altd*0.001;
		cout<<" odooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo";
	}
	else if (xk!=xkp || yk!=ykp || zk!=zkp)
	{
		x=x0-0.3-xk;
		y=y0-yk;
		xof=(x)-(x0-0.3-xk);
		yof=(y)-(y0-yk);
		z=drone_navdata.altd*0.001;
		cout<<"Kinectttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt";
	}
	}
	xkp=xk;
	ykp=yk;
	zkp=zk;
	vox=Vx;
	voy=Vy;
	voz=drone_navdata.vz*0.001;


	double lz1=zdes_dot-Kz*(z-zdes);
	zddh=zdes_ddot-Kz*(lz1 -zdes_dot);
		xddh=xdes_ddot-Kdx*(vodox-xdes_dot)-Kpx*(x-xdes);
		yddh=ydes_ddot-Kdy*(vodoy-ydes_ddot)-Kpy*(y-ydes);
		T=m*sqrt(xddh*xddh+yddh*yddh+(zddh+g)*(zddh+g));
			double lx1 = asin((m/T)*(Kpx*(xdes-x)- Kdx*((vodox)-xdes_dot)));

	//cout<<(1/Kdz)*(zdd_mean-g)<<setw(20)<<T<<endl;
	//cout<<(m/T)<<setw(20)<<Kpx*(xdes-x)<<setw(20)<<(Kd*((vox)-xdes_dot))<<endl;
	//cout<<zdd_mean<<endl;
	double ly1 = asin((m/T)*(Kpy*(ydes-y)- Kdy*((vodoy)-ydes_dot))/cos(lx1));


	if (lx1>1)
	{
		lx1=1;
		ff=ff+1;
	}
	if (lx1<-1)
	{
		lx1=-1;
		ff=ff+1;
	}
	if (ly1>1)
		{
			ly1=1;
			ff=ff+1;
		}
		if (ly1<-1)
		{
			ly1=-1;
			ff=ff+1;
		}
	if (ff>5)
	{
		land();
	}
	if (i>0 && i<1000 )
	{
		lx=0.1;
	}
	if (i>999 && i<2000 )
		{
			lx=0.1;
		}
	if (i>2000 )
		{
			lx=-0.05;
		}


	if (i>0 && i<750)
		{
	  gd=gd+0.0002;
		}
		if (i>749 && i<2250)
			{
		  gd=gd-0.0002;
			}
//		if (i>1599 && i<2250)
//				{
//			  gd=gd+0.0002;
//				}
				if (i>2249 && i<3000)
					{
				  gd=gd+0.0002;
					}
    //move(gd,0,0,0,0,0);
    double alt=drone_navdata.altd*0.001;
	//myfile<<xdes_ddot<<endl;
	//myfile<<xdes<<setw(20)<<x<<setw(20)<<ydes<<setw(20)<<y<<setw(20)<<zdes<<setw(20)<<z<<endl;
    myfile<<ax<<setw(20)<<drone_navdata.vx*0.001<<setw(20)<<ha_mean<<setw(20)<<vxa<<setw(20)<<xa<<setw(20)<<xdes<<setw(20)<<x<<setw(20)<<lx1<<setw(20)<<Vx<<setw(20)<<alt<<endl;
    myfile2<<ay<<setw(20)<<drone_navdata.vy*0.001<<setw(20)<<hb_mean<<setw(20)<<vya<<setw(20)<<ya<<setw(20)<<ydes<<setw(20)<<y<<setw(20)<<ly1<<setw(20)<<Vy<<setw(20)<<alt<<endl;
    //myfile2<<zddh<<setw(20)<<xddh<<setw(20)<<yddh<<setw(20)<<T<<setw(20)<<xdes<<setw(20)<<xdes_dot<<setw(20)<<x<<setw(20)<<lx1<<endl;
cout<<i<<setw(30)<<x<<setw(30)<<y<<endl;
    //myfile<<xdes<<setw(15)<<x<<setw(15)<<lx1<<setw(15)<<ydes<<setw(15)<<y<<setw(15)<<ly1<<setw(20)<<zdes<<setw(20)<<z<<endl;
	//cout<<Vx<<setw(20)<<drone_navdata.vx*0.001<<endl;
    //cout<<xdes<<setw(30)<<x<<setw(30)<<ydes<<setw(20)<<y<<setw(20)<<zdes<<setw(20)<<z<<endl;
    /*velox.publish(drone_navdata.ax);
    veloy.publish(drone_navdata.ay);*/
	//myfile2<<xdes_dot<<setw(20)<<vox<<setw(20)<<ydes_dot<<setw(20)<<voy<<setw(15)<<zdes_dot<<setw(15)<<lz<<setw(15)<<voz<<endl;
	rox=getDistance(x0,0,0,x,0,0);
	roy=getDistance(0,y0,0,0,y,0);
	roo= getDistance(x0,y0,0,x,y,0);
	ros::spinOnce();
	loop_rate.sleep();
    tp=t1;
	}while(i<5000);
		//while(ros::ok() && (abs(roo)>1 ));
		//while(i<1000);

		//while(i<1000);

	cout<<"land";
	myfile.close();
	land();
}

void imu_ekf(double x_d, double y_d, double z_d)
{
double i=0,d2r = 3.14159265/180, theta0=(drone_navdata.rotX)*d2r,phi0=(drone_navdata.rotY)*d2r;
double t0=ros::Time::now().toSec();
//initialize files
ofstream myfile,myfile1;
myfile.open("/home/icgel/icgel7/imuekf_velocities.txt", ios::out | ios::app );
myfile1.open("/home/icgel/icgel7/imuekf_distances.txt", ios::out | ios::app );
//myfile<<"odom_vx odom_vy ekf_vx ekf_vy"<<endl;
myfile<<"Vodo"<<setw(20)<<"theta"<<setw(20)<< "Vx_dot"<<setw(20)<<"Vx_dotk"<<setw(20)<<"ax"<<setw(20)<<"cvx"<<setw(20)<<"a+cvx"<<setw(20)<<"Vx"<<setw(20)<<"HH"<<endl;
myfile1<<"a"<<setw(20)<<"b"<<setw(20)<<"c"<<setw(20)<<"d"<<endl;
double Vx=0,Vy=0,xo = 0,yo = 0, x=0,y=0,Vx0=0,Vy0=0,c=0.12,f=200.0,T=1/200.0;
double Vx_dot,Vy_dot,Vx_dot0=0,Vy_dot0=0;
double phi_dot,theta_dot,phi_dot0 = 0, theta_dot0 = 0;
double p00=1,p11=1,a00=1-c*T,a11=1-c*T;
double q00=0.0001,q11=0.0001;//obtained from analyzing values when hovering at a place
double r00=0.0001,r11=0.0001;
double c00=-0.12,c11=-0.12,k00=0,k11=0;
double Vxo,Vyo,Vxo_i=0,Vyo_i=0;
double e_x,e_y,e_z,lx=0.1,ly=0.1,lz,r,r0;
double gd=0;
double theta=(drone_navdata.rotX)*d2r,phi=(drone_navdata.rotY)*d2r;
r = getDistance(0, 0, 0, x_d, y_d, z_d);
ros::Rate loop_rate(200);//loop 200 hz

	//loop begins here
	do{

	i=i+1;
	cout<<i<<setw(20);
	//move(0.1,0,0,0,0,0);
	//odometry velocities

	Vxo = drone_navdata.vx*0.001;
	Vyo = drone_navdata.vy*0.001;
	double Hem = drone_navdata.altd*0.001;
	myfile<<Vxo<<setw(20);
	double ax = ang.linear_acceleration.x, ay = ang.linear_acceleration.y;
	double theta=(drone_navdata.rotY)*d2r,phi=(drone_navdata.rotX)*d2r;
	//predict equations
	Vx_dot = -9.5*sin(theta-theta0)-c*Vx;
	Vy_dot = 9.5*cos(theta-theta0)*sin(phi-phi0)-c*Vy;
/*	if (Vx_dot>0.25)
			{
				Vx_dot=0;
			}
			if (Vx_dot<-0.25)
			{
				Vx_dot=0;
			}
			*/
	myfile<<(theta-theta0)<<setw(20)<<Vx_dot<<setw(20);

	p00=p00+ (q00+2*a00*p00)/f;
	p11=p11+ (q11+2*a11*p11)/f;
	//kalman gain
	k00=(c00*p00)/(p00*c00*c00+r00);
	k11=(c11*p11)/(p11*c11*c11+r11);
	//update equations!
	//if(lx!=0){

	Vx_dot=Vx_dot+k00*(ax - (-c*Vx));
	myfile<<Vx_dot<<setw(20)<<ax<<setw(20)<<-c*Vx<<setw(20)<<ax+c*Vx<<setw(20);
	//}
	if(ly!=0){
	Vy_dot=Vy_dot+k11*(ay - (-c*Vy));
	}
	//USING SECOND ORDER RUNGE KUTTA TO OBTAIN VX,VY FROM ABOVE Vx_dot and Vy_dot
	Vx = Vx + 0.5*(T*Vx_dot+T*Vx_dot0);
	Vy = Vy + 0.5*(T*Vy_dot+T*Vy_dot0);
	myfile<<Vx<<setw(20)<<Hem<<endl;
	Vx_dot0 = Vx_dot;
	Vy_dot0 = Vy_dot;

	p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
	p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
	//save Vx,Vy obtained from above EKF

	//obtain distance through trapezoidal integration of ekf velocity estimate
	x = x - 0.5*(Vx+Vx0)/f;//ekfs axes are opposite to drones axes(thats why)
	y = y - 0.5*(Vy+Vy0)/f;
	Vx0=Vx;
	Vy0=Vy;

	//controller below
	e_x = x_d-x;
	e_y = y_d-y;
	e_z = 0;

	lz = 0;
	if(abs(x) < x_d){
		lx = 0.2*(e_x)- 2.15*pow(10,-4)*(-Vx); //ekfs axes are opposite to drones axes(thats why)
	}
	else if(abs(x) > x_d){
		lx = 0;
	}
	if(abs(y) < y_d){
		ly = 0.2*(e_y)- 2.06*pow(10,-4)*(-Vy);//ekfs axes are opposite to drones axes(thats why)
	}
	else if(abs(y) > y_d){
		ly = 0;
	}
double gg=0.2;
double pp=0.1;
	if (i<500)
	{
		lx=gg;
	}
	if (i>500 && i<1000 )
	{
		lx=gg;
	}
	if (i>1000 && i<1500 )
		{
			lx=gg;
		}
	if (i>1500 && i<2000 )
			{
				lx=pp;
			}
	if (i>2000 && i<2500 )
			{
				lx=pp;
			}
		if (i>2500 )
		{
			lx=pp;
		}

	ly=0;
	lz=0;
	if (i>0 && i<1000)
	{
  gd=gd+0.0002;
	}
	if (i>999 && i<2000)
		{
	  gd=gd-0.0002;
		}

	move(gd,0,0,0,0,0);
	//if(lx==0 && ly==0){break;}
	r = getDistance(e_x, e_y, e_z, 0,0,0);
	myfile1<<r<<setw(20);

	//obtain distance on integrating odometry velocities
	xo = xo + 0.5*(Vxo+Vxo_i)/f;
	yo = yo + 0.5*(Vyo+Vyo_i)/f;
	Vxo_i=Vxo;
	Vyo_i=Vyo;
	myfile1<<xo<<setw(20)<<yo<<setw(20);

	double t1=ros::Time::now().toSec();
	double dt = t1-t0;
	myfile1<<lx<<setw(20)<<ly<<setw(20)<<dt<<endl;

	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();
	cout<<x<<setw(20)<<xo<<endl;
	}while(i<2000);
	//while((abs(r)>0.5) && (ros::ok()));

	//while((abs(r)>0.5) && (ros::ok()));

cout<<"land";
land();
land();
myfile.close();
myfile1.close();
}


double last_ten_average(double a[],int i){
	double val = (a[i-9]+a[i-8]+a[i-7]+a[i-6]+a[i-5]+a[i-4]+a[i-3]+a[i-2]+a[i-1]+a[i])/10;
	return val;
}
double last_ten_average2(double a[],int i){
	double val = (a[i-19]+a[i-18]+a[i-17]+a[i-16]+a[i-15]+a[i-14]+a[i-13]+a[i-12]+a[i-11]+a[i-10]+a[i-9]+a[i-8]+a[i-7]+a[i-6]+a[i-5]+a[i-4]+a[i-3]+a[i-2]+a[i-1]+a[i])/20;
	return val;
}
void tau(double x0, double y0, double z0)

{
	double xa,ya, d2r = 3.14159265/180.0,theta0,phi0;
		for(int i=0;i<5;i++){
			ros::Rate loop_rate(30);//loop 30 hz
			theta0=(drone_navdata.rotY)*d2r;phi0=(drone_navdata.rotX)*d2r;
			xa=posisi.x;
			ya=posisi.y;
			cout<<"wait for it, still initialization"<<endl;
			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
			}
double hemx,hemy,hemz;
double xkp=est_x, ykp=est_y, zkp=est_z,xk, yk, zk;
double roo=1;
double xof, yof, zof;
int xii=0,yii=0,zii=0;
double ex0, ey0, ez0, x=0, y=0,z=0,r0, i=0,v,phi,iphi, ex=1,ey=1,ez=3,nr,r,a,xdes=1,ydes=1,zdes, tau1,xprev=0,yprev=0,zprev=0,epx=0,epy=0,epz=0,errorx=0,errory=0,errorz=0;
//obtaining initial velocity after providing initial impulse
for(int i=0;i<5;i++){
			ros::Rate loop_rate(30);//loop 30 hz
			move(1,-1,0,0,0,0);
			v = sqrt(pow(posisi.dx,2) + pow(posisi.dy,2));
			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
			}

double ex_dot,ey_dot,xdes_dot,ydes_dot;
double ex0_dot=-v,ey0_dot=-v;
double Vx0=0, Vy0=0,dt=0;
int oho=0;
ex0=x0;
ey0=y0;
ez0=drone_navdata.altd*0.001-z0;
r0=getDistance(ex0, ey0, ez0, 0, 0, 0);
nr = getDistance(ex0, ey0, 0, 0, 0, 0);
iphi = asin(nr/r0);
tau1=-r0/v;
double t0 = ros::Time::now().toSec();
double tprev=t0;
ofstream myfile, myfile2;

	myfile.open("/home/icgel/icgel7/tau_tum.txt", ios::out | ios::app );
	myfile<<"xd"<<setw(20)<<"x"<<setw(20)<<"yd"<<setw(20)<<"y"<<setw(20)<<"zd"<<setw(20)<<"z"<<endl;
	myfile2.open("/home/icgel/icgel7/tau_tumA.txt", ios::out | ios::app );
		myfile2<<"vxd"<<setw(20)<<"vx"<<setw(20)<<"vyd"<<setw(20)<<"vy"<<endl;

	int aa=0;
	ros::Rate loop_rate(30);
double vox,voy;
float ttt=0.1;

do
{

double theta=(drone_navdata.rotY)*d2r,phi=(drone_navdata.rotX)*d2r;
double t1=ros::Time::now().toSec();
dt=t1-tprev;
ttt=ttt+0.75;
if (ex>0.2 ){
	//ex=ex0*pow((1+k*(t1-t0)/tau1x),(1/k));
	ex=ex0*pow((1+k*(ttt)/tau1),(1/k));
	xdes=ex0-ex;
	ex_dot=ex0_dot*pow((1+k*(ttt)/tau1),(1/k)-1);
	xdes_dot=-ex_dot;
	}
	else if (ex < 0.2){
	xdes=x0;
	ex=0.1;
	xdes_dot = 0;}

	if (ey > 0.2){
	//ey=ey0*pow((1+k*(t1-t0)/tau1y),(1/k));
	ey=ey0*pow((1+k*(ttt)/tau1),(1/k));
	ydes=ey0-ey;
	ey_dot=ey0_dot*pow((1+k*(ttt)/tau1),(1/k)-1);
	ydes_dot=-ey_dot;}
	else if (ey < 0.2){
	ydes=y0;
	ey=0.1;
	ydes_dot = 0;}
/*if (ex>0.1 )
{
ex=ex0*pow((1+k*(ttt)/tau1),(1/k));
xdes=ex0-ex;
ex_dot=ex0_dot*pow((1+k*(ttt)/tau1),(1/k)-1);
xdes_dot=ex_dot;
}
else if (ex < 0.1)
{
xdes=x0;
ex=0.05;
}
if (ey > 0.1)
{
ey=ey0*pow((1+k*(ttt)/tau1),(1/k));

ydes=ey0-ey;
}
//ez=ez0*pow((1+k*(ttt)/tau1),(1/k));


else if (ey < 0.1)
{
ydes=y0;
ey=0.05;
}*/
if (ez>0.1)
{
	ez=ez0*pow((1+k*(ttt)/(tau1)),(1/k));
	zdes=ez+z0;

}
else if (ez<0.1)
{
	ez=0.05;
	zdes=z0;
}



if (kinect_ready !=0 && oho==0)
{

	oho=1;
	xof=(posisi.x-xa)-(x0-0.3-est_x);
	yof=(posisi.y-ya)-(y0-est_y);
	xkp=est_x;
	ykp=est_y;
	zkp=est_z;
	x=x0-0.3-est_x;
	y=y0-est_y;
	cout<<"enters the kinecTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"<<endl;
	cout<<"offsets -"<<xof<<" "<<yof<<endl;
	myfile<<"kinect started here................................................................................."<<endl;

	//cout<<"landed--";
	//land();myfile.close();
}
if (oho==0)
{
x=posisi.x-xa;
y=posisi.y-ya;
z=drone_navdata.altd*0.001;
cout<<"SLAMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"<<endl;
}

/*else if(oho==1)
{
	x=x0-0.3-est_x;
	y=y0-est_y;
	z=drone_navdata.altd*0.001;
}*/
else if (oho==1)
{
	xk=est_x;
	yk=est_y;
	zk=est_z;
if (xk==xkp||yk==ykp || zk==zkp)
{
	x=posisi.x-xa-xof;
	y=posisi.y-ya-yof;
	z=drone_navdata.altd*0.001;
	cout<<"slam readings";
}
else if (xk!=xkp || yk!=ykp || zk!=zkp)
{
	x=x0-0.3-xk;
	y=y0-yk;
	xof=(posisi.x-xa)-(x0-0.3-xk);
	yof=(posisi.y-ya)-(y0-yk);
	z=drone_navdata.altd*0.001;
	cout<<"kinect reading"<<xof<<setw(20)<<yof<<endl;
}
}
xkp=xk;
ykp=yk;
zkp=zk;
vox=posisi.dx;
voy=posisi.dy;
lx = asin(cos(theta-theta0)*cos(phi-phi0)/(9.8))*(0.9*(xdes-x)- 0.0433*((vox)-xdes_dot));
ly = asin((cos(theta-theta0)*cos(phi-phi0)/(9.8))*(0.9*(y-ydes)+ 0.0431*((voy)-ydes_dot))/cos(lx));
lz = 1.2*(zdes-z);
move(lx,ly,lz,0,0,0);
myfile<<xdes<<setw(20)<<x<<setw(20)<<ydes<<setw(20)<<y<<setw(20)<<zdes<<setw(20)<<z<<endl;
cout<<xdes<<setw(15)<<x<<setw(15)<<lx<<setw(15)<<ydes<<setw(15)<<y<<setw(15)<<ly<<endl;
myfile2<<xdes_dot<<setw(20)<<vox<<setw(20)<<ydes_dot<<setw(20)<<voy<<endl;
roo=getDistance(x0,y0,0,x,y,0);
ros::spinOnce();
loop_rate.sleep();

}

while(ros::ok()  && abs(roo)> 0.6);
cout<<"land";
myfile.close();
land();
//tau1(est_x, est_y, est_z,x,y,z,xa,ya);
}

void check_kinect()
		{
			double i=0;
					ros::Rate loop_rate(200);//loop 200 hz
					do{
						cout<<est_x<<" "<<est_y<<endl;
					//i=i+1;
					ros::spinOnce();
						loop_rate.sleep();
					}while(i<2);
		}

/*
void imukinect_tau(double x_d, double y_d, double z_d)
{
double i=0,d2r = 3.14159265/180, theta0=(drone_navdata.rotX)*d2r,phi0=(drone_navdata.rotY)*d2r;
double t0=ros::Time::now().toSec();
//initialize files
ofstream myfile,myfile1,myfile2;
//myfile.open("/home/icgel/icgel7/vel_ekf_vals.txt", ios::out | ios::app );
myfile1.open("/home/icgel/icgel7/imu_tau.txt", ios::out | ios::app );
myfile2.open("/home/icgel/icgel7/imuANDgps_tau.txt", ios::out | ios::app );
//myfile<<"odom_vx odom_vy ekf_vx ekf_vy x_ekf y_ekf x_odom y_odom r T"<<endl;
myfile1<<"xdes ydes zdes x y z T"<<endl;

double Vx=0,Vy=0,xo = 0,yo = 0, x=0,y=0,z=drone_navdata.altd*0.001,Vx0=0,Vy0=0,c=x_d/100,f=200.0,T=1/200.0;
double Vx_dot,Vy_dot,Vx_dot0=0,Vy_dot0=0;
double phi_dot,theta_dot,phi_dot0 = 0, theta_dot0 = 0;
double p00=1,p11=1,a00=1-c*T,a11=1-c*T;
double q00=0.0001,q11=0.0001;//obtained from analyzing values when hovering at a place
double r00=0.0001,r11=0.0001;
double c00=-c,c11=-c,k00=0,k11=0;
double Vxo,Vyo,Vxo_i=0,Vyo_i=0;

double e_x,e_y,e_z,ct=0;
double ex=x_d,ey=y_d,ez, ex0 = x_d, ey0 = y_d, ez0,r;
//to throw away a few unwanted/wrong altitude values
do{
	ez = drone_navdata.altd*0.001;
	ez0 = drone_navdata.altd*0.001;
	ct=ct+1;
}while(ct<15);
double r0 = getDistance(0, 0, 0, x_d, y_d, z_d);
r = getDistance(0, 0, 0, x_d, y_d, z_d);
double xdes,ydes,zdes,tau1x=-r0/0.2,tau1y=-r0/0.2,tau1z=-r0/0.2,k=0.51,taut=0.1;
double lx=0.1,ly=0.1,lz=0.1;
double Kpx = 0.3,Kpy = 0.3,Kdx = 0.1,Kdy = 0.1;
double ex_dot, ex0_dot=0.2, ey_dot, ey0_dot=0.2, xdes_dot, ydes_dot;

//getting inital non zero latitude and longitude
ros::Rate loop_rate(200);//loop 200 hz

int oho=0;
double xk,yk,zk,xkp=est_x,ykp=est_y,zkp=est_z,xof,yof;

double theta=(drone_navdata.rotX)*d2r,phi=(drone_navdata.rotY)*d2r;
//ros::Rate loop_rate(200);//loop 200 hz

	//do loop begins here
	do{
	double t1=ros::Time::now().toSec();
	i=i+1;
	//myfile<<Vxo<<setw(20)<<Vyo<<setw(20);
	double ax = ang.linear_acceleration.x, ay = ang.linear_acceleration.y;
	double theta=(drone_navdata.rotY)*d2r,phi=(drone_navdata.rotX)*d2r;
	//predict equations
	Vx_dot = -9.81*sin(theta-theta0)-c*Vx;
	Vy_dot = 9.81*cos(theta-theta0)*sin(phi-phi0)-c*Vy;

	p00=p00+ (q00+2*a00*p00)/f;
	p11=p11+ (q11+2*a11*p11)/f;
	//kalman gain
	k00=(c00*p00)/(p00*c00*c00+r00);
	k11=(c11*p11)/(p11*c11*c11+r11);
	//update equations!
	if(lx!=0)
	Vx_dot=Vx_dot+k00*(ax - (-c*Vx));
	if(ly!=0)
	Vy_dot=Vy_dot+k11*(ay - (-c*Vy));
	//USING SECOND ORDER RUNGE KUTTA TO OBTAIN VX,VY FROM ABOVE Vx_dot and Vy_dot
	Vx = Vx + 0.5*(T*Vx_dot+T*Vx_dot0);
	Vy = Vy + 0.5*(T*Vy_dot+T*Vy_dot0);
	Vx_dot0 = Vx_dot;
	Vy_dot0 = Vy_dot;

	p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
	p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
	//obtain distance through trapezoidal integration of ekf velocity estimate
	x = x - 0.5*(Vx+Vx0)/f;//ekfs axes are opposite to drones axes(thats why)
	y = y - 0.5*(Vy+Vy0)/f;
	z = drone_navdata.altd*0.001;
	Vx0=Vx;
	Vy0=Vy;

	if (kinect_ready !=0 && oho==0)
	{

		oho=1;
		xof=(posisi.x-xa)-(x0-0.3-est_x);
		yof=(posisi.y-ya)-(y0-est_y);
		xkp=est_x;
		ykp=est_y;
		zkp=est_z;
		x=x0-0.3-est_x;
		y=y0-est_y;
		cout<<"enters the kinecTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"<<endl;
		cout<<"offsets -"<<xof<<" "<<yof<<endl;
		myfile<<"kinect started here................................................................................."<<endl;

		//cout<<"landed--";
		//land();myfile.close();
	}
	if (oho==0)
	{
	x=posisi.x-xa;
	y=posisi.y-ya;
	z=drone_navdata.altd*0.001;
	cout<<"SLAMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"<<endl;
	}
	else if (oho==1)
	{
		xk=est_x;
		yk=est_y;
		zk=est_z;
	if (xk==xkp||yk==ykp || zk==zkp)
	{
		x=posisi.x-xa-xof;
		y=posisi.y-ya-yof;
		z=drone_navdata.altd*0.001;
		cout<<"slam readings";
	}
	else if (xk!=xkp || yk!=ykp || zk!=zkp)
	{
		x=x0-0.3-xk;
		y=y0-yk;
		xof=(posisi.x-xa)-(x0-0.3-xk);
		yof=(posisi.y-ya)-(y0-yk);
		z=drone_navdata.altd*0.001;
		cout<<"kinect reading"<<xof<<setw(20)<<yof<<endl;
	}
	}
	xkp=xk;
	ykp=yk;
	zkp=zk;

	//obtaining coordinates of next set-point from tau guidance
	if (ex>0.2 )
	{
	//ex=ex0*pow((1+k*(t1-t0)/tau1x),(1/k));
	ex=ex0*pow((1+k*(taut)/tau1x),(1/k));
	xdes=ex0-ex;
	ex_dot=ex0_dot*pow((1+k*(taut)/tau1x),(1/k)-1);
	xdes_dot=-ex_dot;
	}
	else if (ex < 0.2)
	{
	xdes=x_d;
	ex=0.1;
	xdes_dot = 0;}

	if (ey > 0.2)
	{
	//ey=ey0*pow((1+k*(t1-t0)/tau1y),(1/k));
	ey=ey0*pow((1+k*(taut)/tau1y),(1/k));
	ydes=ey0-ey;
	ey_dot=ey0_dot*pow((1+k*(taut)/tau1y),(1/k)-1);
	ydes_dot=-ey_dot;}
	else if (ey < 0.2)
	{
	ydes=y_d;
	ey=0.1;
	ydes_dot = 0;}

	if (ez > 1)
	{
	//ey=ey0*pow((1+k*(t1-t0)/tau1y),(1/k));
	ez=ez0*pow((1+k*(taut)/tau1z),(1/k));
	zdes=ez;}
	else if (ez < 1){
	zdes=z_d;
	ez=0.1;}

	taut=taut+0.05;

	myfile1<<xdes<<setw(20)<<ydes<<setw(20)<<zdes<<setw(20)<<x<<setw(20)<<y<<setw(20)<<z<<setw(20);
	//controller below [give (coordinates of next set-point) to controller]
	e_x = (xdes)-x;
	e_y = (ydes)-y;
	e_z = (zdes)-z;

	r = getDistance(x_d-x, y_d-y, 0, 0,0,0);//distance between current and final points
	cout<<xdes<<" "<<x<<" "<<ydes<<" "<<y<<" "<<zdes<<" "<<z<<" "<<r<<" ";

	//double m=0.42, Thrust = m*az;
	lx = asin(cos(theta-theta0)*cos(phi-phi0)/(9.8))*(Kpx*(e_x)- Kdx*((-Vx)-xdes_dot));
	ly = asin((cos(theta-theta0)*cos(phi-phi0)/(9.8))*(Kpy*(e_y)- Kdy*((-Vy)-ydes_dot))/cos(lx));
	//lx = (1/(9.8*cos(theta-theta0)*cos(phi-phi0)))*(Kpx*(e_x)- Kdx*((-Vx)-xdes_dot));
	//ly = (1/(9.8*cos(theta-theta0)*cos(phi-phi0)))*(Kpy*(e_y)- Kdy*((-Vy)-ydes_dot));
	lz = 1*(e_z);
	move(lx,ly,lz,0,0,0);

	double T = t1-t0;
	myfile1<<T<<endl;
	cout<<lx<<" "<<ly<<" "<<xdes_dot<<" "<<ydes_dot<<" "<<endl;

	ros::spinOnce();
	loop_rate.sleep();

	}while(ros::ok()  && abs(r)> 0.5);

cout<<"landed";
myfile.close();
land();

}
*/

void imugps_tau(double x_d, double y_d, double z_d)
{
double i=0,d2r = 3.14159265/180, theta0=(drone_navdata.rotX)*d2r,phi0=(drone_navdata.rotY)*d2r;
double t0=ros::Time::now().toSec();
//initialize files
ofstream myfile,myfile1,myfile2;
//myfile.open("/home/icgel/icgel7/vel_ekf_vals.txt", ios::out | ios::app );
myfile1.open("/home/icgel/icgel7/imu_tau.txt", ios::out | ios::app );
myfile2.open("/home/icgel/icgel7/imuANDgps_tau.txt", ios::out | ios::app );
//myfile<<"odom_vx odom_vy ekf_vx ekf_vy x_ekf y_ekf x_odom y_odom r T"<<endl;
myfile1<<"xdes ydes zdes x_ekf y_ekf z T"<<endl;
myfile2<<"xgps ygps xcomb ycomb"<<endl;

double Vx=0,Vy=0,xo = 0,yo = 0, x=0,y=0,z=drone_navdata.altd*0.001,Vx0=0,Vy0=0,c=x_d/100,f=200.0,T=1/200.0;// c varies depending on distance
double Vx_dot,Vy_dot,Vx_dot0=0,Vy_dot0=0;
double phi_dot,theta_dot,phi_dot0 = 0, theta_dot0 = 0;
double p00=1,p11=1,a00=1-c*T,a11=1-c*T;
double q00=0.0001,q11=0.0001;//obtained from analyzing values when hovering at a place
double r00=0.0001,r11=0.0001;
double c00=-c,c11=-c,k00=0,k11=0;
double Vxo,Vyo,Vxo_i=0,Vyo_i=0;

double e_x,e_y,e_z;
double ex=x_d,ey=y_d,ez=z_d, ex0 = x_d, ey0 = y_d, ez0 = drone_navdata.altd*0.001 ,r;
double r0 = getDistance(0, 0, 0, x_d, y_d, z_d);
r = getDistance(0, 0, 0, x_d, y_d, z_d);
double xdes,ydes,zdes,tau1,k=0.51,taut=0.1;
double lx=0.1,ly=0.1,lz=0.1;

//getting inital non zero latitude and longitude
ros::Rate loop_rate(200);//loop 200 hz
do{
	cout<<"initializing lat,lon"<<lat<<" "<<lon<<endl;
	ros::spinOnce();
	loop_rate.sleep();
}while(lat==0);
double lat1 = lat*d2r,lng1=lon*d2r;
double lat2,lng2,dlat,dlng,a_val,c_val,d,R=6371000,l,x_gps,y_gps;
double p00g = 1,p11g=1,q00g=0.1,q11g=10,r00g=0.1,r11g=0.1;
double k00g,k11g,c00g=1,c11g=1,a00g=1,a11g=1,x_ekf,y_ekf;//g stands for gps kalman filter matrix vals
double x_lat, y_lon, psi_angle = 7*d2r;

int oho=0;
double xk,yk,zk,xkp=est_x,ykp=est_y,zkp=est_z,xof,yof;
double Kpx = 0.3,Kpy = 0.3,Kdx = 0.1,Kdy = 0.1;
double ex_dot, ex0_dot, ey_dot, ey0_dot, xdes_dot, ydes_dot;

double theta=(drone_navdata.rotX)*d2r,phi=(drone_navdata.rotY)*d2r;
//ros::Rate loop_rate(200);//loop 200 hz

	//do loop begins here
	do{
	double t1=ros::Time::now().toSec();
	i=i+1;
	//move(0.1,0,0,0,0,0);
	//odometry velocities
	Vxo = drone_navdata.vx*0.001;
	Vyo = drone_navdata.vy*0.001;
	//myfile<<Vxo<<setw(20)<<Vyo<<setw(20);
	double ax = ang.linear_acceleration.x, ay = ang.linear_acceleration.y;
	double theta=(drone_navdata.rotY)*d2r -theta0,phi=(drone_navdata.rotX)*d2r- phi0;
	//predict equations
	Vx_dot = -9.81*sin(theta)-c*Vx;
	Vy_dot = 9.81*cos(theta)*sin(phi)-c*Vy;

	p00=p00+ (q00+2*a00*p00)/f;
	p11=p11+ (q11+2*a11*p11)/f;
	//kalman gain
	k00=(c00*p00)/(p00*c00*c00+r00);
	k11=(c11*p11)/(p11*c11*c11+r11);
	//update equations!
	if(lx!=0)
	Vx_dot=Vx_dot+k00*(ax - (-c*Vx));
	if(ly!=0)
	Vy_dot=Vy_dot+k11*(ay - (-c*Vy));
	//USING SECOND ORDER RUNGE KUTTA TO OBTAIN VX,VY FROM ABOVE Vx_dot and Vy_dot
	Vx = Vx + 0.5*(T*Vx_dot+T*Vx_dot0);
	Vy = Vy + 0.5*(T*Vy_dot+T*Vy_dot0);
	Vx_dot0 = Vx_dot;
	Vy_dot0 = Vy_dot;


	if(i==1){//first loop to get initial values
		double vi = sqrt(pow(Vx,2) + pow(Vy,2));//initial velocity
		tau1 = -r0/vi;
		ex0_dot = -vi;
		ey0_dot = -vi;
		}

	p00=(k00*k00*r00)+p00*(c00*k00-1)*(c00*k00-1);
	p11=(k11*k11*r11)+p11*(c11*k11-1)*(c11*k11-1);
	//save Vx,Vy obtained from above EKF
	//myfile<<Vx<<setw(20)<<Vy<<setw(20);
	//obtain distance through trapezoidal integration of ekf velocity estimate
	x = x - 0.5*(Vx+Vx0)/f;//ekfs axes are opposite to drones axes(thats why)
	y = y - 0.5*(Vy+Vy0)/f;
	x_ekf = x_ekf - 0.5*(Vx+Vx0)/f;//ekfs axes are opposite to drones axes(thats why)
	y_ekf = y_ekf - 0.5*(Vy+Vy0)/f;
	z = drone_navdata.altd*0.001;
	Vx0=Vx;
	Vy0=Vy;
	//myfile<<x<<setw(20)<<y<<setw(20);
	//obtain distance on integrating odometry velocities
	xo = xo + 0.5*(Vxo+Vxo_i)/f;
	yo = yo + 0.5*(Vyo+Vyo_i)/f;
	Vxo_i=Vxo;
	Vyo_i=Vyo;
	//myfile<<xo<<setw(20)<<yo<<setw(20);

	//obtaining x and y gps values from haversine formula
	lat2=lat*d2r;
	lng2=lon*d2r;
	dlat=lat2-lat1;
	dlng=lng2-lng1;//because ylon is not opposite to increasing longitude unlike gps waypoint navigation paper
	a_val=sin(dlat/2)*sin(dlat/2)+ cos(lat1)*cos(lat2)*sin(dlng/2)*sin(dlng/2);
	c_val=2*asin(sqrt(a_val));
	d=R*c_val;
	l=atan2(dlng,dlat);
	x_lat = d*cos(l);//this is along the direction of increasing latitude( along North)
	y_lon = d*sin(l);//this is along the direction of decreasing longitude( along West)
	x_gps = cos(psi_angle)*y_lon - sin(psi_angle)*x_lat;
	y_gps = sin(psi_angle)*y_lon + cos(psi_angle)*x_lat;
	//adding gps to make x,y even better outdoors----
	//gps ekf predict
	p00g=p00g+ (q00g+2*a00g*p00g)/f;
	p11g=p11g+ (q11g+2*a11g*p11g)/f;
	//gps ekf kalman gain
	k00g=(c00g*p00g)/(p00g*c00g*c00g+r00g);
	k11g=(c11g*p11g)/(p11g*c11g*c11g+r11g);
	//gps ekf update
	x=x+k00g*(x_gps - x);
	y=y+k11g*(y_gps - y);

	p00g=(k00g*k00g*r00g)+p00g*(c00g*k00g-1)*(c00g*k00g-1);
	p11g=(k11g*k11g*r11g)+p11g*(c11g*k11g-1)*(c11g*k11g-1);

	//----------changes below------------------
	//obtaining coordinates of next set-point from tau guidance
	if (ex>0.2 ){
	//ex=ex0*pow((1+k*(t1-t0)/tau1x),(1/k));
	ex=ex0*pow((1+k*(taut)/tau1),(1/k));
	xdes=ex0-ex;
	ex_dot=ex0_dot*pow((1+k*(taut)/tau1),(1/k)-1);
	xdes_dot=-ex_dot;
	}
	else if (ex < 0.2){
	xdes=x_d;
	ex=0.1;
	xdes_dot = 0;}

	if (ey > 0.2){
	//ey=ey0*pow((1+k*(t1-t0)/tau1y),(1/k));
	ey=ey0*pow((1+k*(taut)/tau1),(1/k));
	ydes=ey0-ey;
	ey_dot=ey0_dot*pow((1+k*(taut)/tau1),(1/k)-1);
	ydes_dot=-ey_dot;}
	else if (ey < 0.2){
	ydes=y_d;
	ey=0.1;
	ydes_dot = 0;}

	if (ez > 1){
	//ey=ey0*pow((1+k*(t1-t0)/tau1y),(1/k));
	ez=ez0*pow((1+k*(taut)/tau1),(1/k));
	zdes=ez;}
	else if (ez < 1){
	zdes=z_d;
	ez=0.1;}

	taut=taut+0.03;

	myfile1<<xdes<<setw(20)<<ydes<<setw(20)<<zdes<<setw(20)<<x_ekf<<setw(20)<<y_ekf<<setw(20)<<z<<setw(20);
	myfile2<<x_gps<<setw(20)<<y_gps<<setw(20)<<x<<setw(20)<<y<<endl;
	//controller below [give (coordinates of next set-point) to controller]
	e_x = (xdes)-x;
	e_y = (ydes)-y;
	e_z = (zdes)-z;

	r = getDistance(x_d-x, y_d-y, 0, 0,0,0);//distance between current and final points
	cout<<xdes<<" "<<x<<" "<<ydes<<" "<<y<<" "<<zdes<<" "<<z<<" "<<r<<" "<<xdes_dot<<" "<<ydes_dot<<" ";

	lx = asin(cos(theta-theta0)*cos(phi-phi0)/(9.8))*(0.85*(e_x)- 0.2*((-Vx)-xdes_dot));
	ly = asin((cos(theta-theta0)*cos(phi-phi0)/(9.8))*(0.85*(e_y)- 0.2*((-Vy)-ydes_dot))/cos(lx));
	//lx = (1/(9.8*cos(theta-theta0)*cos(phi-phi0)))*(0.85*(e_x)- 0.2*((-Vx)-xdes_dot));
	//ly = (1/(9.8*cos(theta-theta0)*cos(phi-phi0)))*(0.85*(e_y)- 0.2	*((-Vy)-ydes_dot));
	lz = 1*(e_z);
	move(lx,ly,0,0,0,0);
	//if(lx==0 && ly==0){break;}

	//myfile<<r<<setw(20);
	//myfile1<<r<<setw(20)<<lx<<setw(20)<<ly<<setw(20)<<lz<<setw(20);

	double T = t1-t0;
	//myfile<<T<<endl;
	myfile1<<T<<endl;
	cout<<lx<<" "<<ly<<" "<<endl;

	ros::spinOnce();
	loop_rate.sleep();

	}while(ros::ok()  && abs(r)> 0.5);

cout<<"landed";
myfile.close();
land();


}




void compare_tum_position_vals()
{

	double xc=0,yc=0,zc=0,Vx,Vy,dtt=0.1;
	double lx,ly,lz,cx,cy,cz;
	ros::Rate loop_rate(30);//loop 50 hz
	ofstream myfile,myfile2;
	double x0,y0;
	for(int i=0;i<30;i++){
		ros::Rate loop_rate(30);//loop 50 hz
		x0=posisi.x;
		y0=posisi.y;
		cout<<"wait for it, still initialization"<<endl;
		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
		}//since gps send initial 2 lat long values as 0s
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
	//myfile2<<xc<<setw(20)<<yc<<setw(20)<<zc<<endl;
    //save current position values obtained from tum_ardrone package
	cx=posisi.x-x0;
	cy=posisi.y-y0;
	lx=0.02*(8-cx);
	ly=-0.02*(3-cy);
	move(lx,ly,0,0,0,0);
	myfile<<cx<<setw(20)<<xc<<setw(20)<<cy<<setw(20)<<yc<<endl;
	//printing cause i like it!
	cout<<cx<<setw(20)<<xc<<setw(20)<<cy<<setw(20)<<yc<<endl;
	i=i+1;
	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
	}while(abs(8-cx)>0.5);
myfile.close();
land();
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

void tauc(double x0, double y0, double z0)

{
	double xa,ya;
		for(int i=0;i<5;i++){
			ros::Rate loop_rate(30);//loop 50 hz
			xa=posisi.x;
			ya=posisi.y;
			cout<<"wait for it, still initialization"<<endl;
			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();//if process done before 50 hz sleep till 50 hz is over
			}
double hemx,hemy,hemz;
double xkp=est_x, ykp=est_y, zkp=est_z,xk, yk, zk;
double roo=1;
double xof, yof, zof;
double xg,yg,zg;
int xii=0,yii=0,zii=0;
double ex0, ey0, ez0, x=0, y=0,z=0,r0, i=0,v,phi,iphi, ex=1,ey=1,ez=3,nr,r,a,xdes=1,ydes=1,zdes, tau1,xprev=0,yprev=0,zprev=0,epx=0,epy=0,epz=0,errorx=0,errory=0,errorz=0;
v  =0.1;
int fuck=0;
double Vx0=0, Vy0=0,dt=0;
int oho=0;
ex0=x0;
ey0=y0;
ez0=drone_navdata.altd*0.001-z0;
r0=getDistance(ex0, ey0, ez0, 0, 0, 0);
nr = getDistance(ex0, ey0, 0, 0, 0, 0);
iphi = asin(nr/r0);
tau1=-r0/v;
double t0 = ros::Time::now().toSec();
double tprev=t0;
ofstream myfile;

	myfile.open("/home/icgel/icgel7/tau_tumc.txt", ios::out | ios::app );
	myfile<<"Marijuana"<<endl;

	int aa=0;
	ros::Rate loop_rate(30);

float ttt=0.1;

do
{

double t1=ros::Time::now().toSec();
dt=t1-tprev;
ttt=ttt+0.2;
if (ex>0.2 )
{
ex=ex0*pow((1+k*(ttt)/tau1),(1/k));
//xdes=ex0-ex;
}
else if (ex < 0.1)
{
//xdes=x0;
ex=0.05;
}
if (ey > 0.1)
{
ey=ey0*pow((1+k*(ttt)/tau1),(1/k));

//ydes=ey0-ey;
}
//ez=ez0*pow((1+k*(ttt)/tau1),(1/k));


else if (ey < 0.1)
{
//ydes=y0;
ey=0.05;
}
if (ez>0.1)
{
	ez=ez0*pow((1+k*(ttt)/(tau1)),(1/k));
	zdes=ez+z0;

}
else if (ez<.1)
{
	ez=0.9;
	zdes=z0;
}
r=getDistance(ex,ey,ez,0,0,0);
phi=iphi *pow((r/r0),(1/k));
a = r*sin(phi)/(r0*sin(iphi));
xdes = (1-a)* x0;
ydes = (1-a)* y0;

if (kinect_ready !=0 && oho==0)
{
	oho=1;
	xof=(posisi.x-xa)-(x0-0.3-est_x);
	yof=(posisi.y-ya)-(y0-est_y);
	xkp=est_x;
	ykp=est_y;
	zkp=est_z;
	cout<<"enters the kinecTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"<<endl;
	myfile<<"kinect started here................................................................................."<<endl;
	hover();
}

if (oho==0)
{
x=posisi.x-xa;
y=posisi.y-ya;
z=drone_navdata.altd*0.001;
cout<<"SLAMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"<<endl;
}
else if (oho==1)
{
	xk=est_x;
	yk=est_y;
	zk=est_z;
if (xk==xkp||yk==ykp || zk==zkp)
{
	x=posisi.x-xa-xof;
	y=posisi.y-ya-yof;
	z=drone_navdata.altd*0.001;
	cout<<"slam readings";
}
else if (xk!=xkp || yk!=ykp || zk!=zkp)
{
	x=x0+0.3-xk;
	y=y0-yk;
	xof=posisi.x+xa-(x0-0.3-xk);
	yof=posisi.y+ya-(y0-yk);
	z=drone_navdata.altd*0.001;
	cout<<"kinect reading"<<xof<<setw(20)<<yof<<endl;
}
}
xkp=xk;
ykp=yk;
zkp=zk;
hemx=drone_navdata.vx*0.001;
hemy=drone_navdata.vy*0.001;
lx=0.06*(xdes-x)-0.0001*hemx;
ly=0.06*(y-ydes)-0.0001*hemy;
lz=0.02*(zdes-z);
move(lx,ly,lz,0,0,0);
myfile<<xdes<<setw(20)<<x<<setw(20)<<ydes<<setw(20)<<y<<setw(20)<<zdes<<setw(20)<<z<<setw(20)<<phi<<endl;
cout<<xdes<<setw(15)<<x<<setw(15)<<lx<<setw(15)<<ydes<<setw(15)<<y<<setw(15)<<ly<<endl;
	roo=getDistance(xD,yD,0,x,y,0);

ros::spinOnce();
loop_rate.sleep();

}

while(ros::ok()  && abs(roo)> 0.2);
cout<<"land";
myfile.close();
land();
//tau1(est_x, est_y, est_z,x,y,z,xa,ya);
}
//&& abs(xdes-35)>5 && abs(ydes)>5
/*
void tau1(double x0, double y0, double z0, double x1, double y1, double z1, double xof, double yof)
{
double ex0, ey0, ez0, x=0, y=0,z=0,r0, i=0,v,phi,iphi, ex,ey,ez,nr,r,a,xdes,ydes,zdes, tau1,xprev=0,yprev=0,zprev=0,epx=0,epy=0,epz=0;
double p00=1,p11=1,p22=1;
double xs,ys,zs;
double xoff=x1-(xD-x0);
double yoff=y1-(yD-y0);
	double q00=0.0002,q11=0.0002,q22=0.0002;
	double r00=0.15,r11=0.15,r22=0.15;
	double c00=1,c11=1,c22=1;
	double a00=1,a11=1,a22=1;
	double k00,k11,k22;
v = 0.2;
ex0=x0-0.35;
ey0=

ez0=drone_navdata.altd*0.001;

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
	ros::Rate loop_rate(30);

do
{
	 if (xk==xp )
	        {

	          x=posisi.x;
	          y=posisi.y;
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


	////////////////////////
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
/*
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

*/


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
void tum_posecallback(const tum_ardrone::filter_state::ConstPtr & posi)
{
	posisi.x = posi->y;
	posisi.y = posi->x;
	posisi.z = posi->z;
	posisi.dx= posi->dx;
	posisi.dy=posi->dy;
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

	odo.twist = nav_message->twist;
}

void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message)
{
	lat	= 	gps_message->lat_fused;
	lon 	= 	gps_message->long_fused;
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
void push(const std_msgs::Bool::ConstPtr& pul)
{
	rabha= pul->data;
}
void EstCallback(const ardrone_test::est_co::ConstPtr& est)
{
  //cout<<"Target:"<< val->x<< val->y;
	est_x = est->z;
	est_y = est->y;
	est_z = est->x;
	kinect_ready = est->x;
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
void fBuzz()
{
	//ros::Rate loop_rate(10);
	Buzz.publish(emp_msg);
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

	}while(t1 <= (t0+1));

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
void move2(float lx, float ly, float lz, float ax, float ay, float az )
{

	//defining the linear velocity
	vel_msg2.linear.x = lx;
	vel_msg2.linear.y = ly;
	vel_msg2.linear.z = lz;

	//defining the linear velocity
	vel_msg2.angular.x = ax;
	vel_msg2.angular.y = ay;
	vel_msg2.angular.z = az;

	velocity_publisher2.publish(vel_msg2);

}
void moveup(double z_d)
{
	//Measuring t : current time
	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

	double e_z, z =0, i=0 ,z0 ,r = 1, x, y, x0, y0, Vx, Vy, e_x, e_y;

	z0 = gdrone.pose.pose.position.z;

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
			double z = gdrone.pose.pose.position.z;
					lx = 0;
					ly = 0;
					lz = -4*(z-z_d);
			//cout<<"Cmd_vel"<<lx<<setw(10)<<ly<<setw(10)<<lz<<endl;
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

		cout<<"height"<<z<<endl;

		move(lx, ly, lz, 0, 0, 0);


		z = gdrone.pose.pose.position.z;

		i = i+1;

		ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
		loop_rate.sleep();

		}while(abs(e_z)>0.5 && ros::ok());
		myfile.close();

}


void record()
{
	double t0=ros::Time::now().toSec();
	double i =0, f= 200.0, d2r = 3.14159265/180,dt;
	ofstream myfile;
	myfile.open("/home/icgel/icgel7/recorded_data.txt", ios::out | ios::app );
	myfile<<"gx gy gz ax ay az vxo vyo vzo theta phi psi lat lon ele T"<<endl;
	ros::Rate loop_rate(200);
	do{		//if(i<=500)
			//move(0.1,0,0,0,0,0);
			/*else if (i>500 && i<1000){
				move(0,0,0,0,0,0);}
			else{
				move(0.1,0,0,0,0,0);}
			*/
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
			myfile<<(drone_navdata.rotZ)*d2r<<setw(20);

			myfile<<setprecision(12)<<lat<<setw(20);
			myfile<<setprecision(12)<<lon<<setw(20);
			myfile<<setprecision(12)<<ele<<setw(20);

			double t1=ros::Time::now().toSec();
			dt = t1-t0;
			myfile<<dt<<endl;

			i = i+1;

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();
		}while(i<10000);
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
void hem_gdrone(const nav_msgs::Odometry::ConstPtr & nav_msg)
{
	gdrone.pose=nav_msg->pose;
	gdrone.twist=nav_msg->twist;
}
void hem_grobot(const nav_msgs::Odometry::ConstPtr & nav_msg2)
{
	grobot.pose=nav_msg2->pose;
	grobot.twist=nav_msg2->twist;
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





