// this is dumb: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mobilerobots/kinect/proximity_detection
// or this might be better export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mobilerobots/kinect/proximity_detection
// heh? export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mobilerobots/kinect/topdown

// possibly useful: http://www.ros.org/doc/api/nao_openni/html/namespacexn.html
// Include OpenNI
//#include </home/mobilerobots/kinect/OpenNI/Include/XnCppWrapper.h>
//#include </home/stephanie/code/dev_stacks/kinect/OpenNI/Include/XnCppWrapper.h>
// Include NITE
//#include "/home/mobilerobots/kinect/NITE/Nite-1.4.0.5/Include/XnVNite.h"
//#include "/home/stephanie/code/dev_stacks/kinect/NITE/Include/XnVNite.h"
#include "/home/stephanie/drivers/nite/build/Nite-1.3.1.5/Include/XnVNite.h"

// the following 3 things i'm adding because now i'm gonna run this in ros.  source: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <math.h>

// for the writing to file and stuff- to test the output
#include <iostream>
#include <fstream>

// to do the ros msg i have this source http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors
#include </opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include/sensor_msgs/LaserScan.h>
//#include <sensor_msgs/LaserScan.h>

using namespace std;
//using namespace sensor_msgs;



// This macro checks the return code that all OpenNI calls make
// and throws an error if the return code is an error. Use this
// after all calls to OpenNI objects. Great for debugging.
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

#define _TCHAR char
#define _tmain main

void xYToRTheta(double x, double y, double* result);
void rotate(double x, double y, double z, double theta, double* result);

int _tmain(int argc, _TCHAR* argv[]){
	//
	// Variables

	// Keep track of the return code of all OpenNI calls
	XnStatus nRetVal = XN_STATUS_OK;
	// context is the object that holds most of the things related to OpenNI
	xn::Context context;
	// The DepthGenerator generates a depth map that we can then use to do 
	// cool stuff with. Other interesting generators are gesture generators
	// and hand generators.
	xn::DepthGenerator depth;

	//
	// Initialization
	
	// Initialize context object
	nRetVal = context.Init();
	CHECK_RC(nRetVal, "Initialize context");
	// Create the depth object
	nRetVal = depth.Create(context);
	CHECK_RC(nRetVal, "Create Depth");
	
	// Tell the context object to start generating data
	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "Start Generating All Data");

	// to publish:
	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

	double laser_frequency = 40;
	ros::Rate r(15); //run at 15 Hz		

	int angleMax=90;
	int* distances=new int[2*angleMax];
	int* distances_index=new int[2*angleMax];
	int unknownDist=-2000; 

	double* rotatedResult=new double[3];
	double* tempRTheta=new double[2];
	double floorThreshold=-950; //-950; // magical floor threshold: -950 yay!

	int angleIndex;
	int angleIndex2;
	int tiltAngle=-13; //-20; //60; //45; //15; // in degrees
	// magical angle: -13 yay!

	int total_iterations=10;
	int count_iterations=0;

	int real_x_dim=640;
	int real_y_dim=480;

	int x_res=XN_VGA_X_RES; // should equal 640
	int y_res=XN_VGA_Y_RES;	// should equal 480

	int minDist;
	int minIndexJ;

	// what rectangle of data to process
	int x_start=x_res/4;
	int x_end=3*x_res/4;
	int y_start=2*y_res/3;
	int y_end=y_res;

	int pixel_area=(x_end-x_start)*(y_end-y_start);

	int x_sum;
	int y_sum;
	int z_sum;
	double x_sum2,y_sum2,z_sum2;
	int x_avg;
	int y_avg;
	int z_avg;
	int x_min, x_max, y_min, y_max, z_min, z_max;
	double x_dev;
	double y_dev;
	double z_dev;


	const XnDepthPixel* pDepthMap;
	printf("got there line 131\n");
	XnPoint3D realWorld[real_x_dim*real_y_dim];
	printf("got ther eline 132\n");
	// source: https://groups.google.com/group/openni-dev/browse_thread/thread/e5aebba0852f8803?pli=1
	XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];
	printf("got ther eline 134\n");


	ros::Time scan_time;
	sensor_msgs::LaserScan scan;

	// Main loop
	while(count_iterations<1)
	//while (ros::ok()) //(n.ok()) // hey i don't know man, I just commented that out because of this http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	{
		x_sum=0;
		y_sum=0;
		z_sum=0;
		x_min=0;
		x_max=0;
		y_min=1000000;
		y_max=-y_min;
		z_min=1000000;
		z_max=-z_min;

		printf("got ther eline 139\n");
		// Wait for new data to be available
		nRetVal = context.WaitOneUpdateAll(depth);
		CHECK_RC(nRetVal, "Updating depth");
		// Get the new depth map
		pDepthMap = depth.GetDepthMap();

		for (int y=0; y<XN_VGA_Y_RES; y++){
			for(int x=0;x<XN_VGA_X_RES;x++){
			        pointList[y * XN_VGA_X_RES + x ].X =x;
			        pointList[y * XN_VGA_X_RES + x ].Y =y;
			        pointList[y * XN_VGA_X_RES + x ].Z = (short) pDepthMap[y *XN_VGA_X_RES + x];
			}
		} 

		depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		// hey guys, x is horizontal, y is vertical, z is depth.

		for(int i=x_start; i<x_end; i++){ 
			for(int j=y_start; j<y_end; j++){
				rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,-tiltAngle, rotatedResult);
				x_sum+=rotatedResult[0];
				y_sum+=rotatedResult[1];
				z_sum+=rotatedResult[2];
				if(rotatedResult[0]<x_min){ x_min=rotatedResult[0];}
				if(rotatedResult[0]>x_max){ x_max=rotatedResult[0];}
				if(rotatedResult[1]<y_min && rotatedResult[1]!=0){ y_min=rotatedResult[1];}
				if(rotatedResult[1]>y_max && rotatedResult[1]!=0){ y_max=rotatedResult[1];}
				if(rotatedResult[2]<z_min && rotatedResult[2]!=0){ z_min=rotatedResult[2];}
				if(rotatedResult[2]>z_max && rotatedResult[2]!=0){ z_max=rotatedResult[2];}
			}
		}

		x_avg=x_sum/pixel_area;
		y_avg=y_sum/pixel_area;
		z_avg=z_sum/pixel_area;

		printf("x_avg = %d\ny_avg = %d\nz_avg = %d\n",x_avg,y_avg,z_avg);
		printf("x %d to %d\ny %d to %d\nz %d to %d\n",x_min,x_max,y_min,y_max,z_min,z_max);

		// now run through all the pts again to get standard deviation
		x_sum2=0;
		y_sum2=0;
		z_sum2=0;
		for(int i=x_start; i<x_end; i++){ 
			for(int j=y_start; j<y_end; j++){
				rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,-tiltAngle, rotatedResult);
				x_sum2+=(rotatedResult[0]-x_avg)*(rotatedResult[0]-x_avg);
				y_sum2+=(rotatedResult[1]-y_avg)*(rotatedResult[1]-y_avg);
				z_sum2+=(rotatedResult[2]-z_avg)*(rotatedResult[2]-z_avg);

			}
		}

		x_dev=sqrt(x_sum2/pixel_area);
		y_dev=sqrt(((double)y_sum2)/pixel_area);
		z_dev=sqrt(((double)z_sum2)/pixel_area);

		printf("x_dev = %f\ny_dev = %f\nz_dev = %f\n",x_dev,y_dev,z_dev);


		count_iterations++;
		printf("count_iterations = %d\n",count_iterations);

		r.sleep();
	}

	// Clean-up
	context.Shutdown();
	return 0;
}
void xYToRTheta(double x, double y, double* result){
	result[0]=sqrt(x*x+y*y);
	result[1]=atan2(y,x)*180/3.14159;
}
void rotate(double x, double y, double z, double theta, double* result){
	// so for this case, it needs to be (x, depth, height)
	double thetaRad=theta*3.14159/180;
	result[0]=x;
	result[1]=cos(thetaRad)*y - sin(thetaRad)*z;
	result[2]=sin(thetaRad)*y + cos(thetaRad)*z;
}
