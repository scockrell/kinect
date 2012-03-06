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

	int x_res=XN_VGA_X_RES;
	int y_res=XN_VGA_Y_RES;	

	int minDist;
	int minIndexJ;

	int realXMin=-4000;
	int realXMax=4000;
	int realZMin=0;
	int realZMax=3000;

	int finalXRes=100;
	int finalZRes=finalXRes*(realZMax-realZMin)/(realXMax-realXMin);

	double approxX;
	double approxY;
	double approxZ;
	double approxXOld, approxYOld, approxZOld;

	printf("got there line 120\n");
	double topDownMap[finalXRes][finalZRes]; //=new double[finalXRes][finalZRes];
	printf("after topDownMap\n");
	double gradientMap[x_res][y_res-1];
	printf("after gradientMap\n");
	//double simpleMap1[finalXRes][finalZRes-1];
	printf("after simpleMap\n");
	printf("got ther eline 124\n");

	int angleBinsCount=91;
	int* angleBins=new int[angleBinsCount];
	int tempAngle;


	const XnDepthPixel* pDepthMap;
	printf("got there line 131\n");
	XnPoint3D realWorld[real_x_dim*real_y_dim];
	printf("got ther eline 132\n");
	// source: https://groups.google.com/group/openni-dev/browse_thread/thread/e5aebba0852f8803?pli=1
	XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];
	printf("got ther eline 134\n");



	double allowedGradient=250; // dude i know 250 isn't right, i just picked something
	printf("got toehre line 128\n");


	ros::Time scan_time;
	sensor_msgs::LaserScan scan;

	printf("got ther eline 138\n");
	// Main loop
	while(count_iterations<1)
	//while (ros::ok()) //(n.ok()) // hey i don't know man, I just commented that out because of this http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	{
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

		for(int i=0;i<angleBinsCount;i++){
			angleBins[i]=0;
		}



		depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		// hey guys, x is horizontal, y is vertical, z is depth.

		for(int i=0; i<finalXRes; i++){
			for(int j=0; j<finalZRes; j++){
				topDownMap[i][j]=unknownDist;
			}
		}
		printf("got there line 182 eeee\n");
		//initialize the "old" ones
		//approxXOld=(realWorld[0].X-realXMin)*finalXRes/(realXMax-realXMin);
		approxXOld=realWorld[0].X;
		printf("after approxXOld\n");
		approxYOld=realWorld[0].Z;
		printf("after approxYOld\n");
		//approxZOld=(realWorld[0].Y-realZMin)*finalZRes/(realZMax-realZMin);
		approxZOld=realWorld[0].Y;
		printf("after approxZOld\n");
		for(int i=0; i<x_res; i++){ 
			//printf("%d ",i);
			minDist=1000000;
			for(int j=1; j<y_res; j++){
				//rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,-tiltAngle, rotatedResult);
/*
				approxX=(realWorld[j*x_res + i].X-realXMin)*finalXRes/(realXMax-realXMin);
				approxY=realWorld[j*x_res + i].Z;
				approxZ=(realWorld[j*x_res + i].Y-realZMin)*finalZRes/(realZMax-realZMin);
*/
				approxX=realWorld[j*x_res + i].X;
				approxY=realWorld[j*x_res + i].Z;
				approxZ=realWorld[j*x_res + i].Y;
				// HEY NOTICE YOU HAVE Y AND Z SWITCHED we gotta get that figured out

				tempAngle=floor(atan2(approxZ-approxZOld,approxY-approxYOld)*180.0/3.14159+.5);
				//tempAngle=floor(atan2(approxY-approxYOld,approxZ-approxZOld)*180.0/3.14159+.5);
				angleBins[tempAngle]++;

				//gradientMap is now for the angles
				gradientMap[i][j-1]=tempAngle;

/*
				if(approxY==approxYOld){
					gradientMap[i][j-1]=0;
					// notice i'm not using approxX- this is not really right, but for our purposes it should be okay
				}
				else{
					gradientMap[i][j-1]=(approxZ-approxZOld)/(approxY-approxYOld); 
				}
*/

				//if((i+j)%1000==0){printf("%d %d %d\n",approxX,approxY,approxZ);}
/*
				if(approxX>=0 && approxX<=finalXRes && approxZ>=0 && approxZ<=finalZRes){
					if(topDownMap[approxX][approxZ]<approxY){
						topDownMap[approxX][approxZ]=approxY;
					}
				}
*/
				approxXOld=approxX;
				approxYOld=approxY;
				approxZOld=approxZ;
			}
		
		}
		printf("\n");
/*
		ofstream myfile2; // this really should be declared outside of the while loop, but i'm only running it once so whatever
		myfile2.open ("anglemap.txt");
		for(int j=0; j<y_res-1; j++){
			for(int i=0;i<x_res; i++){
				myfile2 << gradientMap[i][j] << ", " ;
			}
			myfile2 << "\n" ;
		}
		myfile2.close();
*/
		for(int i=0;i<angleBinsCount;i++){
			printf("%d %d ",i,angleBins[i]);
			if(i%5==4){
				printf("\n");
			}
		}

		count_iterations++;
		printf("count_iterations = %d\n",count_iterations);

		r.sleep();
	}

	// Clean-up
	context.Shutdown();
	printf("got there after context.Shutdown()\n");
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
