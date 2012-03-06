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

	ifstream calib_file("calibration.txt");
	string tiltAngle_s;
	getline(calib_file,tiltAngle_s);
	double tiltAngle=-atof(tiltAngle_s.c_str()); // NOTE THE NEGATIVE SIGN.  because the double from the file is pos
	string floorThreshold_s;
	getline(calib_file,floorThreshold_s);
	cout << tiltAngle_s << "," << floorThreshold_s << ",\n";
	//printf("%s\n",tiltAngle_s.c_str());
	//printf("%s\n",floorThreshold_s.c_str());
	double floorThreshold=atof(floorThreshold_s.c_str());
	printf("angle = %f height = %f\n",tiltAngle,floorThreshold);

	calib_file.close();

	//double floorThreshold=-950; //-950; // magical floor threshold: -950 yay!

	int angleIndex;
	int angleIndex2;
	//int tiltAngle=-13; //-20; //60; //45; //15; // in degrees
	// magical angle: -13 yay!

	int total_iterations=10;
	int count_iterations=0;

	int real_x_dim=640;
	int real_y_dim=480;

	int x_res=XN_VGA_X_RES;
	int y_res=XN_VGA_Y_RES;	

	int minDist;
	int minIndexJ;

	int realXMin=-2000;
	int realXMax=2000;
	int realZMin=0;
	int realZMax=3000;

	int finalXRes=200;
	int finalZRes=finalXRes*(realZMax-realZMin)/(realXMax-realXMin);

	int approxX;
	int approxY;
	int approxZ;

	int smoothConstant=1;
	double tempSlope1;
	double tempSlope2;

	printf("got there line 120\n");
	double topDownMap[finalXRes][finalZRes]; //=new double[finalXRes][finalZRes];
	printf("after topDownMap\n");
	double gradientMap[finalXRes-smoothConstant][finalZRes-smoothConstant];
	printf("after gradientMap\n");
	//double simpleMap1[finalXRes][finalZRes-1];
	printf("after simpleMap\n");
	printf("got ther eline 124\n");


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



		depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		// hey guys, x is horizontal, y is vertical, z is depth.

		for(int i=0; i<finalXRes; i++){
			for(int j=0; j<finalZRes; j++){
				topDownMap[i][j]=unknownDist;
			}
		}
/*
		ofstream myfile_x;
		myfile_x.open ("real_coord_x.txt");
		ofstream myfile_y;
		myfile_y.open ("real_coord_y.txt");
		ofstream myfile_z;
		myfile_z.open ("real_coord_z.txt");
*/

		for(int i=0; i<x_res; i++){ 
			minDist=1000000;
			for(int j=0; j<y_res; j++){
				rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,-tiltAngle, rotatedResult);
				approxX=(rotatedResult[0]-realXMin)*finalXRes/(realXMax-realXMin);
				approxY=rotatedResult[1]-floorThreshold;
				approxZ=(rotatedResult[2]-realZMin)*finalZRes/(realZMax-realZMin);
				//if((i+j)%1000==0){printf("%d %d %d\n",approxX,approxY,approxZ);}
				if(approxX>=0 && approxX<=finalXRes && approxZ>=0 && approxZ<=finalZRes){
					if(topDownMap[approxX][approxZ]<approxY){
						topDownMap[approxX][approxZ]=approxY;
					}
				}
/*
				myfile_x << realWorld[j*x_res + i].X << "," ;
				myfile_y << realWorld[j*x_res + i].Y << "," ;
				myfile_z << realWorld[j*x_res + i].Z << "," ;
*/
			}
/*
			myfile_x << "\n";
			myfile_y << "\n";
			myfile_z << "\n";
*/		
		}
		// project back for the ones that have no info
		printf("got there line 199\n");
		for(int i=0; i<finalXRes; i++){
			for(int j=1; j<finalZRes; j++){
				if(topDownMap[i][j]==unknownDist){
					topDownMap[i][j]=topDownMap[i][j-1];
				}
			}
		}

		printf("got there line 207\n");
		for(int i=0; i<finalXRes-smoothConstant; i++){
			for(int j=0;j<finalZRes-smoothConstant;j++){
				tempSlope1=(topDownMap[i][j]-topDownMap[i][j+smoothConstant])/smoothConstant;
				tempSlope2=(topDownMap[i][j]-topDownMap[i+smoothConstant][j])/smoothConstant;


				gradientMap[i][j]=sqrt(tempSlope1*tempSlope1 + tempSlope2*tempSlope2);

/*
				if(abs(gradientMap[i][j])>=allowedGradient){
					simpleMap[i][j]=2*allowedGradient;
				}
				else{
					simpleMap[i][j]=abs(gradientMap[i][j]);
				}
*/

			}
		}
		printf("got there line 219\n");


		// write to file
		ofstream myfile;
		myfile.open ("topdownview.txt");
		if(!myfile){
			printf("oh my goodness, the file didn't open\n");
		}
		for(int i=0;i<finalXRes; i++){
			for(int j=0; j<finalZRes; j++){
				myfile << topDownMap[i][j] << ", " ;
			}
			myfile << "\n" ;
		}

		myfile.close();

		ofstream myfile2;
		myfile2.open ("diff.txt");
		for(int i=0;i<finalXRes; i++){
			for(int j=0; j<finalZRes-1; j++){
				myfile2 << gradientMap[i][j] << ", " ;
			}
			myfile2 << "\n" ;
		}
		myfile2.close();

/*
		ofstream myfile3;
		myfile3.open ("simple.txt");
		for(int i=0;i<finalXRes; i++){
			for(int j=0; j<finalZRes-1; j++){
				myfile3 << simpleMap[i][j] << ", " ;
			}
			myfile3 << "\n" ;
		}
		myfile3.close();
*/
/*
		myfile_x.close();
		myfile_y.close();
		myfile_z.close();
*/

		count_iterations++;
		printf("count_iterations = %d\n",count_iterations);
/*
		scan_time = ros::Time::now();

		//populate the LaserScan message
		//sensor_msgs::LaserScan scan; //declaring this BEFORE the while loop instead
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_frame";
		scan.angle_min = -1.57;
		scan.angle_max = 1.57;
		scan.angle_increment = 3.14 / (2*angleMax);
		scan.time_increment = (1 / laser_frequency) / (2*angleMax);
		scan.range_min = -1; //0.0;
		scan.range_max = 100.0;

		scan.set_ranges_size(2*angleMax);
		//scan.set_intensities_size(2*angleMax);
		for(unsigned int i = 0; i < 2*angleMax; ++i){
			scan.ranges[i] = distances[i];
			//scan.intensities[i] = intensities[i];
		}

		scan_pub.publish(scan);
*/
		r.sleep();
	}


	// eh my attempt to free memory
//	delete [] distances;
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
