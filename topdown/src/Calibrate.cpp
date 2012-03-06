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

	int unknownDist=-2000; 

	//double* rotatedResult=new double[3];
	double floorThreshold=-950; //-950; // magical floor threshold: -950 yay!

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

	int realXMin=-4000;
	int realXMax=4000;
	int realZMin=0;
	int realZMax=3000;

	int finalXRes=100;
	int finalZRes=finalXRes*(realZMax-realZMin)/(realXMax-realXMin);

	double approxX;
	double approxY;
	double approxZ;
	//double approxXOld, approxYOld, approxZOld;

	int smoothConstant=100;
	double* yArray=new double[smoothConstant];
	double* zArray=new double[smoothConstant];

	printf("got there line 120\n");
	//double topDownMap[finalXRes][finalZRes]; //=new double[finalXRes][finalZRes];
	//printf("after topDownMap\n");
	double gradientMap[x_res][y_res-1];
	printf("after gradientMap\n");
	//double simpleMap1[finalXRes][finalZRes-1];
	//printf("after simpleMap\n");
	//printf("got ther eline 124\n");

	int angleBinsSum=0;
	double angleBinsMin=0;
	double angleBinsMax= 89;
	//int angleBinsCount=angleBinsMax-angleBinsMin;
	int angleBinsCount=100;
	int tempBinIndex;
	int wonkyCount;
	int noDataCount;
	int* angleBins=new int[angleBinsCount];
	double* binAvgs=new double[angleBinsCount];
	double tempAngle;

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
	while(angleBinsSum==0){
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
			binAvgs[i]=0;
		}
		wonkyCount=0;
		noDataCount=0;

		depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		// hey guys, x is horizontal, y is vertical, z is depth.
/*
		for(int i=0; i<finalXRes; i++){
			for(int j=0; j<finalZRes; j++){
				topDownMap[i][j]=unknownDist;
			}
		}
*/
		for(int i=0; i<x_res; i++){ 
		//for(int i=x_res/4; i<3*x_res/4; i++){ 
			//printf("%d ",i);
			minDist=1000000;
			for(int i2=0; i2<smoothConstant; i2++){
				yArray[i2]=realWorld[i2*x_res + i].Z;
				zArray[i2]=realWorld[i2*x_res + i].Y; // and no one cares about x
			}
			for(int j=0;j<smoothConstant; j++){
				gradientMap[i][j]=0;
			}
			for(int j=smoothConstant; j<y_res; j++){
			//for(int j=y_res/4; j<3*y_res/4; j++){
				approxX=realWorld[j*x_res + i].X;
				approxY=realWorld[j*x_res + i].Z;
				approxZ=realWorld[j*x_res + i].Y;
				// HEY NOTICE YOU HAVE Y AND Z SWITCHED we gotta get that figured out

				tempAngle=atan2(zArray[0]-approxZ,yArray[0]-approxY)*180.0/3.14159;
				//tempAngle=floor(atan2(approxZ-zArray[0],approxY-yArray[0])*180.0/3.14159+.5);
				//tempAngle=floor(atan2(approxY-approxYOld,approxZ-approxZOld)*180.0/3.14159+.5);
				tempBinIndex=(int)(floor((tempAngle-angleBinsMin)/(angleBinsMax-angleBinsMin) * angleBinsCount));

				if(approxY==0 || yArray[0]==0){
					noDataCount++;
				}
				else{
					if(tempBinIndex>=0 && tempBinIndex<angleBinsCount){
						angleBins[tempBinIndex]++;
						binAvgs[tempBinIndex]=(tempAngle + (angleBins[tempBinIndex]-1)*binAvgs[tempBinIndex])/angleBins[tempBinIndex];
					}
					else{
						wonkyCount++;	
/*
						printf("%d %f ",tempBinIndex,tempAngle);
						if(wonkyCount%10 == 0){
							printf("\n");
						}	
*/		
					}
				}

				//gradientMap is now for the angles
				gradientMap[i][j]=tempAngle;

				for(int i2=0;i2<smoothConstant-1; i2++){
					yArray[i2]=yArray[i2+1];
					zArray[i2]=zArray[i2+1];
				}
				yArray[smoothConstant-1]=approxY;
				zArray[smoothConstant-1]=approxZ;
			}
		
		}
		printf("\nfreakin got there line 251\n");

		ofstream myfile2; // this really should be declared outside of the while loop, but i'm only running it once so whatever
		myfile2.open ("anglemap.txt");
		for(int j=0; j<y_res-1; j++){
			for(int i=0;i<x_res; i++){
				myfile2 << gradientMap[i][j] << ", " ;
			}
			myfile2 << "\n" ;
		}
		myfile2.close();
		printf("gpot here line 262\n");

		angleBinsSum=0;
		for(int i=0;i<angleBinsCount;i++){
			//printf("%d %d \n",i,angleBins[i]);
			printf("%f %d %f\n",i*(angleBinsMax-angleBinsMin)/angleBinsCount + angleBinsMin,angleBins[i],binAvgs[i]);
			angleBinsSum+=angleBins[i];
		}
		printf("\nangleBinsSum = %d\n%d x %d = %d\nwonkyCount = %d\nnoDataCount = %d\n",angleBinsSum,x_res,y_res,x_res*y_res,wonkyCount,noDataCount);

		count_iterations++;
		printf("count_iterations = %d\n",count_iterations);

		r.sleep();
	}

	// okay, now you have all the stuff in bins.  so now find the bin with the most
	int maxBinIndex=-1;
	double maxBinValue=-1;
	for(int i=0; i<angleBinsCount; i++){
		if(angleBins[i]>maxBinValue){
			maxBinValue=angleBins[i];
			maxBinIndex=i;
		}
	}
	double finalAngle;
	if(maxBinIndex>0 && maxBinIndex<angleBinsCount-1){
		finalAngle=(binAvgs[maxBinIndex-1]*angleBins[maxBinIndex-1] + binAvgs[maxBinIndex]*angleBins[maxBinIndex] + binAvgs[maxBinIndex+1]*angleBins[maxBinIndex+1])/(angleBins[maxBinIndex-1]+angleBins[maxBinIndex]+angleBins[maxBinIndex+1]);
	}
	else{ // screw it
		finalAngle=binAvgs[maxBinIndex];
	}

	// okay now rotate everything to get heights, put in bins, etc, find the height of kinect
	double* rotatedResult=new double[3];
	double heightBinsMin=-2000;
	double heightBinsMax= 0;
	int heightBinsCount=200;

	int* heightBins=new int[heightBinsCount];
	double* heightBinAvgs=new double[heightBinsCount];
	double tempHeight;

	for(int i=0;i<heightBinsCount;i++){
		heightBins[i]=0;
		heightBinAvgs[i]=0;
	}

	for(int i=0; i<x_res; i++){ 
		for(int j=0; j<y_res; j++){
	//for(int i=x_res/4; i<3*x_res/4; i++){ 
	//	for(int j=y_res/4; j<3*y_res/4; j++){
			rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,finalAngle, rotatedResult);
			tempHeight=rotatedResult[1];
			tempBinIndex=(int)(floor((tempHeight-heightBinsMin)/(heightBinsMax-heightBinsMin) * heightBinsCount));
/*
			approxX=(rotatedResult[0]-realXMin)*finalXRes/(realXMax-realXMin);
			approxY=rotatedResult[1]-floorThreshold;
			approxZ=(rotatedResult[2]-realZMin)*finalZRes/(realZMax-realZMin);
*/

			if(rotatedResult[2]!=0){
				if(tempBinIndex>=0 && tempBinIndex<heightBinsCount){
					heightBins[tempBinIndex]++;
					heightBinAvgs[tempBinIndex]=(tempHeight + (heightBins[tempBinIndex]-1)*heightBinAvgs[tempBinIndex])/heightBins[tempBinIndex];
				}

			}
		}		
	}
	// so all the heights are in bins.  print them out
/*
	for(int i=0;i<heightBinsCount;i++){
		printf("%f %d %f\n",i*(heightBinsMax-heightBinsMin)/heightBinsCount + heightBinsMin,heightBins[i],heightBinAvgs[i]);
	}
*/

	// okay, now you have all the heights in bins.  so now find the bin with the most
	maxBinIndex=-1;
	maxBinValue=-1;
	for(int i=0; i<heightBinsCount; i++){
		if(heightBins[i]>maxBinValue){
			maxBinValue=heightBins[i];
			maxBinIndex=i;
		}
	}
	double finalHeight;
	if(maxBinIndex>0 && maxBinIndex<heightBinsCount-1){
		finalHeight=(heightBinAvgs[maxBinIndex-1]*heightBins[maxBinIndex-1] + heightBinAvgs[maxBinIndex]*heightBins[maxBinIndex] + heightBinAvgs[maxBinIndex+1]*heightBins[maxBinIndex+1])/(heightBins[maxBinIndex-1]+heightBins[maxBinIndex]+heightBins[maxBinIndex+1]);
	}
	else{ // screw it
		finalHeight=binAvgs[maxBinIndex];
	}

	// now write-to-file all the pts in those 3 (height) bins
	ofstream myfile_planefit; 
	myfile_planefit.open ("planefitpts.txt");
	for(int i=x_res/4; i<3*x_res/4; i++){
		for(int j=y_res/2; j<y_res; j++){
	//for(int i=0; i<x_res; i++){ 
	//	for(int j=0; j<y_res; j++){
			rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,finalAngle, rotatedResult);
			tempHeight=rotatedResult[1];
			tempBinIndex=(int)(floor((tempHeight-heightBinsMin)/(heightBinsMax-heightBinsMin) * heightBinsCount));

		//	if(abs(tempBinIndex-maxBinIndex)<=1){
				myfile_planefit << realWorld[j*x_res + i].X << ", ";
				myfile_planefit << realWorld[j*x_res + i].Y << ", ";
				myfile_planefit << realWorld[j*x_res + i].Z << "\n";
		//	}


		}		
	}
	myfile_planefit.close();

	printf("finalAngle  = %f\n",finalAngle);
	printf("finalHeight = %f\n",finalHeight);

	ofstream myfile_calibration; 
	myfile_calibration.open ("calibration.txt");

	myfile_calibration << finalAngle  << "\n";
	myfile_calibration << finalHeight << "\n";

	myfile_calibration.close();

	// Clean-up
	context.Shutdown();
	//printf("got there after context.Shutdown()\n");
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
