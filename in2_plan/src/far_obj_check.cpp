#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "in2_msgs/RoadSurface.h"
#include "in2_msgs/DecInfo.h"
#include "in2_msgs/InsInfo.h"
#include "in2_msgs/Point2D.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

// #define  ABS(X)   (X >= 0 ? X : (-X)) // it's dangerous! such as X = a - b, then -X = -a - b.
                                         // Not -(a - b)! Because there is no "()"!
void sqToMeter(int a,int b,float &x, float &y);
//subscribe variables
ros::Subscriber road_surface_sub_;
ros::Subscriber decInfo_sub_;
ros::Subscriber ins_sub_;

in2_msgs::Point2D obs_xy;

ros::Publisher dis_to_farobj_pub;
ros::Publisher xy_of_farobj_pub;
//varibles
in2_msgs::DecInfo Dec_info;
bool is_Dec_info_rcv = false;
in2_msgs::Point2D DecLane[1000];

in2_msgs::InsInfo ins;
double XX = 0;
double YY = 0;
double ZZ = 0;
bool ins_recv = false;
double Timer_ins;

in2_msgs::RoadSurface RoadSurface;
float FarObjDistance;

double cut_dis = 38.0; // m

//display variables 
IplImage *MapShow;
IplImage *MapShow_S;
char strtemp[sizeof(double)*8+1];
CvFont font;

int filter_cnt = 0;

#define LOCAL_COOR_WIDTH   3000
#define LOCAL_COOR_LENGTH  3000
#define GIS_INTERVAL       0.4

void display(void)
{
	// 1. clear
	// cv::Mat empty_mat = cv::Mat::zeros(LOCAL_COOR_WIDTH, 2*LOCAL_COOR_LENGTH, CV_8UC3);
	// MapShow->imageData = (char*)empty_mat.data;

	// 2. car
	cvCircle(MapShow,cvPoint(1500,2*LOCAL_COOR_LENGTH - 1 - 1000),20,CV_RGB(255,0,0),-1);
	
	// 3. scan
	int pts_num = 720;
	for (int i = 0; i < pts_num; i++) {
		int x = 1500 + RoadSurface.points[i].x / 0.02;
		int y = 1000 + RoadSurface.points[i].y / 0.02;
		if (RoadSurface.points[i].y < cut_dis)
			cvCircle(MapShow,cvPoint(x,2*LOCAL_COOR_LENGTH - 1 - y),12,CV_RGB(0,50,0),-1);
		else
			cvCircle(MapShow,cvPoint(x,2*LOCAL_COOR_LENGTH - 1 - y),12,CV_RGB(0,255,0),-1);
	}
	
	// 4. path
	int path_len = Dec_info.RoadPoints.size();
	for (int i = 0; i < path_len; i++) {
		int x = 1500 + DecLane[i].x / 0.02;
		int y = 1000 + DecLane[i].y / 0.02;
		
		if (DecLane[i].y < cut_dis)
			cvCircle(MapShow,cvPoint(x,2*LOCAL_COOR_LENGTH - 1 - y),8,CV_RGB(50,50,50),-1);
		else
			cvCircle(MapShow,cvPoint(x,2*LOCAL_COOR_LENGTH - 1 - y),8,CV_RGB(255,255,255),-1);
	}
	
	int cut_y_on_img = 0;
	
	// 5. cut line
	cut_y_on_img = 2*LOCAL_COOR_LENGTH - 1 - 1000 - cut_dis * 50.0;
	cvLine(MapShow, cvPoint(0,cut_y_on_img), cvPoint(LOCAL_COOR_WIDTH-1,cut_y_on_img), CV_RGB(255,0,0), 10);
	
	// 6. 40m line
	cut_y_on_img = 2*LOCAL_COOR_LENGTH - 1 - 1000 - 40.0 * 50.0;
	cvLine(MapShow, cvPoint(0,cut_y_on_img), cvPoint(LOCAL_COOR_WIDTH-1,cut_y_on_img), CV_RGB(30,30,30), 10);
	
	if(MapShow != NULL)
	{
		cvResize(MapShow,MapShow_S);
		cv::namedWindow("MapShow",0);
		cvShowImage("MapShow",MapShow_S);
		cvWaitKey(3);
	}
}

in2_msgs::Point2D convENUtoLocal(double x, double y, double x0, double y0, double azimuth)
{
	double dx = x - x0;
	double dy = y - y0;
	double cosa = cos(azimuth * M_PI / 180.0);
	double sina = sin(azimuth * M_PI / 180.0);

	double resultX = dx * cosa - dy * sina;
	double resultY = dx * sina + dy * cosa;

	in2_msgs::Point2D pt;
	pt.x = resultX;
	pt.y = resultY;

	return pt;
}
float Far_Obj_Check()
{
	float offsetX = 1000.0, offsetY = 1000.0;
	float OffsetTemp = 100.0,OffsetMin = 9999.9, OffsetMinold = 10000.0;
	float ObjDistanceTemp = 10000.0;
	double ObjDistanceMin = DBL_MAX;
	int StartPoint = Dec_info.RoadPoints.size(),EndPoint = Dec_info.RoadPoints.size();
	in2_msgs::Point2D ObjPoint;
	for(int k = 0 ; k < Dec_info.RoadPoints.size(); k++)
	{
		if(DecLane[k].y - cut_dis >= 0)	
		{
			StartPoint = k;
			break;
		}
		
	}
	
	/*for(int i = StartPoint; i < EndPoint; i++)
	{
		for(int j = 360; j < 720; j++)
		{
			offsetY = fabs(RoadSurface.points[j].y - DecLane[i].y);
			offsetX = fabs(RoadSurface.points[j].x - DecLane[i].x);
			if(offsetY <= 5.0)
			{
				OffsetTemp = sqrt(offsetX*offsetX + offsetY * offsetY);
				if(OffsetTemp < OffsetMin)     
				{
					OffsetMin = OffsetTemp;
					ObjPoint = DecLane[i];	
				}
			}
		}
		if(OffsetMin <= 2.0)  ObjDistanceTemp = sqrt(DecLane[i].x*DecLane[i].x + DecLane[i].y*DecLane[i].y);
		if(ObjDistanceTemp < ObjDistanceMin)  ObjDistanceMin = ObjDistanceTemp;
		OffsetMin = 100.0;
	}*/
	
	for(int i = StartPoint; i < EndPoint; i++)
	{
		OffsetMin = 9999.9;
		for(int j = 0; j < 720; j++)
		{
			if (RoadSurface.points[j].y < cut_dis) continue;
			
			offsetY = fabs(RoadSurface.points[j].y - DecLane[i].y);
			offsetX = fabs(RoadSurface.points[j].x - DecLane[i].x);
			if(offsetY < 8.0 && offsetX < 8.0)
			{
				OffsetTemp = sqrt(offsetX*offsetX + offsetY * offsetY);
				if(OffsetTemp < OffsetMin)     
				{
					OffsetMin = OffsetTemp;
					ObjPoint = RoadSurface.points[j];
				}
			}
		}
		
		if(OffsetMin < 1.85) {
			ObjDistanceMin = sqrt(DecLane[i].x*DecLane[i].x + DecLane[i].y*DecLane[i].y);
			int x, y;
			
			obs_xy.x = DecLane[i].x * 50.0 + 1500.0;
			obs_xy.y = DecLane[i].y * 50.0 + 1000.0;
			
			x = 1500 + DecLane[i].x / 0.02;
			y = 1000 + DecLane[i].y / 0.02;
			cvCircle(MapShow,cvPoint(x,2*LOCAL_COOR_LENGTH - 1 - y),20,CV_RGB(0,255,255),-1);
			//cout << "DecLane[i]: (" << DecLane[i].x << ", " << DecLane[i].y << ")" << endl;
			
			x = 1500 + ObjPoint.x / 0.02;
			y = 1000 + ObjPoint.y / 0.02;
			cvCircle(MapShow,cvPoint(x,2*LOCAL_COOR_LENGTH - 1 - y),20,CV_RGB(255,0,0),-1);
			
			//cout << "ObjPoint: (" << ObjPoint.x << ", " << ObjPoint.y << ")" << endl;
			
			sprintf(strtemp, "OffsetMin: %.2f", OffsetMin);
			cvPutText(MapShow, strtemp, cvPoint(50,100), &font, CV_RGB(255,255,255));
			
			break;
		}
	}
	
	//printf("ObjDistance:  %f\n",ObjDistanceMin);
	
	return ObjDistanceMin;
}


void transferRoadSurface(const in2_msgs::RoadSurface::ConstPtr &road_surface)
{
	RoadSurface = *road_surface;
	
	// clear
	cv::Mat empty_mat = cv::Mat::zeros(LOCAL_COOR_WIDTH, 2*LOCAL_COOR_LENGTH, CV_8UC3);
	MapShow->imageData = (char*)empty_mat.data;
	
	std_msgs::Float64 FarObjDistanceMsg;
	FarObjDistance = Far_Obj_Check();
	
	if (FarObjDistance < 200.0) {
		filter_cnt++;
	} else {
		filter_cnt = 0;
	}
	if (filter_cnt >= 4) {
		FarObjDistanceMsg.data = FarObjDistance;
	} else {
		FarObjDistanceMsg.data = DBL_MAX;
	}
	
	dis_to_farobj_pub.publish(FarObjDistanceMsg);
	xy_of_farobj_pub.publish(obs_xy);
	display();
}

void transferDecInfo(const in2_msgs::DecInfo::ConstPtr &Dec_info_ptr)
{
	Dec_info = *Dec_info_ptr;
	is_Dec_info_rcv = true;
	//printf("GINInfo:  %d\n",1 );
	for(int i = 0; i < Dec_info.RoadPoints.size(); i ++)
	{
		DecLane[i] = convENUtoLocal(Dec_info.RoadPoints[i].x,Dec_info.RoadPoints[i].y, 
			XX, YY, ZZ);
		//printf("%f %f\n", DecLane[i].x, DecLane[i].y);
	}
}

void transferInsInfo(const in2_msgs::InsInfo::ConstPtr &ins_msg)
{
	ins = *ins_msg;
	XX = ins.position.x;
	YY = ins.position.y;
	ZZ = ins.attitude.z;
	ins_recv = true;
	Timer_ins = ins.sendtime;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "far_obj_check");
	ros::NodeHandle node;
 	
 	road_surface_sub_ = node.subscribe("RoadSurface",10,&transferRoadSurface,ros::TransportHints().tcpNoDelay(true));
 	decInfo_sub_ = node.subscribe("dec_info",10, &transferDecInfo, ros::TransportHints().tcpNoDelay(true));
 	ins_sub_ = node.subscribe("InsInfo",10,&transferInsInfo,ros::TransportHints().tcpNoDelay(true));
	
	dis_to_farobj_pub = node.advertise<std_msgs::Float64>("dis_to_farobj",10);
	xy_of_farobj_pub = node.advertise<in2_msgs::Point2D>("xy_of_farobj",10);
	
	MapShow   = cvCreateImage(cvSize(LOCAL_COOR_WIDTH, 2*LOCAL_COOR_LENGTH),IPL_DEPTH_8U,3);
	MapShow_S = cvCreateImage(cvSize(750,2*750),IPL_DEPTH_8U,3);
	double hScale = 3;
	double vScale = 3;
	int lineWidth = 6;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC,hScale,vScale,0,lineWidth);
	
	/*while(ros::ok())
	{
		ros::spinOnce();
	}*/
	ros::spin();

	ros::shutdown();
	
	return 0;
}
