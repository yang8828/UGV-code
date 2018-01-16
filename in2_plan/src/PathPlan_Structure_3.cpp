// MoPlan_UnStructure.cpp : \\
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <in2_msgs/LaneInfoV2.h>
#include <in2_msgs/InsInfo.h>
#include <in2_msgs/TcpGeneral.h>
#include <in2_msgs/Point2D.h>
//#include <in2_velodyne/ScanInfo.h>
#include <in2_msgs/GisInfo.h>
#include <in2_msgs/UdpGeneralShort.h>
#include <in2_msgs/RoadEdge.h>
#include <in2_msgs/DecInfo.h>
#include <in2_msgs/VehicleInfo.h>
#include <in2_msgs/TrafficLights.h>
#include <in2_msgs/LaneMarks.h>
#include <in2_msgs/IbeoObjects.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <ctime>

#include "stdafx.h"
#include "CreatObsMap.h"
#include "DrawPlanLines.h"
#include "Node.h"
#include "FirstPlan.h"
#include "C_Ins.h"
#include "GetParaLines.h"
#include "avoidobstacle.h"
#include <stdio.h>
#include <stdlib.h>

#define GIS_INTERVAL 0.4 // m
#define Offset 0.7 //Keep to the right
#define CenterReducePer 1.0
#define Obstaclecostweight 12.0//25km
#define Rulecostweight 2.5//25km
#define RULECOSTWEIGHT2   2.5
double Rulecostweight2 = RULECOSTWEIGHT2;//25km
#define Obstaclecostweight2 12.0//25km
#define LANE_WEIGHT 0.015
double lane_weight = LANE_WEIGHT;
#define frontoffset 0.0
#define Switch_Prospect 55   //can see 22m
#define variance_threshold1 2000.0 //variance for check to change mode
#define variance_threshold2 2500.0 //variance for check to change mode
#define blockpercent_threshold2 0.85
#define blockpercent_threshold1 0.75
#define blockpercent2_threshold2 0.90
#define blockpercent2_threshold1 0.80
//#define lanechangespeed 15
//#define lanechangespeed2 15
#define WatchDogamount 2000 //100s
#define WatchDogamount4 200
#define speedfiler 30 //the speed of speed changing
#define speedfiler_acc 3
#define carwidth (1.8 / 0.02)
#define carlength (5.1 / 0.02)
#define filterup 1
#define filterdown 1
#define stoplanefilter 18
//#define speed_corner 80000
#define tfsnspeed 10000
#define forcestratege 0 //0:general,1:gis+avoid,2:only roadedge
//#define curlanedischage 0.9
//#define DistancecostMax 30.0//distance cost max
//#define DistancecostMax2 30.0//distance cost max
#define Crossselectlane 0
#define INS_Lanedistime 500.0
//#define M_PI 3.14

#define lanechangespeedMax 7
#define lanechangespeedMin 4
#define lanechangespeed2Max 7
#define lanechangespeed2Min 4 //yue xiao huan xian yue man
#define DistancecostMax_Max 150.0
#define DistancecostMax_Min 50.0
#define DistancecostMax2_Max 50.0 //yue da su du jia kuai yue bu rong yi huan xian 
#define DistancecostMax2_Min 35.0 //yue da su du jia kuai yue bu rong yi huan xian 

#define curlanedischageMax 1.0
#define curlanedischageMin 0.6 //yue xiao yue bu rong yi huan xian

#define linecostdistanceMax 1.0
#define linecostdistanceMin 0.55 //yue xiao yue yuan bi zhang

#define speed_gisUpLine 15000

#define GIS_PARA      0
#define EDGE_PARA     1
#define LANE_PARA     2
#define GIS_AVOID     3
#define GIS_NO_WAY    4
#define EDGE_NO_WAY   5
#define HOLD_NO_WAY   6
#define GIS_SOFT      7

bool filter_speed_up = false;
bool no_filter = false;

double k_spd = 1.0;
double k_lane = 1.0;
double g_distancefront=5000.0;
double g_distancefront_last = 5000.0;
 
//caracters for change: according to the current speed to change the values
int lanechangespeed = 15;
int lanechangespeed2 = 15;
long speed_corner = 80000;
double edge_width = 0.0;
bool no_bing_xian = false;
double dis_to_stop_line = DBL_MAX;
bool is_use_stop_line = false;
bool in_shi_gong = false;

double  DistancecostMax = 50.0;
double  DistancecostMax2 = 50.0;
double  curlanedischage = 0.9;
double  linecostdistance = 0.8;

double dis_to_far_obs = DBL_MAX;

std_msgs::Int8       decision_result;
bool is_decision_result_rcv = false;
ros::Subscriber decision_sub_;

int which_light = -1; // -1: do not care light
                      //  0: straight
                      //  1: left
                      //  2: right

/******************************************************************/
//static const std::string OPENCV_WINDOW = "Heightmap";

ros::Subscriber ibeo_objects_sub_;
in2_msgs::IbeoObjects ibeo_objects;
bool is_ibeo_objects_rcv = false;

ros::Subscriber gisRect_sub_;
in2_msgs::GisInfo gisRect;
bool gisrect_recv = false;

ros::Subscriber controlpanel_sub_;
long controlpanel_info;
bool controlpanel_recv;
in2_msgs::UdpGeneralShort ControlPanelInfo;

ros::Publisher tfsntfl_pub_;
std_msgs::Int16 tfsntfl;

ros::Publisher parkingcontrol_pub_;
std_msgs::Int16 parkingcontrol;

ros::Publisher gisback_pub_;
std_msgs::Int16 gisback;

ros::Subscriber scaninfo_sub_;
//in2_velodyne::ScanInfo scanInfo;

ros::Subscriber ParkingFinish_sub_;
std_msgs::Int16 ParkingFinishmsg;

ros::Subscriber PassengerDetected_sub_;
std_msgs::Int16 PassengerDetectedmsg;

ros::Subscriber ins_sub_;
in2_msgs::InsInfo ins;
bool ins_recv = false;

ros::Subscriber dis_to_far_obs_sub_;
bool is_far_obs_rcv = false;

ros::Subscriber xy_of_far_obs_sub_;
in2_msgs::Point2D obs_xy;

ros::Subscriber GIS_sub_;
in2_msgs::UdpGeneral gis;
bool gis_received = false;

ros::Subscriber roadedge_sub_;
in2_msgs::RoadEdge roadedge;

ros::Subscriber TREK_sub_;
in2_msgs::TcpGeneral trek;
in2_msgs::TcpGeneral trek_prev;
bool trek_received = false;
bool trek_init_flag = true;

ros::Subscriber vehicle_info_sub_;
in2_msgs::VehicleInfo vehicle_info;
bool is_vehicle_info_rcv = false;

ros::Subscriber traffic_lights_sub_;
in2_msgs::TrafficLights traffic_lights;
bool is_traffic_lights_rcv = false;

ros::Subscriber lanemodelinfo_sub_;
in2_msgs::LaneInfoV2 laneModelInfo;
bool lanemodelinfo_recv;

ros::Subscriber tfl_sub_;
in2_msgs::UdpGeneralShort tfl;
bool tfl_recv;

ros::Subscriber tfsn_sub_;
in2_msgs::UdpGeneralShort tfsn;
bool tfsn_recv;

ros::Subscriber movtgtnum_sub_;
in2_msgs::UdpGeneralShort MovTgtNum;
bool movtgtnum_recv;

ros::Subscriber lane_marks_sub;
in2_msgs::LaneMarks lane_marks;
bool is_lane_marks_rcv;

ros::Subscriber scan_info_v2_sub_;

ros::Publisher dec_info_pub_;
in2_msgs::TcpGeneral tcp_msg;
in2_msgs::DecInfo    dec_msg;
/******************************************************************/
long int traffic_lights_dog = 0;
int match_state;//1:before 2:on 3:end
long speed_gis;
double max_speed_temp = 60.0;
long drive_state;
int cross_state;//0 1 2: none light,stright,left,right;3 4 5:light,stright,left,right;
double cross_dist;//m
double end_dist;// match states = 3 is effctive
double dev_angle;
CvPoint Gis_Lane[1000];
CvPoint Gis_LaneBefore[1000];
CvPoint Gis_Lane_Ori[1000];
CvPoint LaneIncar[3][1000];
CvPoint LaneIncarforlast[3][1000];
CvPoint Laneparatemp[3][1000];
int Gis_Lanenum;
int Gis_SelectLanenum;//0xff is any is ok 
int Gis_TurnDirection;//0: stright 1:left 2:right
int overpassflag;//0: is not overpassflag; 1:is overpassflag 
int islight;//near cross, islight is 0 or 1, others is 2
int is_shinroad = 0;//0:none avoid 1: avoid 
long specialnum;
int tfsn_flag;//0:none,2-10:know,1:don't know
long valuetemp = 0;
double XX=0;
double YY=0;
double ZZ=0;
double XX_used=0;
double YY_used=0;
double ZZ_used=0;
double XX_last = 0;
double YY_last = 0;
double ZZ_last = 0;
bool INSflag = false;
int coutfilter = 0;
double crossdistanceins = 10000000000.0;
double crossdistancetemp = 10000000000.0;
double insdistance = 0.0;
double tempdist;
double blockpercent = 0.0;
double blockpercent2 = 0.0;

double distance2car;
double changelanedueper;

double cur_angle = 0.0;
double last_angle = 0.0;
double old_angle_coe = 0.6;

double gis_left_para_width = DBL_MAX; // m
double gis_right_para_width = DBL_MAX; // m
double gis_para_width_invalid = 999; // m

int forwd=1;
int lturn=1;
int rturn=1;
int lalarm;
int ralarm;
int upalarm;
bool IsEstop;
bool IsEstop2;
bool IsEstop3;
long int estop_3_cnt = 0;
bool force_move = false;
long int force_move_tmr = 0;

bool FirstTime = true;
bool CrossDetFlag = true;
bool ManDetFlag = true;

int moveobs_num = 0;

double Cur_EV_Speed = 0; 
double Cur_EV_Orient = 0; 
double Cur_EV_Odometer = 0.0;
double Cur_EV_Odometerlast = 0.0;

CvPoint roadpts1_lane[1000];
CvPoint roadpts2_lane[1000];
CvPoint roadpts_lane[1000];
CvPoint roadpts_Planner[1000];
CvPoint roadpts_Sent[1000];
CvPoint PlannerFollow_lane[1000];
CvPoint Select_bestlane[1000];
CvPoint Select_bestlanelast[1000];
CvPoint Select_nearestlane[1000];
CvPoint Plan_nearestlane[1000];
CvPoint Select_sentlane[1000];
CvPoint traffic_lane[150][31];
CvPoint traffic_lanePro[150][1000];
CvPoint plines[500][1000];
int pathwaynum = 0;
int laneincarnum;
int trafficlanenum = 0;
int trafficlanecenterrnum = 0;

int tfsn_num;
int tfsn_X[100];
int tfsn_Y[100];
int tfsn_type[100];

bool is_in_fee = false;

void processGisLane();
void processRoadLane();
void FindEdgePoint(double Angle,double dis,double *p1x,double *p1y,double *p2x,double *p2y);
void processTrafficLane();
void SelectBestWay();
void SelectBestWay2();
void SelectNearestWay();
void SelectNearestWay2();
void SelectSentWay();
void SelectSentWay2();
void checkDriveMode();
bool Issamecluster();
long Caculatespeed();
long Caculatespeed2();
long calcSpeedFull();
//void processTFL();
void processTFSN();
void EStop();
bool IsRedLight();
void EstopWatchDog();
void Whetherstart();
void display();
void mcvSendRoadPoints(CvPoint *roadpoints,long &speed);
void paralane(CvPoint* lanetopara, CvPoint point, CvPoint* outputlane);
double getLineCost(CvPoint* pts, int start);
void processControlPanel();
void doRT2(double src_x,double src_y,double &dst_x,double &dst_y,double T_NEx_src,double T_NEy_src,double T_NEx_dst,double T_NEy_dst,double azimuth_src,double azimuth_dst);
int SelectPathIndex;
int NearcarLaneIndex;
int NearPlanLaneIndex;
int SelectPlanPathIndex;
int no_filt_SelectPlanPathIndex;
double SelectPlanPathIndex_last = 250;
double SelectPlanPathIndex_dbl;
int selectnum;
long speedsent;
long speedsent_last = 0;
long WatchDogcount = 0;
long WatchDogcount4 = 0;
int TFSNTFLchange=0;//0-none, 1-TFSN, 2-TFL, 3-all  for sent
int parkingcontrolnum=0;//0-none, 1-true
int ParkingFinishnum = 0;//0-none,1-true
int PassengerDetectednum = 0;
int PassengerDetectednumDel = 0;
int GISbackcar = 0;//0-forward, 1-back
int Laneuseful = 1;
int Laneuseful2 = 1;

int DriveMode = 0;//0-roadedge, 1-gis
int strategemode = 1;//1-lane, 2-roadedge planner, 3-gis planner
int strategemodelast = 0;
double variance;
double variancelast = 0.0;

double crossdistance;
double endlinedistance = 10000000000.0;

int speedlimit;
int leftlen,rightlen;
int planlanenumber;
int curlaneid = 250;
int planst, planed;

double Timer_ins;
double Timer_lanemodel;
C_InsFrameHis InsFrameHis;

/******************************************************************/

cv::Mat obs_mat;
cv::Mat lane_mat;
cv::Mat img_show;

IplImage * MoPlanMap;
IplImage * PlanLinesMap;
IplImage * FirstPlanMap;
IplImage * CostMap;
IplImage * CostMapPre;
IplImage * LaneMap;
IplImage * LaneMapPre;
IplImage * PlanMapShow;
IplImage * PlanMapShow_S;
CCreatObsMap CreatObsMap;
CDrawPlanLines DrawPlanLines;
CFirstPlan FirstPlan;
CvFont font;
double hScale = 2;
double vScale = 2;
int lineWidth = 4;
char strtemp[sizeof(double)*8+1];

CvPoint* TargetPoint;
CvPoint* TargetPointPre;
CvPoint  FTargetPointPre;
CvPoint  FTargetPoint;
CvPoint StartPoint;
CvPoint StartPointPre;
CvScalar PointVal;
CvPoint* FinalPathPoints = new CvPoint[1000000];
CvPoint* PathPointsInCar = new CvPoint[1000000];
CvPoint* PathPointsSent = new CvPoint[1000];

CvPoint3D64d CarPose;
CvPoint2D64d TargetInGlobal;
CvPoint2D64d StartInGlobal;

int* FirstBreakPNum;

cv::Mat ImageGet;

bool Planfinished = true;
int AllTargetNum;
int CarAngleNow;
int FinalPathPointNum;
int PathPointsInCarNum;
int col,row;

bool FirstPlanSuccess;
bool *SecondPlanSuccess;

double Target_E;
double Target_N;
double angleTarget;
CvPoint sent_pts_last[1000];
bool is_sent_pts_last_init = false;
in2_msgs::UdpGeneral astar2planner_msg;
ros::Publisher astar_publisher_;
float two_lines_filter = 0.8;

CvPoint convENUto3000( double x, double y, double x0, double y0, double azimuth );
CvPoint2D64f conv3000toENU(int x, int y, double x0, double y0, double azimuth );
void sqToMeter(int a, int b, float &x, float &y);
void meterToSq(double x, double y, int &a, int &b);
void twoLinesFilter(CvPoint *last, CvPoint *current, CvPoint *out, float lazy_coe);

void pathFilter() {

// ****** line to line filter ****** //
	if (1||is_sent_pts_last_init) {
	
		//twoLinesFilter(sent_pts_last, roadpts_Sent, roadpts_Sent, (100-lanechangespeed)*0.01/*two_lines_filter*/);
		//for (int i = 0; i < 301; i++)
		//	sent_pts_last[i] = roadpts_Sent[i];
		
		double lazy_coe = (100-lanechangespeed)*0.01;
		double id = SelectPlanPathIndex_last * lazy_coe + SelectPlanPathIndex * (1.0f - lazy_coe);
		SelectPlanPathIndex = (int)(id + 0.5);
		SelectPlanPathIndex_dbl = id;
	}
	else {
	
		//for (int i = 0; i < 301; i++)
		//	sent_pts_last[i] = roadpts_Sent[i];
		//is_sent_pts_last_init = true;
		
		SelectPlanPathIndex_last = SelectPlanPathIndex;
		//is_sent_pts_last_init = true;
	}
}

void twoLinesFilter(CvPoint *last, CvPoint *current, CvPoint *out, float lazy_coe) {

	if (filter_speed_up) {
		lazy_coe = lazy_coe * 0.81;
	}
	
	/*if (no_filter) {
		lazy_coe = 0.0;
	}*/
	
	lazy_coe = lazy_coe * k_lane;
	
	CvPoint2D64f last_register[301];
	for (int i = 0; i < 301; i++) {
		doRT2(last[i].x, last[i].y, last_register[i].x, last_register[i].y,
		      XX_last,YY_last,XX,YY,ZZ_last,ZZ);
	}
	
	for (int i = 0; i < 301; i++) {
	
		out[i].x = last_register[i].x * lazy_coe + current[i].x * (1.0f - lazy_coe);
		out[i].y = last_register[i].y * lazy_coe + current[i].y * (1.0f - lazy_coe);
	}
}

CvPoint GetStartPoint(CvPoint PointNow)
{
	int r = 3;
	while(r<120) //find for 2.4m length
	{
		for(int angle=0;angle<16;angle++)
		{
			int yy = (int)( ((double)r)*sin(angle*22.5*M_PI/180.0));
			int xx = (int)( ((double)r)*cos(angle*22.5*M_PI/180.0));
			/*printf(" %d \n",r);
			printf(" %d \n",xx);printf(" %d \n",yy);*/
			if( ((PointNow.x+xx) < MoPlanMap->width) && ((PointNow.x+xx)>=0) && ((PointNow.y+yy) < MoPlanMap->height) && ((PointNow.y+yy) >= 0) ) 
			{
				PointVal = cvGet2D( CostMap,(PointNow.y+yy),(PointNow.x+xx) );
				if(PointVal.val[0]<160)
				{
					return(cvPoint(PointNow.x+xx,PointNow.y+yy));
				}
			}						
		}

		r=r+3;
	}
	return(cvPoint(-1,-1));
}

CvPoint tranfer(double x, double y, double azuith)//   
{
            double resultX, resultY;
            int finalX, finalY;
			double cosa = cos(azuith * M_PI / 180);
            double sina = sin(azuith * M_PI / 180);
            resultX = x * cosa - y * sina;
            resultY = x * sina + y * cosa;//
            finalX = (long)(resultX * 50.0 + 1500);
            finalY = (long)(resultY * 50.0 + 1000);
            bool flag = false;
            if (finalX < 1)
            {
                finalX = 1;
                flag = true;
            }
            else if (finalX > 2999)
            {
                finalX = 2999;
                flag = true;
            }
            if (finalY < 1)
            {
                finalY = 1;
                flag = true;
            }
            else if (finalY > 2999)
            {
                finalY = 2999;
                flag = true;
            }
            CvPoint temp;
            temp.x=finalX;
            temp.y=finalY;
            return temp;
}

CvPoint GetTargetInMap(CvPoint3D64d CarPoseNow, CvPoint2D64d TargetGivenByTop)//konw the point in global, change into car index, regard the car as center
{
	double dis_X,dis_Y;
	dis_X = abs(TargetGivenByTop.x-CarPoseNow.x);
	dis_Y = abs(TargetGivenByTop.y-CarPoseNow.y);

	int TargetX,TargetY;
	double angle;
	double TargetXGlobal,TargetYGlobal;

	if(TargetGivenByTop.x>CarPoseNow.x)
	{
		angle = 180.0*atan((TargetGivenByTop.y-CarPoseNow.y)/(TargetGivenByTop.x-CarPoseNow.x))/M_PI;
		if(angle<0.0)
			angle = 360.0 + angle ;
	}
	else if(TargetGivenByTop.x<CarPoseNow.x)
	{
		angle = 180.0 + 180.0*atan((TargetGivenByTop.y-CarPoseNow.y)/(TargetGivenByTop.x-CarPoseNow.x))/M_PI;
	}
	else if(TargetGivenByTop.y>CarPoseNow.y)
		angle = 90.0;
	else 
		angle = 270.0;
	//printf("angle: %f\n",angle);

	if( (dis_X>=25.0) || (dis_Y>=25.0) )//
	{
		if( ((angle>=45.0)&&(angle<90.0)) || ((angle>90.0)&&(angle<135.0)) )
		{
			TargetXGlobal = 25.0 * (TargetGivenByTop.x-CarPoseNow.x)/(TargetGivenByTop.y-CarPoseNow.y) + CarPoseNow.x;
			TargetYGlobal = CarPoseNow.y + 25.0;
		}
		else if( ((angle>=135.0)&&(angle<180.0)) || ((angle>180.0)&&(angle<225.0)) )
		{
			TargetYGlobal = CarPoseNow.y - 25.0*(TargetGivenByTop.y-CarPoseNow.y)/(TargetGivenByTop.x-CarPoseNow.x);
			TargetXGlobal = CarPoseNow.x -25.0;
		}
		else if( ((angle>=225.0)&&(angle<270.0)) || ((angle>270.0)&&(angle<315.0)) )
		{
			TargetXGlobal = -25.0 * (TargetGivenByTop.x-CarPoseNow.x)/(TargetGivenByTop.y-CarPoseNow.y) + CarPoseNow.x;
			TargetYGlobal = CarPoseNow.y - 25.0;
		}
		else if( ((angle>=315.0)&&(angle<360.0)) || ((angle>0.0)&&(angle<45.0)) )
		{
			TargetYGlobal = CarPoseNow.y + 25.0*(TargetGivenByTop.y-CarPoseNow.y)/(TargetGivenByTop.x-CarPoseNow.x);
			TargetXGlobal = CarPoseNow.x +25.0;
		}
		else if(angle==0.0)
		{
			TargetYGlobal = CarPoseNow.y;
			TargetXGlobal = CarPoseNow.x + 25.0;
		}
		else if(angle==90.0)
		{
			TargetXGlobal = CarPoseNow.x;
			TargetYGlobal = CarPoseNow.y + 25.0;
		}
		else if(angle==180.0)
		{
			TargetYGlobal = CarPoseNow.y;
			TargetXGlobal = CarPoseNow.x - 25.0;
		}
		else 
		{
			TargetXGlobal = CarPoseNow.x;
			TargetYGlobal = CarPoseNow.y - 25.0;
		}
	}
	else
	{
		TargetXGlobal = TargetGivenByTop.x;
		TargetYGlobal = TargetGivenByTop.y;	
	}

	TargetX = (int)( (TargetXGlobal - CarPoseNow.x) * 50.0 + 1500.0 );
	TargetY = (int)( (TargetYGlobal - CarPoseNow.y) * 50.0 + 1500.0 );
	
	if(TargetX<0)
		TargetX=0;
	if(TargetX>2999)
		TargetX=2999;
	if(TargetY<0)
		TargetY=0;
	if(TargetY>2999)
		TargetY=2999;

	int TargetXX;
	TargetXX = TargetX;
	int TargetYY;
	TargetYY = 3000 - TargetY;

	return( cvPoint(TargetXX,TargetYY) );	
}

bool AStarPlan(double east, double north, double azimuth, double StartX, double StartY, double angleroad1,double TargetX, double TargetY, double angleroad2)
{	  

	FirstPlanSuccess = false;	

	CarPose.x = east;
	CarPose.y = north;
	CarPose.z = azimuth;

	/**************************************************************************************************/
	//obtain target in car coord
	angleTarget = angleroad2;   //angle1 and angle2's +direction is +y direction and shun shi zhen 	
	if( (angleTarget>=0.0)&&(angleTarget<=90.0) )
		angleTarget = 90.0 - angleTarget;
	else 		
		angleTarget = 450.0 - angleTarget;
	FTargetPointPre.x = TargetX;
	FTargetPointPre.y = 3000 - TargetY;
	
	//obtain start in car coord
	if(angleroad1>=360.0)
		angleroad1=0.0;
	if(angleroad1<0.0)
		angleroad1=0.0;
	int anglecar;
	anglecar = angleroad1;
	CarAngleNow = 23 - anglecar/15;  //angle1 and angle2's +direction is +y direction and shun shi zhen 	
	StartPointPre.x = StartX;
	StartPointPre.y = 3000 - StartY;
	/**************************************************************************************************/

	PointVal = cvGet2D(CostMap,StartPointPre.y,StartPointPre.x);
	if(PointVal.val[0]>180)
	{
		StartPoint = GetStartPoint(StartPointPre);
		if((StartPoint.x==-1)&&(StartPoint.y==-1))
		{
			printf("StartPoint can't been found!\n");
			return (false);// can't plan using A*
		}
	}
	else
		StartPoint = StartPointPre;

	PointVal = cvGet2D(CostMap,FTargetPointPre.y,FTargetPointPre.x);
	if(PointVal.val[0]>180)
	{
		FTargetPoint = GetStartPoint(FTargetPointPre);
		if((FTargetPoint.x==-1)&&(FTargetPoint.y==-1))
		{
			printf("TargetPoint can't been found!\n");
			return (false);
		}
	}
	else
		FTargetPoint = FTargetPointPre;
	
	FirstPlanMap = FirstPlan.FirstPlan(MoPlanMap);
	
	if(!FirstPlanSuccess)
	{
		printf("FirstPlanning Error!\n");
		return (false);
	}

	if((FirstPlan.FPathpointsNum % 30)!=0)
	{
		TargetPoint = new CvPoint[FirstPlan.FPointNum+1];
		SecondPlanSuccess = new bool[FirstPlan.FPointNum+1];
		FirstBreakPNum = new int[FirstPlan.FPointNum+1];
		AllTargetNum = FirstPlan.FPointNum+1;

		for(int i=0;i<FirstPlan.FPointNum;i++)
		{
			TargetPoint[i]=FirstPlan.FPathpoints[FirstPlan.FPathpointsNum-30*(i+1)];
			FirstBreakPNum[i]=FirstPlan.FPathpointsNum-30*(i+1);
			cvCircle(FirstPlanMap,TargetPoint[i],1,CV_RGB(0,255,255),3);
		}

		TargetPoint[FirstPlan.FPointNum]=FirstPlan.FPathpoints[0];
		FirstBreakPNum[FirstPlan.FPointNum]=0;
		cvCircle(FirstPlanMap,TargetPoint[FirstPlan.FPointNum],1,CV_RGB(0,255,255),3);

	}
	else
	{
		TargetPoint = new CvPoint[FirstPlan.FPointNum];
		SecondPlanSuccess = new bool[FirstPlan.FPointNum];
		FirstBreakPNum = new int[FirstPlan.FPointNum];
		AllTargetNum = FirstPlan.FPointNum;

		for(int i=0;i<FirstPlan.FPointNum;i++)
		{
			TargetPoint[i]=FirstPlan.FPathpoints[FirstPlan.FPathpointsNum-30*(i+1)];
			FirstBreakPNum[i]=FirstPlan.FPathpointsNum-30*(i+1);
			cvCircle(FirstPlanMap,TargetPoint[i],1,CV_RGB(0,255,255),3);
		}
	}

	PlanLinesMap = DrawPlanLines.DrawPlanLines(FirstPlanMap);//secondplan
	
	cvCircle(PlanLinesMap,StartPoint,1,CV_RGB(255,255,0),3); 
	cvCircle(PlanLinesMap,FTargetPoint,1,CV_RGB(0,255,255),3);

	if(FirstPlanSuccess)
	{  
		if(AllTargetNum>0)
		{
			int TargetBreakNum = 0;
			for(int m=0;m<AllTargetNum;m++)
			{
				if(!SecondPlanSuccess[m])
				{
				    break;
				}
				TargetBreakNum++;
			}

			if(TargetBreakNum==AllTargetNum) 
			{
				FinalPathPointNum = 0;
				for(int i=0 ; i < DrawPlanLines.PathPointsNum ; i++)
				{
					FinalPathPoints[FinalPathPointNum++] = DrawPlanLines.PathPoints[i];
				}
			}//conduct the second plan completely
			else if(TargetBreakNum==0) 
			{				
				FinalPathPointNum = 0;
				for(int i=FirstPlan.FPathpointsNum-1 ; i >=0 ; i--)
				{
					FinalPathPoints[FinalPathPointNum++] = FirstPlan.FPathpoints[i];
				}
			}//only finish the first plan
			else
			{
				FinalPathPointNum = 0;
				for(int i=0 ; i < DrawPlanLines.PathPointsNum ; i++)
				{
					FinalPathPoints[FinalPathPointNum++] = DrawPlanLines.PathPoints[i];
				}
				for(int i=FirstBreakPNum[TargetBreakNum-1]; i>=0 ;i--)
				{
					FinalPathPoints[FinalPathPointNum++] = FirstPlan.FPathpoints[i];
				}//wan cheng pin jie, qian ban duan yong secondplan, hou ban duan yong firstplan.
			}
		}
		else
		{
			FinalPathPointNum = 0;
			for(int i=0 ; i < FirstPlan.FPathpointsNum ; i++)
			{
				FinalPathPoints[FinalPathPointNum++] = FirstPlan.FPathpoints[i];
			}
		}
	}
	else
	{
		for(int i =0 ; i<301 ; i++)
		{
			FinalPathPoints[i].x = 1500;//(int)((double)StartPoint.x + (double)i * Xstep);
			FinalPathPoints[i].y = 1500;//(int)( k * ((double)FinalPathPoints[i].x-(double)StartPoint.x) + (double)StartPoint.y );
		}
		FinalPathPointNum = 301;
	}// never conduct this part

	for(int k=0;k<FinalPathPointNum;k++)
	{
		PathPointsInCar[k].x = FinalPathPoints[k].x;
		PathPointsInCar[k].y = 3000 - FinalPathPoints[k].y;
		if(k<(FinalPathPointNum-1))
			cvLine(PlanMapShow,FinalPathPoints[k+1], FinalPathPoints[k], CV_RGB(255,0,0), 2);
	}
	cvCircle(PlanMapShow,StartPoint,1,CV_RGB(0,255,0),3);
	cvCircle(PlanMapShow,FTargetPoint,1,CV_RGB(0,255,255),3);
	ROS_ERROR("FinalPathPointNum : %d", FinalPathPointNum);
	
	PathPointsInCarNum = FinalPathPointNum;
	if(PathPointsInCarNum>301)
	{
		double Step;
		Step = ( (double)(PathPointsInCarNum-1) )/300.0;
		for( int j =0;j<301;j++)
		{
			int num;
			num = (int)((double)j)*Step;
			if(num>(PathPointsInCarNum-1))
				num = PathPointsInCarNum-1;

			PathPointsSent[j] =  PathPointsInCar[num];
		}
	}
	else if(PathPointsInCarNum>0)
	{
		for(int t=0;t<PathPointsInCarNum;t++)
		{
			PathPointsSent[t]=PathPointsInCar[t];
		}
		for(int y=PathPointsInCarNum;y<301;y++)
		{
			PathPointsSent[y] = PathPointsInCar[PathPointsInCarNum-1];
		}
	}
	else
	{
		delete[] TargetPoint;
		delete[] SecondPlanSuccess;
		delete[] FirstBreakPNum;
		return(false);
	}
	//select 301 points from the plan route

	/*
	for(int i=0;i<301;i++)
	{
		astar2planner_msg.x[i] = PathPointsSent[i].x;
		astar2planner_msg.y[i] = PathPointsSent[i].y;
	}
	for(int i=0; i<255; i++)
	{
		astar2planner_msg.data[i] = 0;
	}
	astar_publisher_.publish(astar2planner_msg);
	*/

	delete[] TargetPoint;
	delete[] SecondPlanSuccess;
	delete[] FirstBreakPNum;
	return (true);
}

void imageLaneConvert(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	cv::InputArray cv_in_arr = cv_ptr->image;
	lane_mat = cv_in_arr.getMat();
}

/****************************************************************************************************/
void imageObsConvert(const sensor_msgs::ImageConstPtr& msg)
{

	if (!gisrect_recv) return;
	
	XX_used = XX;
	YY_used = YY;
	ZZ_used = ZZ;
	for (int i = 0; i < gisRect.RoadPoints.size(); i++) {
	
		CvPoint pt_3000 = convENUto3000(gisRect.RoadPoints[i].x, gisRect.RoadPoints[i].y,
							XX_used, YY_used, ZZ_used);
		
		Gis_Lane[i].x = pt_3000.x;
		Gis_Lane[i].y = pt_3000.y;
	}
	

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}				
	double 	StartX, StartY, angle1, TargetX, TargetY, angle2;	
	bool temp;

	cv::InputArray Heightmap = cv_ptr->image;
	obs_mat = Heightmap.getMat();//obtain the costmap
	col = obs_mat.cols;
	row = obs_mat.rows;
	//printf("%d,%d\n",col,row);
	if(FirstTime)
	{
		FirstTime = false;
		MoPlanMap = cvCreateImage(cvSize(3000,3000),IPL_DEPTH_8U,3);
		CostMapPre = cvCreateImage(cvSize(col,row),IPL_DEPTH_8U,1);
		CostMap = cvCreateImage(cvSize(3000,3000),IPL_DEPTH_8U,1);
		LaneMapPre = cvCreateImage(cvSize(col,row),IPL_DEPTH_8U,1);
		LaneMap = cvCreateImage(cvSize(3000,3000),IPL_DEPTH_8U,1);
		PlanMapShow = cvCreateImage(cvSize(3000,3000),IPL_DEPTH_8U,3);
		PlanMapShow_S = cvCreateImage(cvSize(750,750),IPL_DEPTH_8U,3);

		leftlen = 500;
		rightlen = 500;
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC,hScale,vScale,0,lineWidth);
	}

	CostMapPre->imageData = (char*)obs_mat.data;
	LaneMapPre->imageData = (char*)lane_mat.data;
	cvResize(CostMapPre,CostMap);
	cvResize(LaneMapPre,LaneMap);
	Mat obs_mat_3000(CostMap);
	Mat lane_mat_3000(LaneMap);
	//cvCvtColor(CostMap,PlanMapShow,CV_GRAY2RGB);
	//cvCvtColor(CostMap,MoPlanMap,CV_GRAY2RGB);
	
	Mat mattemp[3];
	img_show = Mat::zeros(3000, 3000, CV_8UC3);
	lane_mat_3000.copyTo(mattemp[0]); // blue
	obs_mat_3000.copyTo(mattemp[1]); // green
	obs_mat_3000.copyTo(mattemp[2]); // red
	merge(mattemp, 3, img_show);
	PlanMapShow->imageData = (char*)img_show.data;

	cvRectangle(PlanMapShow,cvPoint(1500-carwidth/2,1925-carlength/2),cvPoint(1500+carwidth/2,1925+carlength/2),CV_RGB(255,255,0),14,8,0);//draw the car in map


	//processGisLane();	
	//processTrafficLane();	
	//processRoadLane();//process the road lines and obtain the road edges and the rightline and draw on the show map		
	//checkDriveMode();
	
	int ext_mode = decision_result.data;
	
	Rulecostweight2 = RULECOSTWEIGHT2;
	if (ext_mode == GIS_PARA) {
		strategemode = 3;
		no_bing_xian = false;
	}
		
	else if (ext_mode == EDGE_PARA) {
		strategemode = 2;
	}
		
	else if (ext_mode == LANE_PARA) {
		strategemode = 1;
	}
		
	else if (ext_mode == GIS_AVOID) {
		strategemode = 5;
	}
	
	else if (ext_mode == GIS_NO_WAY) {
		strategemode = 3;
		no_bing_xian = true;
	}
		
	else if (ext_mode == GIS_SOFT) {
		strategemode = 3;
		Rulecostweight2 = 50.0;
		no_bing_xian = false;
	}
	
	else {
		strategemode = 3;
		no_bing_xian = false;
	}
	
	if(endlinedistance<100.0 * 50) { // near destination
		no_bing_xian = true;
	}
	
	
	switch(strategemode)
	{
		case 1:
			SelectNearestWay();//Select the way who is nearest to the car			
			SelectBestWay();//Select the way who is best
			if(Laneuseful==1)//if most lane is on the road
			{
				SelectSentWay();//filter			
				for(int i=0;i<301;i++)
				{
					roadpts_Sent[i]=Select_sentlane[i];
				}
				speedsent = Caculatespeed();
			}
			else//if most lane is not on the road
			{
				leftlen = 500;
				rightlen = 500;
				getParaLines(traffic_lanePro[pathwaynum-1], leftlen, rightlen, plines, planlanenumber, curlaneid, speedlimit, planst, planed);			
				SelectNearestWay2();
				SelectBestWay2();
				SelectSentWay2();
				for(int i=0;i<301;i++)
				{
					roadpts_Sent[i]=Select_sentlane[i];
				}
				speedsent = Caculatespeed2();
			}
		break;
		case 2:
			if (gis_left_para_width >= gis_para_width_invalid) {
				leftlen = 0.7 * edge_width / 0.02;
			} else {
				leftlen = gis_left_para_width * 50.0;
			}
			
			if (gis_right_para_width >= gis_para_width_invalid) {
				rightlen = 0.3 * edge_width / 0.02;
			} else {
				rightlen = gis_right_para_width * 50.0;
			}
			
			getParaLines(traffic_lanePro[pathwaynum-1], leftlen, rightlen, plines, planlanenumber, curlaneid, speedlimit, planst, planed);			
			SelectNearestWay2();
			SelectBestWay2();
			SelectSentWay2();
			for(int i=0;i<301;i++)
			{
				roadpts_Sent[i]=Select_sentlane[i];
			}
			speedsent = Caculatespeed2();
		break;
		case 3:
			/*if (crossdistance < 40*50 && crossdistance > 0*50) {
				leftlen = 800;
				rightlen = 800;
				//no_bing_xian = true;
			} else {
				leftlen = 800;
				rightlen = 800;
				//no_bing_xian = false;
			}*/
			
			if (gis_left_para_width >= gis_para_width_invalid) {
				leftlen = 250;
			} else {
				leftlen = gis_left_para_width * 50.0;
			}
			
			if (gis_right_para_width >= gis_para_width_invalid) {
				rightlen = 250;
			} else {
				rightlen = gis_right_para_width * 50.0;
			}
			
			getParaLines(Gis_Lane, leftlen, rightlen, plines, planlanenumber, curlaneid, speedlimit, planst, planed);			
			SelectNearestWay2();
			SelectBestWay2();
			if(DriveMode==0 && 0)
			{
				if(Laneuseful2==1)//if most lane is on the road
				{
					SelectSentWay2();
					for(int i=0;i<301;i++)
					{
						roadpts_Sent[i]=Select_sentlane[i];
					}
					speedsent = Caculatespeed2();
				}
				else
				{
					leftlen = 800;
					rightlen = 800;
					getParaLines(traffic_lanePro[pathwaynum-1], leftlen, rightlen, plines, planlanenumber, curlaneid, speedlimit, planst, planed);			
					SelectNearestWay2();
					SelectBestWay2();
					SelectSentWay2();
					for(int i=0;i<301;i++)
					{
						roadpts_Sent[i]=Select_sentlane[i];
					}
					speedsent = Caculatespeed2();
				}
			}
			else
			{
				SelectSentWay2();
				for(int i=0;i<301;i++)
				{
					roadpts_Sent[i]=Select_sentlane[i];
				}
				speedsent = Caculatespeed2();
			}
		break;
		case 4:
			for(int i=0;i<301;i++)
			{
				Select_bestlane[i]=traffic_lanePro[pathwaynum-1][i];
				roadpts_Sent[i]=traffic_lanePro[pathwaynum-1][i];
			}
			speedsent = Caculatespeed();
		break;
		case 5:
			RoamDirection = 0;
			processLaser(Gis_Lane);
			for(int i=0;i<301;i++)
			{
				roadpts_Sent[i]=roadpts_laser[i];
			}
			speedsent = speed_laser;
		break;
		case 6:
			RoamDirection = 1;
			processLaser(Gis_Lane);
			for(int i=0;i<301;i++)
			{
				roadpts_Sent[i]=roadpts_laser[i];
			}
			speedsent = speed_laser;
		break;
		case 7:
			RoamDirection = 2;
			processLaser(Gis_Lane);
			for(int i=0;i<301;i++)
			{
				roadpts_Sent[i]=roadpts_laser[i];
			}
			speedsent = speed_laser;
		break;
		case 8:
			for(int i=0;i<301;i++)
			{
				roadpts_Sent[i]=Gis_Lane[i];
			}
			speedsent = Caculatespeed2();
		break;
		case 9:
			for(int i=0;i<301;i++)
				roadpts_Sent[i]=Gis_Lane[i];
			speedsent = Caculatespeed2();
		break;
	}
	strategemodelast = strategemode;

	/*double distancetemp;
	double distancemintemp = 10000000000.0;
	int nearcarindextemp;	
	double angletemp;
	for(int i=0;i<301;i++)
	{
		distancetemp = sqrt(((double)roadpts_Sent[i].x-1500.0)*((double)roadpts_Sent[i].x-1500.0)+((double)roadpts_Sent[i].y-1000.0)*((double)roadpts_Sent[i].y-1000.0));
		if(distancetemp<distancemintemp)
		{
			distancemintemp = distancetemp;
			nearcarindextemp = i;
		}
	}
	distance2car = distancemintemp;
	if(distance2car>300.0)
		distance2car = 300.0;
	if(distance2car<18.0)
		distance2car = 18.0;
	
	if(strategemode==1||strategemode==2)
	{
		if(nearcarindextemp>10&&nearcarindextemp<290)
		{
			if(roadpts_Sent[nearcarindextemp-10].y==roadpts_Sent[nearcarindextemp+10].y)
			{
				if(roadpts_Sent[nearcarindextemp-10].x!=roadpts_Sent[nearcarindextemp+10].x)
					speedsent=15000;
				else
					changelanedueper = 1.0 - (distance2car-18.0)/340.0;					
			}
			else
			{
				angletemp = 180.0*atan((abs((double)roadpts_Sent[nearcarindextemp+10].x-(double)roadpts_Sent[nearcarindextemp-10].x))/(abs((double)roadpts_Sent[nearcarindextemp+10].y-(double)roadpts_Sent[nearcarindextemp-10].y)))/M_PI;
				if(angletemp<1.0)
					angletemp=1.0;
				if(angletemp>35.0)
					angletemp=35.0;
				//changelanedueper = ((1.0 - (distance2car-18.0)/340.0)*4.5+(1.0-(angletemp-1.0)/40.0)*5.5)/10.0;
				changelanedueper = (1.0 - (distance2car-18.0)/340.0)*(1.0-(angletemp-1.0)/35.0);
				//fprintf(stderr,"%f,%f",((1.0 - (distance2car-18.0)/340.0)*4.5)/10.0,((1.0-(angletemp-1.0)/40.0)*5.5)/10.0);
			}
		}	
		else {
			changelanedueper = 1.0 - (distance2car-18.0)/450.0;
		}
	}
	else {
		changelanedueper = 1.0 - (distance2car-18.0)/470.0;
	}*/
	
	/*
	if(speedsent>14000)
	{	
		speedsent = (long)((double)speedsent * changelanedueper);
		if(speedsent<10000)
			speedsent = 10000;
	}
	*/

	//speedsent = (speedsent*speedfiler + speedsent_last*(100-speedfiler))/100; 
	
	if (fabs(speedsent) > fabs(speedsent_last)) {
		speedsent = (speedsent*speedfiler_acc + speedsent_last*(100.0-speedfiler_acc))/100.0;
		if (speedsent > 300 + speedsent_last) {
			speedsent = 300 + speedsent_last;
		}
	}
	else {
		speedsent = (speedsent*speedfiler + speedsent_last*(100.0-speedfiler))/100.0;
	}
	
	// san huan
	//speedsent = (speedsent*speedfiler + speedsent_last*(100.0-speedfiler))/100.0;
	
	//speedsent_last = speedsent;
	
	EstopWatchDog();//ensure it won't be isEstop = true all the time 

	if(match_state==1)
	{
		if(forwd==1)
			valuetemp++;
		if(forwd==0)
			valuetemp--;
		if(valuetemp<0)
			valuetemp = 0;
		if(valuetemp>1000000)
			valuetemp = 1000000;
		//if((valuetemp<100)&&(controlpanel_info!='2'))//10s
		//	speedsent = 0;
	}
	
	//cout << "IsEstop: " << IsEstop << endl;
	//cout << "IsEstop2: " << IsEstop2 << endl;
	//cout << "IsEstop3: " << IsEstop3 << endl;

	if (IsEstop3 && !IsRedLight()) estop_3_cnt++;
	else                           		     estop_3_cnt = 0;
	
	if (estop_3_cnt > 200) {
		speedsent = 5000;
		force_move = true;
	} else if (force_move_tmr > 100) {
		force_move = false;
	}
	
	if (force_move) {
		force_move_tmr++;
	} else {
		force_move_tmr = 0;
	}
	
	if (!force_move && (IsEstop||IsEstop2||IsEstop3||(PassengerDetectednumDel==1))) {
		EStop();
	}
	else {
		//if (force_move) speedsent = 5000;
		
		double dis_filt = -(SelectPlanPathIndex_dbl - curlaneid) * 20;
		getOnePara(Gis_Lane, dis_filt, roadpts_Sent, 8);
		
		mcvSendRoadPoints(roadpts_Sent,speedsent);
		speedsent_last = speedsent;
	}

	/*tfsntfl.data = TFSNTFLchange;
	tfsntfl_pub_.publish(tfsntfl);

	if(ParkingFinishnum==1)
		parkingcontrolnum = 0;
	parkingcontrol.data = parkingcontrolnum;
	parkingcontrol_pub_.publish(parkingcontrol);

	gisback.data = GISbackcar;
	gisback_pub_.publish(gisback);
		
	XX_last = XX;
	YY_last = YY;
	ZZ_last = ZZ;	
	Cur_EV_Odometerlast = Cur_EV_Odometer;*/
	
	display();
}

//SCAN INFO CALLBACK
/*
void transferScanInfo(const in2_velodyne::ScanInfo::ConstPtr &scaninfo_msg)
{
	scanInfo = *scaninfo_msg;
	// scan from velodyne	
	for (int i=0;i<scanInfo.count;i++)
	{
		fused_scan[i] = cvPoint2D32f(scanInfo.x[i],scanInfo.y[i]);
	}
}
*/

void transferParkingFinish(const std_msgs::Int16::ConstPtr &ParkingFinish_msg)
{
	ParkingFinishmsg = *ParkingFinish_msg;
	// scan from velodyne	
	ParkingFinishnum = ParkingFinishmsg.data;
}

void PassengerDetectedDel(const std_msgs::Int16::ConstPtr &PassengerDetected_msg)
{
	PassengerDetectedmsg = *PassengerDetected_msg;
	// scan from velodyne	
	PassengerDetectednum = PassengerDetectedmsg.data;
}

//TFL CALLBACK
/*
void transferTFL(const in2_msgs::UdpGeneralShort::ConstPtr &tfl_msg)
{
	tfl = *tfl_msg;
	tfl_recv = true;
	processTFL();
}
*/

//TFSN CALLBACK
void transferTFSN(const in2_msgs::UdpGeneralShort::ConstPtr &tfsn_msg)
{
	tfsn = *tfsn_msg;
	tfsn_recv = true;
	processTFSN();
}

//MovTgt
void transferMovTgtNum(const in2_msgs::UdpGeneralShort::ConstPtr &movtgtnum_msg)
{
	MovTgtNum = *movtgtnum_msg;
	movtgtnum_recv = true;
	moveobs_num = MovTgtNum.data[1];
}

//GIS_RECT CALLBACK
void transferGISRect(const in2_msgs::GisInfo::ConstPtr &gisRect_msg)
{
	gisRect = *gisRect_msg;
	gisrect_recv = true;
	
	//cout << "gis callback!" << endl;
	
	cross_dist = gisRect.DistanceToNextCross;
	//if (cross_dist < 5.0) lane_weight = 0.0;
	//else                  lane_weight = LANE_WEIGHT;
	crossdistance = cross_dist * 50.0;
	
	match_state = gisRect.MissionState;
	
	speed_gis = gisRect.SpeedLimit * 1000;
	//speed_gis = 60 * 1000;

	//for(int i=0;i<301;i++)
	//{
	//	Gis_Lane[i].x = gisRect.x[i];
	//	Gis_Lane[i].y = gisRect.y[i];
	//}
	
	/*XX_used = XX;
	YY_used = YY;
	ZZ_used = ZZ;
	int break_id = gisRect.RoadPoints.size();
	for (int i = 0; i < gisRect.RoadPoints.size(); i++) {
	
		CvPoint pt_3000 = convENUto3000(gisRect.RoadPoints[i].x, gisRect.RoadPoints[i].y,
							XX_used, YY_used, ZZ_used);
		
		Gis_Lane[i].x = pt_3000.x;
		Gis_Lane[i].y = pt_3000.y;
	}*/
	
	// ****** generate a straight gis line manually for debug ****** //
	/*for (int i = 0; i < gisRect.RoadPoints.size(); i++) {
	
		Gis_Lane[i].x = 2.75 * LOCAL_COOR_WIDTH / 5;
		Gis_Lane[i].y = i * ((2 * LOCAL_COOR_LENGTH-1) / (gisRect.RoadPoints.size()-1));
	}*/
	
	if(match_state == 3)
	{
		end_dist = cross_dist;
		cross_dist = 1000.0;
		endlinedistance = end_dist * 50.0;
		crossdistance = cross_dist * 50.0;
	}
	else
	{
		end_dist = 1000.0;
		endlinedistance = end_dist * 50.0;
	}
	
	cross_state = gisRect.CrossRoadType;
	
	if(cross_state<3)
	{
		islight = 0;
		if(cross_state==0)
			Gis_TurnDirection = 0;
		else if(cross_state==1)
			Gis_TurnDirection = 1;
		else
			Gis_TurnDirection = 2;
	}
	else
	{
		islight = 1;
		if(cross_state==3)
			Gis_TurnDirection = 0;
		else if(cross_state==4)
			Gis_TurnDirection = 1;
		else
			Gis_TurnDirection = 2;
	}
	
	if (Gis_TurnDirection == 0 && crossdistance < 2500) {
		dec_msg.TurnLight = dec_msg.TURN_LIGHT_NONE;
	} else if (Gis_TurnDirection == 1 && crossdistance < 2500) {
		dec_msg.TurnLight = dec_msg.TURN_LIGHT_LEFT;
	} else if (Gis_TurnDirection == 2 && crossdistance < 2500) {
		dec_msg.TurnLight = dec_msg.TURN_LIGHT_RIGHT;
	} else {
		dec_msg.TurnLight = dec_msg.TURN_LIGHT_NONE;
	}
	
	// transfer is_in_fee
	if (gisRect.LaneChangeFlag == 5) is_in_fee = true;
	else                             is_in_fee = false;
	
	
	// TODO: transfer gis_*_para_width
}

//INS CALLBACK
void transferINS(const in2_msgs::InsInfo::ConstPtr &ins_msg)
{
	ins = *ins_msg;
	XX = ins.position.x;
	YY = ins.position.y;
	ZZ = ins.attitude.z;
	//cout << XX << endl;
	//cout << YY << endl;
	ins_recv = true;
	Timer_ins = ins.sendtime;
	//C_InsFrame temp(ins_msg->sendtime,ins_msg->second,ins_msg->longitude,ins_msg->latitude,ins_msg->position,ins_msg->attitude,ins_msg->velocity);
	//long LastInsInfo_TS = InsFrameHis.addInsFrameHis(temp);
	
	/*XX_used = XX;
	YY_used = YY;
	ZZ_used = ZZ;
	
	for (int i = 0; i < gisRect.RoadPoints.size(); i++) {
	
		CvPoint pt_3000 = convENUto3000(gisRect.RoadPoints[i].x, gisRect.RoadPoints[i].y,
							XX_used, YY_used, ZZ_used);
		
		Gis_Lane[i].x = pt_3000.x;
		Gis_Lane[i].y = pt_3000.y;
	}*/
	
	/*CvPoint2D64f roadpts_Sent_dbl[1000];
	for (int i = 0; i < 301; i++) {
		doRT2(roadpts_Sent[i].x, roadpts_Sent[i].y, roadpts_Sent_dbl[i].x, roadpts_Sent_dbl[i].y,
		      XX_last,YY_last,XX,YY,ZZ_last,ZZ);
	}
	for (int i = 0; i < 301; i++) {
		roadpts_Sent[i].x = roadpts_Sent_dbl[i].x;
		roadpts_Sent[i].y = roadpts_Sent_dbl[i].y;
	}*/
	
	// TODO: transfer is_use_stop_line
}

//GIS CALLBACK
void transferGIS(const in2_msgs::UdpGeneral::ConstPtr &GIS_msg)
{
	gis = *GIS_msg;
	gis_received = true;
	speed_gis = gis.data[0x13];
	if(speed_gis>speed_gisUpLine)
		speed_gis = speed_gisUpLine;
	drive_state = gis.data[0x12];
	int bit[8];
	
	match_state = ((drive_state&(0x0000000f<<6*4))>>6*4)&(0x0f);
	//fprintf(stdout,"match_state: %d\n",match_state);
	 
	cross_state = ((drive_state&(0x0f<<5*4))>>5*4)&(0x0f);
	//fprintf(stderr,"cross_state: %d\n",cross_state);
	
	bit[2] = ((drive_state&(0x0f<<4*4))>>4*4)&(0x0f);
	bit[3] = ((drive_state&(0x0f<<3*4))>>3*4)&(0x0f);
	bit[4] = ((drive_state&(0x0f<<2*4))>>2*4)&(0x0f);
	
	int bit2[8];
	bit2[0] = ((gis.data[0x10]&(0x0f<<7*4))>>7*4)&(0x0f);
	bit2[1] = ((gis.data[0x10]&(0x0f<<6*4))>>6*4)&(0x0f);
	bit2[2] = ((gis.data[0x10]&(0x0f<<5*4))>>5*4)&(0x0f);
	bit2[3] = ((gis.data[0x10]&(0x0f<<4*4))>>4*4)&(0x0f);

	bit2[4] = ((gis.data[0x10]&(0x0f<<3*4))>>3*4)&(0x0f);
	bit2[5] = ((gis.data[0x10]&(0x0f<<2*4))>>2*4)&(0x0f);
	bit2[6] = ((gis.data[0x10]&(0x0f<<1*4))>>1*4)&(0x0f);
	bit2[7] = ((gis.data[0x10]&(0x0f<<0*4))>>0*4)&(0x0f);
	
	//cross_dist = (double)((bit[2]<<4*2)+(bit[3]<<4*1)+(bit[4]<<4*0))/10.0;
	//int dist = (bit[2]<<4*2)+(bit[3]<<4*1)+(bit[4]<<4*0);
	int dist = ((bit[2]<<4*2)&(0x0f<<4*2))|((bit[3]<<4*1)&(0x0f<<4*1))|((bit[4]<<4*0)&(0x0f<<4*0));
	if(dist <= 2047)
		cross_dist = double(dist)/10.0;
	else
		cross_dist = double(dist-4095)/10.0;
	if(Gis_TurnDirection == 0)
	{
		if(cross_dist<200.0)
			cross_dist = cross_dist - 7.0;
	}
	dev_angle = gis.data[0x11]/1000.0 * PI/180.0;
	Gis_Lanenum = (bit2[0]<<4*3)+(bit2[1]<<4*2)+(bit2[2]<<4*1)+(bit2[3]<<4*0);
	Gis_SelectLanenum = (bit2[4]<<4*3)+(bit2[5]<<4*2)+(bit2[6]<<4*1)+(bit2[7]<<0);
	//fprintf(stderr,"Gis_Lanenum = %d\n",Gis_Lanenum);
	//if(cross_dist<45.0&&cross_dist>0.0)
		//Gis_SelectLanenum = 2;
	//else
	//	Gis_SelectLanenum = 255;
	tfsn_flag = ((drive_state&0x000000f0)>>4)&0x0000000f; 
	specialnum = ( (drive_state&0xf0000000)>>28 )&0x0000000f;
	switch(specialnum)
	{
		case 0:
			is_shinroad = 0;
			overpassflag = 0;
		break;
		case 1:
			is_shinroad = 1;
			overpassflag = 0;
		break;
		case 2:
			is_shinroad = 2;
			overpassflag = 0;
		break;
		case 3:
			is_shinroad = 0;
			overpassflag = 1;
		break;
		case 4:
			is_shinroad = 1;
			overpassflag = 1;
		break;
		case 5:
			is_shinroad = 2;
			overpassflag = 1;
		break;
	}
	//fprintf(stderr,"cis_shinroad:%d\n",is_shinroad);
			
	if(match_state == 3)
	{
		end_dist = cross_dist;
		cross_dist = 1000.0;
	}
	else
	{
		end_dist = 1000.0;
	}

	if(cross_state<3)
	{
		islight = 0;
		if(cross_state==0)
			Gis_TurnDirection = 0;
		else if(cross_state==1)
			Gis_TurnDirection = 1;
		else
			Gis_TurnDirection = 2;
	}
	else
	{
		islight = 1;
		if(cross_state==3)
			Gis_TurnDirection = 0;
		else if(cross_state==4)
			Gis_TurnDirection = 1;
		else
			Gis_TurnDirection = 2;
	}
	if(cross_dist<40.0&&cross_dist>0.0)
		TFSNTFLchange = 2;	
	else
		TFSNTFLchange = 0;

	for(int i=0;i<301;i++)
	{
		Gis_LaneBefore[i].x = gis.x[i];
		Gis_LaneBefore[i].y = gis.y[i];
	}

	if(tfsn_flag==2)	
		parkingcontrolnum = 1;
	if(tfsn_flag==1)
		speed_gis = tfsnspeed;
		
}

//TREK CALLBACK
void transferTREK(const in2_msgs::TcpGeneral::ConstPtr &TREK_msg)
{
	trek = *TREK_msg;
	trek_received = true;
	Cur_EV_Odometer = (double)trek.data[0x01];//cm
	Cur_EV_Speed = trek.data[0x02]/1000.0; //µ±Ç°³µËÙ£¬µ¥Î»km/h
	Cur_EV_Orient = -(trek.data[0x06]/1000.0 - 50.0)/50.0*40.0*PI/180.0; //0-50000-speed_slow0 µ±Ç°ÐÐÊ»·œÏò£¬µ¥Î»»¡¶È£¬·œÏò¶šÒå£º³µÕýÇ°·œÎª0£¬×óÎªÕý£¬ÓÒÎªžº
	if(trek_init_flag)
	{
		trek_prev = *TREK_msg;
		trek_init_flag = false;
	}
	//fprintf(stdout,"Cur_EV_Speed: %f\n",Cur_EV_Speed);
	double speedtemp;
	speedtemp = Cur_EV_Speed;
	if(speedtemp>100.0)
		speedtemp = 100.0;
	if(speedtemp<2.0)
		speedtemp = 2.0;
	
	lanechangespeed = (int)((100.0-speedtemp)/(100.0-2.0)*((double)lanechangespeedMax-(double)lanechangespeedMin) + (double)lanechangespeedMin);
        lanechangespeed2 = (int)((100.0-speedtemp)/(100.0-2.0)*((double)lanechangespeed2Max-(double)lanechangespeed2Min) + (double)lanechangespeed2Min);

	DistancecostMax = (speedtemp - 2.0)*(speedtemp - 2.0)/((100.0-2.0)*(100.0-2.0))*(DistancecostMax_Max-DistancecostMax_Min) + DistancecostMax_Min;	
	DistancecostMax2 = (speedtemp - 2.0)*(speedtemp - 2.0)/((100.0-2.0)*(100.0-2.0))*(DistancecostMax2_Max-DistancecostMax2_Min) + DistancecostMax2_Min;	

	curlanedischage = (100.0-speedtemp)/(100.0-2.0)*(curlanedischageMax-curlanedischageMin) + curlanedischageMin;

	linecostdistance = (100.0-speedtemp)/(100.0-2.0)*(linecostdistanceMax-linecostdistanceMin) + linecostdistanceMin;
	
	if(lanechangespeed>lanechangespeedMax)
		lanechangespeed = lanechangespeedMax;
	if(lanechangespeed<lanechangespeedMin)
		lanechangespeed = lanechangespeedMin;
	if(lanechangespeed2>lanechangespeed2Max)
		lanechangespeed2 = lanechangespeed2Max;
	if(lanechangespeed2<lanechangespeed2Min)
		lanechangespeed2 = lanechangespeed2Min;

	if(DistancecostMax>DistancecostMax_Max)
		DistancecostMax = DistancecostMax_Max;
	if(DistancecostMax<DistancecostMax_Min)
		DistancecostMax = DistancecostMax_Min;
	if(DistancecostMax2>DistancecostMax2_Max)
		DistancecostMax2 = DistancecostMax2_Max;
	if(DistancecostMax2<DistancecostMax2_Min)
		DistancecostMax2 = DistancecostMax2_Min;
	if(curlanedischage>curlanedischageMax)
		curlanedischage = curlanedischageMax;
	if(curlanedischage<curlanedischageMin)
		curlanedischage = curlanedischageMin;
	if(linecostdistance>linecostdistanceMax)
		linecostdistance = linecostdistanceMax;
	if(linecostdistance<linecostdistanceMin)
		linecostdistance = linecostdistanceMin;

}

//RoadEdgeLINES CALLBACK
void transferRoadEdge(const in2_msgs::RoadEdge::ConstPtr &roadedge_msg)
{
	roadedge = *roadedge_msg;
	edge_width = roadedge.leftdis + roadedge.rightdis;
	
	cur_angle = roadedge.Angle;
	
	cur_angle = cur_angle * (1.0 - old_angle_coe) + last_angle * (old_angle_coe);
	
	last_angle = cur_angle;
	
	roadedge.Angle = cur_angle;
}

//ROADLINES CALLBACK
void transferLaneModelInfo(const in2_msgs::LaneInfoV2::ConstPtr &laneinfo_msg)
{
	laneModelInfo = *laneinfo_msg;
	Timer_lanemodel = laneModelInfo.sendtime;
	lanemodelinfo_recv = true;
}

void transferControlPanel(const in2_msgs::UdpGeneralShort::ConstPtr &controlpanel_msg)
{
	ControlPanelInfo = *controlpanel_msg;
	controlpanel_recv = true;
	controlpanel_info = ControlPanelInfo.data[1];
}

/****************************************************************************************/
bool Issamecluster()
{
	int selectnum,nearnum;
	if((SelectPathIndex<trafficlanenum)||(NearcarLaneIndex<trafficlanenum))
	{
		if((SelectPathIndex<trafficlanenum)&&(NearcarLaneIndex>=trafficlanenum))
		{
			selectnum = SelectPathIndex;
			nearnum = (NearcarLaneIndex-trafficlanenum)/7;
		}
			else if((SelectPathIndex>=trafficlanenum)&&(NearcarLaneIndex<trafficlanenum))
			{
				nearnum = NearcarLaneIndex;
				selectnum = (SelectPathIndex-trafficlanenum)/7;
			}	
			else
			{
				selectnum = SelectPathIndex;
				nearnum = NearcarLaneIndex;
			}
	}
	else
	{	
		selectnum = (SelectPathIndex-trafficlanenum)/7;
		nearnum = (NearcarLaneIndex-trafficlanenum)/7;
	}		
	if(selectnum==nearnum)
	{
		return(true);
	}
	else
	{
		return(false);
	}
}
	
void checkDriveMode()    
{
	double dis_average;
	int NearCarIndex2 = 0;
	double distance2;
	double distance_min2 = 100000000.0;
	long GisPointNum = 0;
	bool PointCheck = true;
	double A,B,C;	
	double distanceSum=0.0;	
	long sumnum = 0;
	CvPoint *GisSelectPoints = new CvPoint[1000];
	double *distance_group = new double[1000]; 

	for(int i=0;i<301;i++)
	{
		distance2= sqrt( (1500.0-(double)Gis_Lane[i].x)*(1500.0-(double)Gis_Lane[i].x) + 
				(1000.0-(double)Gis_Lane[i].y)*(1000.0-(double)Gis_Lane[i].y)
			      );
		if(distance2 < distance_min2)
		{
			NearCarIndex2 = i;
			distance_min2 = distance2;
		}
	}

	for(int i=NearCarIndex2;i<301;i++) //Remove the same points in GIS points group
	{
		for(int j=0;j<GisPointNum;j++)
		{
			if((Gis_Lane[i].x==GisSelectPoints[j].x)&&(Gis_Lane[i].y==GisSelectPoints[j].y))
			{
				PointCheck = false;
				break;
			}
			else
				PointCheck = true;
		}
		if(PointCheck)
		{
			GisSelectPoints[GisPointNum++]=Gis_Lane[i];
		}
	}//select gis points and store in GisSelectPoints[]

	A=traffic_lanePro[pathwaynum-1][300].y-traffic_lanePro[pathwaynum-1][0].y;
	B=traffic_lanePro[pathwaynum-1][0].x-traffic_lanePro[pathwaynum-1][300].x;
	C=traffic_lanePro[pathwaynum-1][300].x*traffic_lanePro[pathwaynum-1][0].y-traffic_lanePro[pathwaynum-1][0].x*traffic_lanePro[pathwaynum-1][300].y;
	
	long numtemp;
	if(GisPointNum>Switch_Prospect)
		GisPointNum=Switch_Prospect;//only see 24m
	if(GisPointNum<2)
	{
		variance = 0.0;//lane
	}	
	else	
	{	
		for(int i=0;i<GisPointNum;i++)
		{
			double x0 =  GisSelectPoints[i].x;
			double y0 =  GisSelectPoints[i].y;
			distance_group[i]= abs(A*x0+B*y0+C)/sqrt(A*A + B*B);
			//fprintf(stdout,"distance_group=%f\r\n",distance_group[i]);
			numtemp = GisPointNum-i;
			distanceSum += distance_group[i] * (double)numtemp;
			sumnum += numtemp;
		}
	
		dis_average = distanceSum / (double)sumnum ; 
		
		double sumtemp=0.0;
		sumnum = 0;
		for(int i=0;i<GisPointNum;i++)
		{
			numtemp = GisPointNum-i;
			sumtemp += (distance_group[i]-dis_average)*(distance_group[i]-dis_average) * (double)numtemp;
			sumnum += numtemp;				
		}
		
		variance = sumtemp / (double)sumnum ; 
	}

	variance = variance*0.5+variancelast*0.5;
	//fprintf(stdout,"variance = %f \r\n", variance);
	variancelast = variance;

	switch(DriveMode)
	{
		case 0:
			if(variance>variance_threshold2)
			{
				DriveMode = 1;//gis
			}
			break;
		case 1:
			if(variance<variance_threshold1)
			{
				DriveMode = 0;//lane
			}
			break;
	}

	if(crossdistance<1000.0)// distance from cross is 22m or in cross
	{
		strategemode = 3;
		speed_corner = 80000;
		/*if(crossdistance<300.0)
			strategemode = 3; //gis
		else
		{
			if(DriveMode==0)
			{
				if(trafficlanenum>1)
					strategemode = 1;//lane
				else
				{
					if(distance_min2<100.0)	
						strategemode = 8;//gis dan xian
					else
						strategemode = 3;
				}
			}
			else
			{
				strategemode = 3;
			}
		}*/
	}
	else
	{
		if(DriveMode==0)
		{
			if(trafficlanenum>1)
				strategemode = 1;//lane
			else
			{
				strategemode = 3;
				speed_corner = 80000;					
			}
		}
		else
		{
			strategemode = 3;
			speed_corner = 80000;
		}
	}
	if(forcestratege==1)//gis+avoiding
	{
		strategemode = 5;	
	}
	if(forcestratege==2)//roadedge 
	{
		strategemode = 2;
	}
	//fprintf(stderr,"Ins_Lanedistime = %f\n",abs(Timer_lanemodel-Timer_ins));
	if(abs(Timer_lanemodel-Timer_ins)>INS_Lanedistime)
	{
		strategemode = 3;
		//fprintf(stderr,"LaneModelBoom!\n");
		speed_corner = 80000;
	}
	
	//strategemode = 3;
	//fprintf(stdout,"DriveMode=%d\r\n",DriveMode);
	delete[] GisSelectPoints;
	delete[] distance_group;
}

CvPoint conv3000ToMap(CvPoint src_pt, int MapSize)
{
	int x = (1.0f * src_pt.x) / 3000 * MapSize;
	int y = MapSize - (1.0f * src_pt.y) / 3000 * MapSize;
	
	return ( cvPoint(x,y) );
}

double getLineCost(CvPoint* pts, int start)
{
	double costtemp = 0.0;
	CvPoint *CaculatePoints = new CvPoint[1000];
	long CacPointNum = 0;
	long sumnum = 0; 
	long numtemp = 0;
	bool PointCheck = true;
	double costsum = 0.0;
	
	for(int i=start;i<301;i++) //Remove the same points in caculate points group
	{
		for(int j=0;j<CacPointNum;j++)
		{
			if((pts[i].x==CaculatePoints[j].x)&&(pts[i].y==CaculatePoints[j].y))
			{
				PointCheck = false;
				break;
			}
			else
				PointCheck = true;
		}
		if(PointCheck)
		{
			CaculatePoints[CacPointNum++]=pts[i];
		}
	}//select caculate points and store in CaculatePoints[]

	for(int i=0;i<CacPointNum;i++)
	{
		numtemp = linecostdistance * (CacPointNum-i)*(CacPointNum-i);
		
		//double coe = CacPointNum-i;
		//if (coe < 75) coe = 75;
		//numtemp = linecostdistance * coe;
		
		if(CaculatePoints[i].y<1) {
			CaculatePoints[i].y = 1;
			break;
		}
		if(CaculatePoints[i].y>2999) {
			CaculatePoints[i].y = 2999;
			break;
		}
		if(CaculatePoints[i].x<1) {
			CaculatePoints[i].x = 1;
			break;
		}
		if(CaculatePoints[i].x>2999) {
			CaculatePoints[i].x = 2999;
			break;
		}

		double costpxel = (double)cvGet2D(CostMap,(3000-CaculatePoints[i].y),CaculatePoints[i].x).val[0];	
		costsum += costpxel * (double)numtemp;
		sumnum += numtemp;
	}
	costtemp = costsum / (double)sumnum;

	delete[] CaculatePoints;
	return(costtemp);
}

double* CaculateRulecost() 
{
	double* Rulecosttemp = new double[pathwaynum];
	int detectlanenum;
	int centernum;
	int tempnum;
	int sumnumtemp;
	detectlanenum = trafficlanenum-1;
	
	if(is_shinroad==0)
	{
		if(detectlanenum>0)//0
		{
			centernum = (trafficlanenum+1)/2-1;
			if(detectlanenum<Gis_Lanenum)//1
			{
				for(int i=0;i<trafficlanenum;i++)
				{
					if( i==0||i==(trafficlanenum-1) )
						Rulecosttemp[i] = 200.0;
					else
					{
						if(i>=centernum)//right half' cost is lower
							Rulecosttemp[i] = 140.0 * ((double)(i-centernum)+1.0) / (double)(trafficlanenum-centernum);
						else//left half' cost is higher
							Rulecosttemp[i] = 240.0 * ((double)(centernum-i)+1.0) / (double)(trafficlanenum-centernum);
					}
				} 
				for(int i=trafficlanenum; i<(pathwaynum-1);i++)
				{
					int temp = (i-trafficlanenum)%7;
					int temp2 = (i-trafficlanenum)/7;
					switch(temp)
					{
						case 0:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/7.0;
						break;
						case 1:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 2:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 3:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *1.0/7.0;
						break;
						case 4:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 5:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 6:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/4.0;
						break;	
					}
				}
				Rulecosttemp[pathwaynum-1] = 10000000000.0;		
			}
			else if(detectlanenum>Gis_Lanenum)//1 regard right as min cost value
			{
				for(int i=0;i<trafficlanenum;i++)
				{
					if( i==0||i==(trafficlanenum-1) )
						Rulecosttemp[i] = 200.0;
					else
						Rulecosttemp[i] = 200.0 * (double)(trafficlanenum-i) / (double)trafficlanenum;
				} 
				for(int i=trafficlanenum; i<(pathwaynum-1);i++)
				{
					int temp = (i-trafficlanenum)%7;
					int temp2 = (i-trafficlanenum)/7;
					switch(temp)
					{
						case 0:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/4.0;
						break;
						case 1:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *6.0/8.0;
						break;
						case 2:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/8.0;
							break;
						case 3:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/8.0;
						break;
						case 4:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/8.0;
						break;
						case 5:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *6.0/8.0;
						break;
						case 6:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/4.0;
						break;	
					}
				}
				Rulecosttemp[pathwaynum-1] = 10000000000.0;	
			}	
			else//1
			{
				for(int i=0;i<trafficlanenum;i++)
				{
					if( i==0||i==(trafficlanenum-1) )
						Rulecosttemp[i] = 200.0;
					else
					{
						if(i>=centernum)//right half' cost is lower
							Rulecosttemp[i] = 140.0 * ((double)(i-centernum)+1.0) / (double)(trafficlanenum-centernum);
						else//left half' cost is higher
							Rulecosttemp[i] = 240.0 * ((double)(centernum-i)+1.0) / (double)(trafficlanenum-centernum);
					}
				} 
				for(int i=trafficlanenum; i<(pathwaynum-1);i++)
				{
					int temp = (i-trafficlanenum)%7;
					int temp2 = (i-trafficlanenum)/7;
					switch(temp)
					{
						case 0:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/7.0;
						break;
						case 1:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 2:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 3:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *1.0/7.0;
						break;
						case 4:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 5:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 6:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/7.0;
						break;	
					}
				}
				Rulecosttemp[pathwaynum-1] = 10000000000.0;
			}
		}
		else
		{
			if(trafficlanenum==1)
			{
				Rulecosttemp[0] = 200.0;
				Rulecosttemp[1] = 10.0;
			}
			else
			{
				Rulecosttemp[0] = 10.0;
			}
		}
	}
	else if(is_shinroad==1)
	{
		if(detectlanenum>0)//0
		{
			centernum = Gis_SelectLanenum-1;
			if((trafficlanenum-centernum) > centernum)
				sumnumtemp = trafficlanenum-centernum;
			else
				sumnumtemp = centernum;
			if(detectlanenum<Gis_Lanenum)//1
			{
				for(int i=0;i<trafficlanenum;i++)
				{
					if( i==0||i==(trafficlanenum-1) )
						Rulecosttemp[i] = 200.0;
					else
					{
						if(i>=centernum)//right half' cost is lower
							Rulecosttemp[i] = 140.0 * ((double)(i-centernum)+1.0) / ((double)sumnumtemp);
						else//left half' cost is higher
							Rulecosttemp[i] = 240.0 * ((double)(centernum-i)+1.0) / ((double)sumnumtemp);
					}
				} 
				for(int i=trafficlanenum; i<(pathwaynum-1);i++)
				{
					int temp = (i-trafficlanenum)%7;
					int temp2 = (i-trafficlanenum)/7;
					switch(temp)
					{
						case 0:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/7.0;
						break;
						case 1:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 2:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 3:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *1.0/7.0;
						break;
						case 4:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 5:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 6:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/4.0;
						break;	
					}
				}
				Rulecosttemp[pathwaynum-1] = 10000000000.0;		
			}
			else if(detectlanenum>Gis_Lanenum)//1 regard right as min cost value
			{
				for(int i=0;i<trafficlanenum;i++)
				{
					if( i==0||i==(trafficlanenum-1) )
						Rulecosttemp[i] = 200.0;
					else
						Rulecosttemp[i] = 200.0 * (double)(trafficlanenum-i) / (double)trafficlanenum;
				} 
				for(int i=trafficlanenum; i<(pathwaynum-1);i++)
				{
					int temp = (i-trafficlanenum)%7;
					int temp2 = (i-trafficlanenum)/7;
					switch(temp)
					{
						case 0:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/4.0;
						break;
						case 1:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *6.0/8.0;
						break;
						case 2:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/8.0;
							break;
						case 3:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/8.0;
						break;
						case 4:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/8.0;
						break;
						case 5:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *6.0/8.0;
						break;
						case 6:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/4.0;
						break;	
					}
				}
				Rulecosttemp[pathwaynum-1] = 10000000000.0;	
			}	
			else//1
			{
				for(int i=0;i<trafficlanenum;i++)
				{
					if( i==0||i==(trafficlanenum-1) )
						Rulecosttemp[i] = 200.0;
					else
					{
						if(i>=centernum)//right half' cost is lower
							Rulecosttemp[i] = 140.0 * ((double)(i-centernum)+1.0) / ((double)sumnumtemp);
						else//left half' cost is higher
							Rulecosttemp[i] = 240.0 * ((double)(centernum-i)+1.0) / ((double)sumnumtemp);
					}
				} 
				for(int i=trafficlanenum; i<(pathwaynum-1);i++)
				{
					int temp = (i-trafficlanenum)%7;
					int temp2 = (i-trafficlanenum)/7;
					switch(temp)
					{
						case 0:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/7.0;
						break;
						case 1:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 2:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 3:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *1.0/7.0;
						break;
						case 4:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *3.0/7.0;
						break;
						case 5:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/7.0;
						break;
						case 6:
							Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *7.0/7.0;
						break;	
					}
				}
				Rulecosttemp[pathwaynum-1] = 10000000000.0;
			}
		}
		else
		{
			if(trafficlanenum==1)
			{
				Rulecosttemp[0] = 200.0;
				Rulecosttemp[1] = 10.0;
			}
			else
			{
				Rulecosttemp[0] = 10.0;
			}
		}
	}
	else
	{
		if(detectlanenum>0)//0
		{
			for(int i=0;i<trafficlanenum;i++)
			{
				if( i==0||i==(trafficlanenum-1) )
					Rulecosttemp[i] = 200.0;
				else
					Rulecosttemp[i] = 200.0 * (double)(trafficlanenum-i) / (double)trafficlanenum;
			} 
			for(int i=trafficlanenum; i<(pathwaynum-1);i++)
			{
				int temp = (i-trafficlanenum)%7;
				int temp2 = (i-trafficlanenum)/7;
				switch(temp)
				{
					case 0:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/4.0;
					break;
					case 1:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *6.0/8.0;
					break;
					case 2:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/8.0;
					break;
					case 3:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/8.0;
					break;
					case 4:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *5.0/8.0;
					break;
					case 5:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *6.0/8.0;
					break;
					case 6:
						Rulecosttemp[i] =  CenterReducePer * Rulecosttemp[temp2] *4.0/4.0;
					break;	
				}
			}
			Rulecosttemp[pathwaynum-1] = 10000000000.0;
		}
		else
		{
			if(trafficlanenum==1)
			{
				Rulecosttemp[0] = 200.0;
				Rulecosttemp[1] = 10.0;
			}
			else
			{
				Rulecosttemp[0] = 10.0;
			}
		}
	}
	return(Rulecosttemp);
}

double CaculateObstaclecost1(CvPoint* pathway_ori,double &neardistance)
{		
	double cost;
	double distancetemp;
	double distancemintemp = 10000000000000000.0;
	int nearcarindextemp;	
	CvPoint *pathway = new CvPoint[301];
	for (int i = 0; i < 301; i++) {
		pathway[i] = pathway_ori[i];
	}

	for(int i=0;i<301;i++)
	{
		distancetemp = ((double)pathway[i].x-1500.0)*((double)pathway[i].x-1500.0)+((double)pathway[i].y-1200.0)*((double)pathway[i].y-1200.0);
		if(distancetemp<distancemintemp)
		{
			distancemintemp = distancetemp;
			nearcarindextemp = i;
		}
	}
	neardistance = distancemintemp;

	if(pathway[nearcarindextemp].x>=1&&pathway[nearcarindextemp].x<=3000&&pathway[nearcarindextemp].y>=1&&pathway[nearcarindextemp].y<=3000)
		cvCircle(PlanMapShow,cvPoint(pathway[nearcarindextemp].x,3000-pathway[nearcarindextemp].y),3,CV_RGB(255,0,0),3);	

	double distancefront = 0.0;
	for(int i=(nearcarindextemp+1);i<301;i++) //check whether the lane is block
	{				
		//printf("%d,%d\n",pathway[i].x,3000-pathway[i].y);

		if(pathway[i].y<1) {
			pathway[i].y = 1;
			break;
		}
		if(pathway[i].y>2999) {
			pathway[i].y = 2999;
			break;
		}
		if(pathway[i].x<1) {
			pathway[i].x = 1;
			break;
		}
		if(pathway[i].x>2999) {
			pathway[i].x = 2999;
			break;
		}	

		distancefront += sqrt( ((double)pathway[i].x-(double)pathway[i-1].x)*((double)pathway[i].x-(double)pathway[i-1].x)+((double)pathway[i].y-(double)pathway[i-1].y)*((double)pathway[i].y-(double)pathway[i-1].y) );	
		
		if( cvGet2D(CostMap,(3000-pathway[i].y),pathway[i].x).val[0] > 160)
		//if( cvGet2D(CostMap,(3000-pathway[i].y),pathway[i].x).val[0] > 250)
		{			
			if(distancefront<220.0)//4.5m
			//if(distancefront<2000.0)
			{
				return (10000000000.0);
			}
			else
			{
				break;
			}
		}	
	}

	cost = getLineCost(pathway,nearcarindextemp);	
	
	delete[] pathway;
	
	return(cost);
}

double CaculateObstaclecost2(CvPoint* pathway_ori,double &neardistance)
{	
	double cost;
	double distancetemp;
	double distancemintemp = 10000000000000000.0;
	int nearcarindextemp;
	int nearcarindextemp2;	
	double distancetemp2;
	CvPoint *pathway = new CvPoint[301];
	for (int i = 0; i < 301; i++) {
		pathway[i] = pathway_ori[i];
	}

	for(int i=0;i<301;i++)
	{
		distancetemp = ((double)pathway[i].x-1500.0)*((double)pathway[i].x-1500.0)+((double)pathway[i].y-1200.0)*((double)pathway[i].y-1200.0);
		if(distancetemp<distancemintemp)
		{
			distancemintemp = distancetemp;
			nearcarindextemp = i;
		}
	}
	neardistance = distancemintemp;

	for(int i=nearcarindextemp;i<301;i++)//find the start point
	{
		distancetemp2 = ((double)pathway[i].x-(double)pathway[nearcarindextemp].x)*((double)pathway[i].x-(double)pathway[nearcarindextemp].x)+((double)pathway[i].y-(double)pathway[nearcarindextemp].y)*((double)pathway[i].y-(double)pathway[nearcarindextemp].y);

		nearcarindextemp2 = i;
		if(distancetemp2>=(frontoffset*distancemintemp))
		{
			break;
		}		
	}
	
	if(nearcarindextemp2>=299)
	{
		return(10000000000.0);		
	}
	
	if(pathway[nearcarindextemp2].x>=1&&pathway[nearcarindextemp2].x<=3000&&pathway[nearcarindextemp2].y>=1&&pathway[nearcarindextemp2].y<=3000)
		cvCircle(PlanMapShow,cvPoint(pathway[nearcarindextemp2].x,3000-pathway[nearcarindextemp2].y),3,CV_RGB(255,0,0),3);
	
	double distancefront = 0.0;
	for(int i=(nearcarindextemp2+1);i<301;i++) //check whether the lane is block
	{				
		//printf("%d,%d\n",pathway[i].x,3000-pathway[i].y);
		if(pathway[i].y<1) {
			pathway[i].y = 1;
			break;
		}
		if(pathway[i].y>2999) {
			pathway[i].y = 2999;
			break;
		}
		if(pathway[i].x<1) {
			pathway[i].x = 1;
			break;
		}
		if(pathway[i].x>2999) {
			pathway[i].x = 2999;
			break;
		}	

		distancefront += sqrt( ((double)pathway[i].x-(double)pathway[i-1].x)*((double)pathway[i].x-(double)pathway[i-1].x)+((double)pathway[i].y-(double)pathway[i-1].y)*((double)pathway[i].y-(double)pathway[i-1].y) );	
	
		if( cvGet2D(CostMap,(3000-pathway[i].y),pathway[i].x).val[0] > 160)
		//if( cvGet2D(CostMap,(3000-pathway[i].y),pathway[i].x).val[0] > 250)
		{			
			if (distancefront<200) // 4m
			//if(distancefront<2000.0)//40m
			{
				return (10000000000.0);
			}
			else
			{
				break;
			}
		}	
	}

	cost = getLineCost(pathway,nearcarindextemp2);	
	
	delete[] pathway;
	
	return(cost);	
}

void SelectNearestWay()
{
	double distance1;
	double distancemin1;
	double distancemin2 = 10000000000000000.0;

	for(int i=0; i<pathwaynum;i++)
	{
		distancemin1 = 10000000000000000.0;
		for(int j=0; j<301;j++)
		{
			distance1 = ((double)traffic_lanePro[i][j].x - 1500.0)*((double)traffic_lanePro[i][j].x - 1500.0)+((double)traffic_lanePro[i][j].y - 1000.0)*((double)traffic_lanePro[i][j].y - 1000.0);
			if(distance1<distancemin1)
			{
				distancemin1 = distance1;
			}
		}
		if(distancemin1<distancemin2)
		{
			distancemin2 = distancemin1;
			NearcarLaneIndex = i;
		}
	}
	
	for(int i=0; i<301;i++)
	{
		Select_nearestlane[i] = traffic_lanePro[NearcarLaneIndex][i];
	} //Select the nearest lane to the car and index is NearcarLaneIndex	
}

void SelectNearestWay2()
{
	double distance1;
	double distancemin1;
	double distancemin2 = 10000000000000000.0;

	for(int i=planst; i<planed;i++)
	{
		distancemin1 = 10000000000000000.0;
		for(int j=0; j<301;j++)
		{
			distance1 = ((double)plines[i][j].x - 1500.0)*((double)plines[i][j].x - 1500.0)+((double)plines[i][j].y - 1000.0)*((double)plines[i][j].y - 1000.0);
			if(distance1<distancemin1)
			{
				distancemin1 = distance1;
			}
		}
		if(distancemin1<distancemin2)
		{
			distancemin2 = distancemin1;
			NearPlanLaneIndex = i;
		}
	}

	for(int i=0; i<301;i++)
	{
		Plan_nearestlane[i] = plines[NearPlanLaneIndex][i];
	} //Select the nearest lane to the car and index is NearcarLaneIndex	
}

void SelectBestWay()
{
	double* Rulecost;
	double* Obstaclecost = new double[pathwaynum];
	double* Rankdistance = new double[pathwaynum];
	double* Distancecost = new double[pathwaynum];
	double* Lanecost = new double[pathwaynum];
	int blocknum=0;
	double neardistance;
	double distancemax=-0.1,distancemin=10000000000.0;
	
	Rulecost = CaculateRulecost();
	
	for(int i=0;i<pathwaynum;i++)
	{
		if(i==NearcarLaneIndex)//nearcarlane's cost caculation method is not same to others
		{
			Obstaclecost[i] = CaculateObstaclecost1(traffic_lanePro[i],neardistance);
			Rankdistance[i] = neardistance;			
		}		
		else
		{
			Obstaclecost[i] = CaculateObstaclecost2(traffic_lanePro[i],neardistance);
			Rankdistance[i] = neardistance;	
		}
	}//caculate the obstacle cost for each lane
	for(int i=0;i<pathwaynum;i++)
	{
		if(Rankdistance[i]>distancemax)
			distancemax = Rankdistance[i];
		if(Rankdistance[i]<distancemin)
			distancemin = Rankdistance[i];
	}
	distancemax = sqrt(distancemax);
	distancemin = sqrt(distancemin);
	for(int i=0;i<pathwaynum;i++)
	{
		Distancecost[i] = (sqrt(Rankdistance[i])-distancemin)/(distancemax-distancemin)*DistancecostMax;
	}

	if(trafficlanenum>1)
	{
		for(int i=0;i<trafficlanenum;i++)
		{
			if(i==0)
			{
				if(Obstaclecost[i*7+trafficlanenum]>=10000000000.0)
					Obstaclecost[i] += 200.0; 
			}
			else if(i==(trafficlanenum-1))
			{
				if(Obstaclecost[(i-1)*7+trafficlanenum+6]>=10000000000.0)
					Obstaclecost[i] += 200.0; 
			}
			else
			{
				if((Obstaclecost[i*7+trafficlanenum]>=10000000000.0)||(Obstaclecost[i*7+trafficlanenum-1]>=10000000000.0))
					Obstaclecost[i] += 200.0;			
			}
		}

		for(int i=trafficlanenum;i<(pathwaynum-1);i++)
		{
			if(i==trafficlanenum)
			{
				if((Obstaclecost[0]>=10000000000.0)||(Obstaclecost[i+1]>=10000000000.0))
						Obstaclecost[i] += 200.0; 
			}
			else if(i==(pathwaynum-2))
			{
				if((Obstaclecost[trafficlanenum-1]>=10000000000.0)||(Obstaclecost[i-1]>=10000000000.0))
						Obstaclecost[i] += 200.0; 
			}
			else
			{
				if((i-trafficlanenum)%7==6)
				{
					if((Obstaclecost[(i-trafficlanenum)/7+1]>=10000000000.0)||(Obstaclecost[i-1]>=10000000000.0))
						Obstaclecost[i] += 200.0; 
				}
				else if((i-trafficlanenum)%7==0)
				{
					if((Obstaclecost[(i-trafficlanenum)/7]>=10000000000.0)||(Obstaclecost[i+1]>=10000000000.0))
						Obstaclecost[i] += 200.0; 
				}
				else
				{
					if((Obstaclecost[i-1]>=10000000000.0)||(Obstaclecost[i+1]>=10000000000.0))
						Obstaclecost[i] += 200.0; 
				}
			}
		}
	}

	if(cross_dist>50.0)
	{
		for(int i=0;i<(pathwaynum-1);i++)	
		{
			if(Obstaclecost[i]>=1000000000.0)
				blocknum++;
		}
		blockpercent = (double)blocknum / (double)pathwaynum;
		//fprintf(stdout,"blockpercent:%f\n",blockpercent);
		switch(Laneuseful)
		{
			case 0:
				if(blockpercent<blockpercent_threshold1)
				{
					Laneuseful = 1;//lane
				}
				break;
			case 1:
				if(blockpercent>blockpercent_threshold2)
				{
					Laneuseful = 0;//roadedge
				}
				break;
		}
	}
	else
	{
		Laneuseful = 1;
	}

	double costMin = 10000000000.0;//10000000000.0 is the boundary
	
	for(int i=0; i<pathwaynum;i++)
	{
		Lanecost[i] = Rulecost[i] * Rulecostweight + Obstaclecost[i] * Obstaclecostweight + Distancecost[i];
		if(i==NearcarLaneIndex)
		{
			Lanecost[i] = Lanecost[i] * curlanedischage;
		}		
		if(Lanecost[i]<costMin)
		{	
			costMin = Lanecost[i];
			SelectPathIndex = i;	
		}
		//fprintf(stderr,"1-no.%d Lanecost: %f\n",i,Lanecost[i]);
		//fprintf(stderr,"1-no.%d Rulecost: %f\n",i,Rulecost[i]);
		//fprintf(stderr,"1-no.%d Obstaclecost: %f\n",i,Obstaclecost[i]);
		//fprintf(stderr,"1-no.%d Distancecost: %f\n",i,Distancecost[i]);
	}
	if(costMin<10000000000.0)
	{
		for(int i=0;i<301;i++)
		{
			Select_bestlane[i] = traffic_lanePro[SelectPathIndex][i];
		}
	}
	else
	{
		for(int i=0;i<301;i++)
		{
			SelectPathIndex = NearcarLaneIndex;
			Select_bestlane[i] = traffic_lanePro[NearcarLaneIndex][i];
		}	
	}//if there is no bestlane, regard the nearest lane as the best lane
	
	delete[] Rulecost;
	delete[] Obstaclecost;
	delete[] Lanecost;
	delete[] Rankdistance;	
	delete[] Distancecost;
}

double calcLaneCost(CvPoint *pts) {

	double cost = 0.0;
	
	for(int i = 10; i < 290; i++) {
		if(pts[i].x >= 0 && pts[i].x < 3000 && pts[i].y >= 0 && pts[i].y < 3000 ) {
			cost += (double)cvGet2D(LaneMap, 2999 - pts[i].y, pts[i].x).val[0];
		}
	}

	return cost;
}

void SelectBestWay2()
{
	//printf("%d\n",planlanenumber);
	
	double* Rulecost = new double[planlanenumber];
	double* Obstaclecost = new double[planlanenumber];
	double* lane_cost = new double[planlanenumber];
	double* Lanecost = new double[planlanenumber];
	double* Rankdistance = new double[planlanenumber];
	double* Distancecost = new double[planlanenumber];
	double costMin = 10000000000.0;
	double neardistance;
	double distancemax=-0.1,distancemin=10000000000.0;
	int blocknum2=0;

	//for(int i=0;i<planlanenumber;i++)
	for(int i=planst;i<planed;i++)
	{
		if(i==NearPlanLaneIndex)//nearcarlane's cost caculation method is not same to others
		{
			Obstaclecost[i-planst] = CaculateObstaclecost1(plines[i],neardistance);
			Rankdistance[i-planst] = neardistance;	
		}		
		else
		{
			Obstaclecost[i-planst] = CaculateObstaclecost2(plines[i],neardistance);
			Rankdistance[i-planst] = neardistance;	
		}
	
	}//caculate the obstacle cost for each lane
	
	// lane
	//for(int i=0;i<planlanenumber;i++)
	for(int i=planst;i<planed;i++)
		lane_cost[i-planst] = calcLaneCost(plines[i]);
	
	//for(int i=0;i<planlanenumber;i++)
	for(int i=planst;i<planed;i++)
	{
		if(Rankdistance[i-planst]>distancemax)
			distancemax = Rankdistance[i-planst];
		if(Rankdistance[i-planst]<distancemin)
			distancemin = Rankdistance[i-planst];
	}
	distancemax = sqrt(distancemax);
	distancemin = sqrt(distancemin);
	
	
	//for(int i=0;i<planlanenumber;i++)
	for(int i=planst;i<planed;i++)
	{
		Distancecost[i-planst] = (sqrt(Rankdistance[i-planst])-distancemin)/(distancemax-distancemin)*DistancecostMax2;
	}
	
	if(planlanenumber>1)
	{
		//for(int i=0;i<planlanenumber;i++)
		for(int i=planst;i<planed;i++)
		{
			if(i==planst)
			{
				if(Obstaclecost[1]>=10000000000.0)
					Obstaclecost[i-planst] += 200.0;
			}
			else if(i==planed-1)
			{
				if(Obstaclecost[i-1-planst]>=10000000000.0)
					Obstaclecost[i-planst] += 200.0;
			}
			else
			{
				if((Obstaclecost[i-1-planst]>=10000000000.0)||(Obstaclecost[i+1-planst]>=10000000000.0))
					Obstaclecost[i-planst] += 200.0; 
			}
		}	
	}

	if(cross_dist>50.0)
	{
		//for(int i=0;i<planlanenumber;i++)
		for(int i=planst;i<planed;i++)
		{
			if(Obstaclecost[i-planst]>=1000000000.0)
				blocknum2++;
		}
		blockpercent2 = (double)blocknum2 / (double)planlanenumber;
		//fprintf(stdout,"blockpercent:%f\n",blockpercent);
		switch(Laneuseful2)
		{
			case 0:
				if(blockpercent2<blockpercent2_threshold1)
				{
					Laneuseful2 = 1;//lane
				}
				break;
			case 1:
				if(blockpercent2>blockpercent2_threshold2)
				{
					Laneuseful2 = 0;//roadedge
				}
				break;
		}
	}
	else
	{
		Laneuseful2 = 1;
	}

	//for(int i=0;i<planlanenumber;i++)
	for(int i=planst;i<planed;i++)
	{
		Rulecost[i-planst] = (double)abs(curlaneid - i) * 100.0 / (double)planlanenumber;
	}
	
	if (is_sent_pts_last_init) {
		SelectPlanPathIndex_last = SelectPlanPathIndex_dbl;
	} else {
		is_sent_pts_last_init = true;
		SelectPlanPathIndex_last = 250;
	}
	
	//for(int i=0;i<planlanenumber;i++)
	for(int i=planst;i<planed;i++)
	{
		Lanecost[i-planst] = Rulecost[i-planst]* Rulecostweight2 + Obstaclecost[i-planst]*Obstaclecostweight2 + Distancecost[i-planst] + lane_cost[i-planst] * lane_weight;
		//Lanecost[i-planst] = Rulecost[i-planst]* Rulecostweight2 + Obstaclecost[i-planst]*Obstaclecostweight2 + Distancecost[i-planst];
		//fprintf(stderr,"2-no.%d Lanecost: %f\n",i,Lanecost[i-planst]);
		//fprintf(stderr,"2-no.%d Rulecost: %f\n",i,Rulecost[i-planst]*Rulecostweight2);
		//fprintf(stderr,"2-no.%d Obstaclecost: %f\n",i,Obstaclecost[i-planst]*Obstaclecostweight2);
		//fprintf(stderr,"2-no.%d Distancecost: %f\n",i,Distancecost[i-planst]);	
		if(Lanecost[i-planst]<costMin)
		{	
			costMin = Lanecost[i-planst];
			SelectPlanPathIndex = i;	
		}
		//printf("3\n");
	}
	
	if (no_bing_xian) SelectPlanPathIndex = curlaneid;
	
	no_filt_SelectPlanPathIndex = SelectPlanPathIndex;
	
	pathFilter();
	
	//cout << "SelectPlanPathIndex: " << SelectPlanPathIndex << endl;
	//printf("NearPlanLaneIndex: %d\n",NearPlanLaneIndex);
	//printf("distancemin: %.3lf, distancemax: %.3lf\n", distancemin, distancemax);
	delete[] Rulecost;
	delete[] Obstaclecost;
	delete[] Lanecost;
	delete[] Rankdistance;	
	delete[] Distancecost;
}

//	crossdistance = cross_dist * 50.0;		
//	endlinedistance = 10000000000.0;

/*
bool IsRedLight()
{	
	if (traffic_lights_dog > 5 * 10) { // 5s
		forwd = 1;
		lturn = 1;
		rturn = 1;
		cout << "traffic lights is boom!" << endl;
		return(false);
	}

	if(0 && overpassflag==1)//if is overpassflag, never stop
		return(false);
	else
	{
		if(islight==1)
		{
			if((Gis_TurnDirection==0&&forwd==0)||(Gis_TurnDirection==1&&lturn==0)||(Gis_TurnDirection==2&&rturn==0))
				return(true);			
			else if(moveobs_num!=0)
				return(true);
			else
				return(false);
		}
		else if(islight==0)
		{
			if(moveobs_num!=0)
				return(true);
			else
				return(false);
		}
		else
			return(false);
	}
}
*/

bool IsRedLight() {
	// watch dog
	if (traffic_lights_dog > 5 * 10) { // 5s
		forwd = 1;
		lturn = 1;
		rturn = 1;
		cout << "traffic lights is boom!" << endl;
		return(false);
	}
	
	// infer which_light by myself
	if (islight == 1) {
		which_light = Gis_TurnDirection;
	} else {
		which_light = -1;
	}
	
	// check light
	if ((which_light == 0 && forwd == 0) ||
	    (which_light == 1 && lturn == 0) ||
	    (which_light == 2 && rturn == 0)) {
		return true;
	} else {
		return false;
	}
}

//Select_bestlane
long Caculatespeed()
{
	long speedsenttemp,speed_gistemp;
	speedsent = speed_gis;
	speed_gistemp = speed_gis;
	IsEstop = false;
	IsEstop2 = false;
	IsEstop3 = false;

	double distancefront;
	double distancetemp;
	double distancemintemp = 10000000000000000.0;
	int nearcarindextemp;
	int nearcarindextemp2;	
	double distancetemp2;

	if(crossdistance<2000.0&&crossdistance>=0.0)//near cross
	{
		if(speed_gistemp>80000)
			speed_gistemp = 80000;
		speedsenttemp = crossdistance / 2000.0 * speed_gistemp;

		if(crossdistance<600.0&&crossdistance>0.0)//0m - 8m
		{
			speedsenttemp = 9000;
			if(IsRedLight()&&CrossDetFlag)
				IsEstop = true;
			else
				IsEstop = false;
		}	
		else
			WatchDogcount = 0;
	
		if(speedsenttemp<9000)
			speedsenttemp = 9000;	
	}
	else if(crossdistance<0)
	{
		WatchDogcount = 0;
		CrossDetFlag = true;
		speedsenttemp = 12000;//speed in cross is 12km/h
		ManDetFlag = true;
		WatchDogcount4 = 0;
	}
	else// others speed stratege
	{
		WatchDogcount = 0;
		CrossDetFlag = true;
		ManDetFlag = true;
		WatchDogcount4 = 0;
		for(int i=0;i<301;i++)
		{
			distancetemp = ((double)roadpts_Sent[i].x-1500.0)*((double)roadpts_Sent[i].x-1500.0)+((double)roadpts_Sent[i].y-1000.0)*((double)roadpts_Sent[i].y-1000.0);
			if(distancetemp<distancemintemp)
			{
				distancemintemp = distancetemp;
				nearcarindextemp = i;
			}
		}
		for(int i=nearcarindextemp;i<301;i++)//find the start point
		{
			distancetemp2 = ((double)roadpts_Sent[i].x-(double)roadpts_Sent[nearcarindextemp].x)*((double)roadpts_Sent[i].x-(double)roadpts_Sent[nearcarindextemp].x)+((double)roadpts_Sent[i].y-(double)roadpts_Sent[nearcarindextemp].y)*((double)roadpts_Sent[i].y-(double)roadpts_Sent[nearcarindextemp].y);

			nearcarindextemp2 = i;
			if(distancetemp2>=(frontoffset*distancemintemp))
			{
				break;
			}		
		}
		for(int i=nearcarindextemp2;i<301;i++) //check whether the lane is block
		{				
			if(roadpts_Sent[i].y<1)
				roadpts_Sent[i].y = 1;
			if(roadpts_Sent[i].y>2999)
				roadpts_Sent[i].y = 2999;
			if(roadpts_Sent[i].x<1)
				roadpts_Sent[i].x = 1;
			if(roadpts_Sent[i].x>2999)
				roadpts_Sent[i].x = 2999;	
			
			if( cvGet2D(CostMap,(3000-roadpts_Sent[i].y),roadpts_Sent[i].x).val[0] > 180)
			{
				distancefront = sqrt( ((double)roadpts_Sent[i].x-(double)roadpts_Sent[nearcarindextemp2].x)*((double)roadpts_Sent[i].x-(double)roadpts_Sent[nearcarindextemp2].x)+((double)roadpts_Sent[i].y-(double)roadpts_Sent[nearcarindextemp2].y)*((double)roadpts_Sent[i].y-(double)roadpts_Sent[nearcarindextemp2].y) );
			
				double speed_coef = distancefront / 2000.0;
				if (speed_coef > 1.0) speed_coef = 1.0;
				speedsenttemp = speed_coef * speed_gis;
				if(distancefront<500.0)//10m;in general, we won't select lane whose distancefront is shorter than 120.0
				{
					IsEstop3 = true;
				}
				else
				{
					IsEstop3 = false;
				}
				break;
			}
			if(i==300)
			{
				speedsenttemp = speed_gis;
			}				
		}
	}

	if(endlinedistance<2000.0)//near destination
	{		
		if(speed_gistemp>80000)
			speed_gistemp = 80000;
		speedsenttemp = endlinedistance / 2000.0 * speed_gistemp;
		if(endlinedistance<400.0||speedsenttemp<4000)//8m
		{
			speedsenttemp = 4000;
			if(endlinedistance<170.0)//3m stop car
				IsEstop2 = true;
			else
				IsEstop2 = false;
		}
	}
	if(speedsenttemp<10000)
		speedsenttemp = 10000;
	if(speedsenttemp>speed_gis)
		speedsenttemp = speed_gis;

	return(speedsenttemp);
}

double checkForwardSpeed(double cx, double cy) {
	double f_spd = 0.0;
	
	double min_dis = DBL_MAX;
	int min_id = -1;
	int objects_num = ibeo_objects.Object.size();
	for (int i = 0; i < objects_num; i++) {
		int y_3000 = 1000.0 + 0.01 * ibeo_objects.Object[i].ReferencePoint.x / LOCAL_COOR_RESOLUTION;
		int x_3000 = 1500.0 - 0.01 * ibeo_objects.Object[i].ReferencePoint.y / LOCAL_COOR_RESOLUTION;
		double dis = sqrt((cx - x_3000) * (cx - x_3000) + (cy - y_3000) * (cy - y_3000));
		if (dis < min_dis) {
			min_dis = dis;
			min_id = i;
		}
		//cvCircle(PlanMapShow, cvPoint(x_3000, 3000 - y_3000), 30, CV_RGB(0, 255, 255), -1);
	}
	
	double dis_thre = 4.8; // m
	if (min_dis < DBL_MAX) {
		sprintf(strtemp, "forward min_dis: %.3lfm", min_dis * 0.02);
			cvPutText(PlanMapShow, strtemp, cvPoint(2000,1100), &font, CV_RGB(255,0,0));
	} else {
		sprintf(strtemp, "forward min_dis: MAX");
			cvPutText(PlanMapShow, strtemp, cvPoint(2000,1100), &font, CV_RGB(255,0,0));
	}
	
	if (min_id >= 0) cout << "ibeo_objects.Object[min_id].RelativeVelocity.x: " << ibeo_objects.Object[min_id].RelativeVelocity.x << endl;
	cout << "min_id: " << min_id << endl;
	
	if (min_dis * 0.02 < dis_thre) {
		// TODO: unit should be paid attention! (I need km/h)
		// TODO: whether low speed objects should be ignored?
		double rela_speed = ibeo_objects.Object[min_id].RelativeVelocity.x * 0.01 * 3600.0 / 1000.0;
		f_spd = rela_speed + Cur_EV_Speed;
		sprintf(strtemp, "forward obj_rela_vel_x: %.1fkm/h", rela_speed);
			cvPutText(PlanMapShow, strtemp, cvPoint(2000,1300), &font, CV_RGB(255,0,0));
	} else {
		f_spd = 0.0;
	}
	
	return f_spd;
}

long Caculatespeed2()
{
	long speedsenttemp,speed_gistemp;
	//speedsent = speed_gis;
	//speed_gistemp = speed_gis;
	speedsent = max_speed_temp * 1000.0;
	speed_gistemp = max_speed_temp * 1000.0;
	IsEstop = false;
	IsEstop2 = false;
	IsEstop3 = false;

	g_distancefront = 0.0;
	double distancetemp;
	double distancemintemp = 10000000000000000.0;
	int nearcarindextemp;
	int nearcarindextemp2;	
	double distancetemp2;
	
	//cout << "crossdistance: " << crossdistance << endl;

	//if (IsRedLight() && crossdistance<600.0 && crossdistance>0.0)

	//if(crossdistance<2000.0&&crossdistance>=0)//near cross
	if(crossdistance<3000.0)//near cross
	{
		if(speed_gistemp>28000) {
			speed_gistemp = 28000;
		}
		speedsenttemp = crossdistance / 3000.0 * speed_gistemp;
		if (!IsRedLight()) {
			if (Gis_TurnDirection == 0) {
				if (speedsenttemp < 20000) {
					speedsenttemp = 20000;
				}
			} else {
				if (speedsenttemp < 14000) {
					speedsenttemp = 14000;
				}
			}
		}
		else {
			speedsenttemp *= 0.4;
			if (speedsenttemp > 20000) speedsenttemp = 20000;
			if (speedsenttemp < 5000) speedsenttemp = 5000;
			cout << "speedsenttemp: " << speedsenttemp << endl;
		}
		
		//if (!is_use_stop_line) {
		if (1) {
			if(crossdistance<650.0&&crossdistance>-3.0) { //0m - 8m
			//if(crossdistance<600.0) { //0m - 8m
				if(IsRedLight()&&CrossDetFlag) {
				
					if (fabs(Cur_EV_Speed) < 10.0) {
						IsEstop = true;
					}
					else {
						speedsenttemp = Cur_EV_Speed * 0.8 * 1000;
						if (speedsenttemp > 18000) speedsenttemp = 18000;
						IsEstop = false;
					}
				}
				else {
					IsEstop = false;
				}
			}
			else {
				WatchDogcount = 0;
			}
		} else {
			if(dis_to_stop_line<12.0&&dis_to_stop_line>0.0) {
			//if(dis_to_stop_line<10.0) {
				if(IsRedLight()&&CrossDetFlag) {
				
					if (Cur_EV_Speed < 10.0) {
						IsEstop = true;
					}
					else {
						speedsenttemp = Cur_EV_Speed * 0.8 * 1000;
						if (speedsenttemp > 18000) speedsenttemp = 18000;
						IsEstop = false;
					}
				}
				else {
					IsEstop = false;
				}
			}
			else {
				WatchDogcount = 0;
			}
		}
		
		//if(speedsenttemp<12000)
		//	speedsenttemp = 12000;
	}
	else {
		speedsenttemp = max_speed_temp * 1000.0;
	}
	
	IsEstop = false; // virtual wall master red light
	
	double pros_dis = 60.0; // m
	double discard_dis = 35.0; // m
	long link_speed_low = 27 * 1000;
	long avoid_need_speed = 30 * 1000;
	long must_down_dis = 47.0;
	
	if (1) {
	
		if (1) {
			WatchDogcount = 0;
			CrossDetFlag = true;
			ManDetFlag = true;
			WatchDogcount4 = 0;
		
			int nearest_id = NearPlanLaneIndex;
			int best_id = no_filt_SelectPlanPathIndex;
			
			//cout << "nearest_id, best_id: " << nearest_id << ", " << best_id << endl;
			
			int for_dir = (nearest_id <= best_id) ? 1 : -1;
			int for_sz = abs(nearest_id - best_id);
			int st_id, ed_id;
			if (nearest_id <= best_id) {
				st_id = nearest_id;
				ed_id = best_id;
			} else {
				st_id = best_id;
				ed_id = nearest_id;
			}
			double check_min_dis[1000];
			int    check_min_i[1000];
			bool have_speed_down = false;
			//for (int line_id = st_id; line_id <= ed_id; line_id++) {
			for (int k = 0; k <= for_sz; k++) {
				int line_id = nearest_id + k * for_dir;
				
				g_distancefront = 0.0;
				distancemintemp = DBL_MAX;
		
				//double estop_dis_decay_coe = (fabs(Cur_EV_Orient) / 600.0) * 1.2;
		
				double estop_dis = 9 - fabs(line_id - best_id) * 0.9;
				//double estop_dis = 7.0 - fabs(line_id - best_id) * estop_dis_decay_coe;
				if (estop_dis < 8) {
					estop_dis = 8;
				}
				//cout << "estop_dis_decay_coe: " << estop_dis_decay_coe << endl;
				//estop_dis = 5.0;
				
				//int check_max_id = 301;
			
				for(int i=0;i<301;i++)
				{
					distancetemp = ((double)plines[line_id][i].x-1500.0)*((double)plines[line_id][i].x-1500.0)+
					((double)plines[line_id][i].y-1000.0)*((double)plines[line_id][i].y-1000.0);
					if(distancetemp<distancemintemp)
					{
						distancemintemp = distancetemp;
						nearcarindextemp = i;
					}
				}
				for(int i=nearcarindextemp;i<301;i++)//find the start point
				{
					distancetemp2 = ((double)plines[line_id][i].x-(double)plines[line_id][nearcarindextemp].x)*
					((double)plines[line_id][i].x-(double)plines[line_id][nearcarindextemp].x)+
					((double)plines[line_id][i].y-(double)plines[line_id][nearcarindextemp].y)*
					((double)plines[line_id][i].y-(double)plines[line_id][nearcarindextemp].y);

					nearcarindextemp2 = i;
					if(distancetemp2>=(0.0*distancemintemp))
					{
						break;
					}		
				}
				
				//cvCircle(PlanMapShow, cvPoint(plines[line_id][nearcarindextemp2 + 1].x, 3000 - plines[line_id][nearcarindextemp2 + 1].y), 15, CV_RGB(0, 255, 0), -1);
				
				g_distancefront += (nearcarindextemp2 - nearcarindextemp) * 0.4 * 50.0;
				
				double farest_dis = 39.0;
				int farest_id = nearcarindextemp + 1 + farest_dis / 0.4;
				int check_id_decay_coe = (fabs(Cur_EV_Orient) / 300.0) * 60;
				int check_max_id = farest_id - fabs(line_id - best_id) * check_id_decay_coe;
				if (check_max_id < nearcarindextemp2 + 2) {
					check_max_id = nearcarindextemp2 + 2;	
				}
				//cvCircle(PlanMapShow, cvPoint(plines[line_id][check_max_id-1].x, 3000 - plines[line_id][check_max_id-1].y), 18, CV_RGB(0, 0, 255), -1);
				check_min_dis[line_id] = DBL_MAX;
				check_min_i[line_id] = 301 - 1;
				for (int i = nearcarindextemp2 + 1; i<check_max_id; i++) // check whether the lane is block
				{
					if (plines[line_id][i].y<1 || plines[line_id][i].y>2999 || plines[line_id][i].x<1 || plines[line_id][i].x>2999) {
						check_min_dis[line_id] = DBL_MAX;
						check_min_i[line_id] = i;
						break;
					}
					
					if (i == check_max_id-1) {
						check_min_dis[line_id] = DBL_MAX;
						check_min_i[line_id] = i;
						break;
					}
		
					g_distancefront += sqrt( ((double)plines[line_id][i].x-(double)plines[line_id][i-1].x)*
									((double)plines[line_id][i].x-(double)plines[line_id][i-1].x)+
									((double)plines[line_id][i].y-(double)plines[line_id][i-1].y)*
									((double)plines[line_id][i].y-(double)plines[line_id][i-1].y) );
			
					
					if( cvGet2D(CostMap,(3000-plines[line_id][i].y),plines[line_id][i].x).val[0] >= 255)
					{
						//cvCircle(PlanMapShow, cvPoint(plines[line_id][i].x, 3000 - plines[line_id][i].y), 18, CV_RGB(100, 0, 0), -1);
						check_min_dis[line_id] = g_distancefront;			
						check_min_i[line_id] = i;
						
						break;
					}
				}
			
			} // line_id
		
			double min_dis = DBL_MAX;
			int min_id = -1;
			for (int k = 0; k <= for_sz; k++) {
				int line_id = nearest_id + k * for_dir;
				if (check_min_dis[line_id] <= min_dis) {
					min_dis = check_min_dis[line_id];
					min_id = line_id;
				}
			}
			
			cout << "[check] min_id: " << min_id << endl;
			
			
			int line_id = min_id;
			int i = check_min_i[line_id];
			double estop_dis = 7.5;
			
			//if (is_in_fee) {
			//	estop_dis = 13.0;
			//}
			
			g_distancefront = min_dis;
			
			double obj_x = plines[line_id][i].x;
			double obj_y = plines[line_id][i].y;
			double dis2far = dis_to_far_obs * 50.0;
			if (dis2far < g_distancefront) {
				g_distancefront = dis2far;
				obj_x = obs_xy.x;
				obj_y = obs_xy.y;
			}
			
			if (g_distancefront > 5000.0)
				g_distancefront = 100.0 * 50.0;
			
			cout << "i_check: " << i << endl;
			
			// san huan
			//g_distancefront = g_distancefront * 0.3 + g_distancefront_last * 0.7;
			
			g_distancefront_last = g_distancefront;
			
			cvCircle(PlanMapShow, cvPoint(plines[line_id][i].x, 3000 - plines[line_id][i].y), 18, CV_RGB(255, 0, 0), -1);
		 	double forward_speed = checkForwardSpeed(obj_x, obj_y);
			
			sprintf(strtemp, "forward_speed: %.1lfkm/h", forward_speed);
			cvPutText(PlanMapShow, strtemp, cvPoint(2000,1500), &font, CV_RGB(255,0,0));
							
			/*if (IsRedLight()) {
				if (crossdistance < g_distancefront && crossdistance > 0.0) g_distancefront = crossdistance;
			}*/
			
			if (IsRedLight()) {
				if (crossdistance < g_distancefront && crossdistance > -3.0) {
					g_distancefront = crossdistance;
					if (estop_dis <= 13.0) {
						estop_dis = 13.0;
					}
				}
			}
			
			cout << "g_distancefront: " << g_distancefront << endl;			
			
			// san huan
			double chao_che_speed = 25.0;
			if (g_distancefront > 4990.0) {
				speedsenttemp = max_speed_temp * 1000.0;
			} else if (g_distancefront <= 4990.0 && g_distancefront > 400) {
				double dis = g_distancefront / 50.0;
				//double speed_40_km = (speedsenttemp / 1000.0) * 2.0 / 3.0;
				speedsenttemp = 5.0 + (dis - 8.0) * (60.0-5) / 92.0;
				speedsenttemp *= 1000.0;
					
			} else if (g_distancefront <= 400) {
			
				speedsenttemp = 5000.0;
				if(g_distancefront<estop_dis*50.0) // 10m; in general, we won't select lane whose g_distancefront is shorter than 120.0

				{
					IsEstop3 = true;
				}
				else

				{
					IsEstop3 = false;
				}
			}
			
			/*double chao_che_speed = 25.0;
			if (g_distancefront > 5000.0) {
				speedsenttemp = max_speed_temp * 1000.0;
			} else if (g_distancefront <= 5000.0 && g_distancefront > 4500) {
				double dis = g_distancefront / 50.0;
				//double speed_40_km = (speedsenttemp / 1000.0) * 2.0 / 3.0;
				speedsenttemp = 40.0 + 2.0 * (dis-90.0);
				speedsenttemp *= 1000.0;
					
			} else if (g_distancefront <= 4500 && g_distancefront > 2000) {
				double dis = g_distancefront / 50.0;
				speedsenttemp = chao_che_speed + (dis - 40) * (40.0-chao_che_speed) / 50.0;
				speedsenttemp *= 1000.0;
					
			} else if (g_distancefront <= 2000 && g_distancefront > 400) {
				
				double dis = g_distancefront / 50.0;
				speedsenttemp = 5.0 + (dis - 8) * (chao_che_speed-5.0) / 32.0;
				speedsenttemp *= 1000.0;
				
				if(g_distancefront<estop_dis*50.0)
				{
					IsEstop3 = true;
				}
				else
				{
					IsEstop3 = false;
				}
				
			} else if (g_distancefront <= 400) {
			
				speedsenttemp = 5000.0;
				if(g_distancefront<estop_dis*50.0) // 10m; in general, we won't select lane whose g_distancefront is shorter than 120.0
				{
					IsEstop3 = true;
				}
				else
				{
					IsEstop3 = false;
				}
			}*/
			
			/*if (g_distancefront > 5000.0) {
				speedsenttemp = max_speed_temp * 1000.0;
			} else if (g_distancefront <= 5000.0 && g_distancefront > 4500) {
				double dis = g_distancefront / 50.0;
				//double speed_40_km = (speedsenttemp / 1000.0) * 2.0 / 3.0;
				speedsenttemp = 40.0 + 2.0 * (dis-90.0);
				speedsenttemp *= 1000.0;
					
			} else if (g_distancefront <= 4500 && g_distancefront > 400) {
				double dis = g_distancefront / 50.0;
				speedsenttemp = 5 + (dis - 8) * 35.0 / 82.0;
				speedsenttemp *= 1000.0;
					
				if(g_distancefront<estop_dis*50.0) // 10m; in general, we won't select lane whose g_distancefront is shorter than 120.0
				{
					IsEstop3 = true;
				}
				else
				{
					IsEstop3 = false;
				}
			} else if (g_distancefront <= 400) {
			
				speedsenttemp = 5000.0;
				if(g_distancefront<estop_dis*50.0) // 10m; in general, we won't select lane whose g_distancefront is shorter than 120.0
				{
					IsEstop3 = true;
				}
				else
				{
					IsEstop3 = false;
				}
			}*/
			
			// dyna. obs.
			cout << "[before] speedsenttemp: " << speedsenttemp << endl;
			speedsenttemp = speedsenttemp + forward_speed * 1000.0;
			if (speedsenttemp > (max_speed_temp * 1000.0)) speedsenttemp = max_speed_temp * 1000.0;
			cout << "[after] speedsenttemp: " << speedsenttemp << endl;
			
		} // if (1)
	}
	
	if(endlinedistance<5000.0)//near destination
	{		
		speedsenttemp = (endlinedistance / 5000.0) * (max_speed_temp * 1000.0);
		if (speedsenttemp < 0.0) speedsenttemp = 0.0;
		
		if(endlinedistance<1000.0||speedsenttemp<5000)//8m
		{
			speedsenttemp = 5000;
			if(endlinedistance<340.0)
				IsEstop2 = true;
			else
				IsEstop2 = false;
		}
	}
	
	if(strategemode==3)
	{
		if(speedsenttemp<5000)
			speedsenttemp = 5000;
		if(speedsenttemp>speed_gis)
			speedsenttemp = speed_gis;
	}
	else
	{
		if(speedsenttemp<5000)
			speedsenttemp = 5000;
		if(speedsenttemp>speed_gis)
			speedsenttemp = speed_gis;
	}
		
	return(speedsenttemp);	
}

void SelectSentWay()
{
	//laneincarnum = getParaLine(Select_nearestlane, cvPoint(1500,1000), LaneIncar);
	if(strategemodelast != strategemode)
	{
		for(int i=0;i<301;i++)
		{
			Select_bestlanelast[i] = LaneIncarforlast[strategemode-1][i];
			Select_sentlane[i] = Select_bestlanelast[i];
		}
	}	
	else
	{
		double distancemin1 = 10000000000000000.0;
		double distance1;
		int nearcarindextemp;	
		double distancemin2 = 10000000000000000.0;
		double distancemintemp2 = 10000000000000000.0;
		double distance2;
		double distancetemp2;
		int nearcarindextemp2;	
		CvPoint xytemp;
		CvPoint xyafterins;
		double xyafterinsx,xyafterinsy;
		double xbeforeins,ybeforeins;

		for(int j=0; j<301;j++)
		{
			distance2 = (1500.0 - (double)Select_bestlanelast[j].x)*(1500.0 - (double)Select_bestlanelast[j].x)+(1000.0 - (double)Select_bestlanelast[j].y)*(1000.0 - (double)Select_bestlanelast[j].y);
			if(distance2<distancemin2)
			{
				distancemin2 = distance2;
				nearcarindextemp2 = j;
			}
		}		

		xbeforeins = (double)Select_bestlanelast[nearcarindextemp2].x;
		ybeforeins = (double)Select_bestlanelast[nearcarindextemp2].y;
		doRT2(xbeforeins,ybeforeins,xyafterinsx,xyafterinsy,XX_last,YY_last,XX,YY,ZZ_last,ZZ);
		xyafterins.x = (int)xyafterinsx;
		xyafterins.y = (int)xyafterinsy;
	
		if(xyafterins.x<3000&&xyafterins.x>0&&xyafterins.y>0&&xyafterins.y<3000)
		{
			for(int j=0; j<301;j++)
			{
				distance1 = ((double)Select_bestlane[j].x - (double)xyafterins.x)*((double)Select_bestlane[j].x - (double)xyafterins.x)+((double)Select_bestlane[j].y - (double)xyafterins.y)*((double)Select_bestlane[j].y - (double)xyafterins.y);
				if(distance1<distancemin1)
				{
					distancemin1 = distance1;
					nearcarindextemp = j;
				}
			}
			xytemp.x =(int) (((double)Select_bestlane[nearcarindextemp].x * (double)lanechangespeed + (double)xyafterins.x * (double)(100-lanechangespeed) ) / 100.0);
			xytemp.y = (int)( ((double)Select_bestlane[nearcarindextemp].y * (double)lanechangespeed + (double)xyafterins.y * (double)(100-lanechangespeed) ) / 100.0);
			
			for(int j=0; j<301;j++)
			{
				distancetemp2 = ((double)xytemp.x - (double)Select_bestlane[j].x)*((double)xytemp.x - (double)Select_bestlane[j].x)+((double)xytemp.y - (double)Select_bestlane[j].y)*((double)xytemp.y - (double)Select_bestlane[j].y);
				if(distancetemp2<distancemintemp2)
				{
					distancemintemp2 = distancetemp2;
				}
			}	
			//fprintf(stderr,"distancemintemp2 %f\n",sqrt(distancemintemp2));
			double distancetemptobest;	
			distancetemptobest = sqrt(distancemintemp2);
			if(distancetemptobest>10.0&&distancetemptobest<50.0)
			{
				xytemp.x =(int) (((double)Select_bestlane[nearcarindextemp].x * 15.0 + (double)xytemp.x * 85.0 ) / 100.0);
				xytemp.y = (int)(((double)Select_bestlane[nearcarindextemp].y * 15.0 + (double)xytemp.y * 85.0 ) / 100.0);
				paralane(Select_bestlane,xytemp,Select_sentlane);
			}
			else if(distancetemptobest>=50.0)		
				paralane(Select_bestlane,xytemp,Select_sentlane);
			else
			{
				for(int i=0;i<301;i++)
				{
					Select_sentlane[i] = Select_bestlane[i];
				}
			}
			for(int i=0;i<301;i++)
			{
				Select_bestlanelast[i] = Select_sentlane[i];
			}
		}
		else
		{
			for(int i=0;i<301;i++)
			{		
				Select_sentlane[i] = Select_bestlane[i];
				Select_bestlanelast[i] = Select_sentlane[i];
			}
		}
	}
}

void SelectSentWay2()
{
	/*if(strategemodelast != strategemode)
	{
		for(int i=0;i<301;i++)
		{
			Select_bestlanelast[i] = LaneIncarforlast[strategemode-1][i];
			Select_sentlane[i] = Select_bestlanelast[i];
		}
	}	
	else
	{
		double distancemin1 = 10000000000000000.0;
		double distance1;
		int nearcarindextemp;	
		double distancemin2 = 10000000000000000.0;
		double distance2;
		int nearcarindextemp2;	
		double distancemintemp2 = 10000000000000000.0;
		double distancetemp2;
		CvPoint xytemp;
		CvPoint xyafterins;
		double xyafterinsx,xyafterinsy;
		double xbeforeins,ybeforeins;
		int tempnum;
		
		for(int j=0; j<301;j++)
		{
			distance2 = (1500.0 - (double)Select_bestlanelast[j].x)*(1500.0 - (double)Select_bestlanelast[j].x)+(1000.0 - (double)Select_bestlanelast[j].y)*(1000.0 - (double)Select_bestlanelast[j].y);
			if(distance2<distancemin2)
			{
				distancemin2 = distance2;
				nearcarindextemp2 = j;
			}
		}
		xbeforeins = (double)Select_bestlanelast[nearcarindextemp2].x;
		ybeforeins = (double)Select_bestlanelast[nearcarindextemp2].y;
		doRT2(xbeforeins,ybeforeins,xyafterinsx,xyafterinsy,XX_last,YY_last,XX,YY,ZZ_last,ZZ);
		xyafterins.x = (int)xyafterinsx;
		xyafterins.y = (int)xyafterinsy;
		
		if(xyafterins.x<3000&&xyafterins.x>0&&xyafterins.y>0&&xyafterins.y<3000)
		{
			for(int j=0; j<301;j++)
			{
				distance1 = ((double)plines[SelectPlanPathIndex][j].x - (double)xyafterins.x)*((double)plines[SelectPlanPathIndex][j].x - (double)xyafterins.x)+((double)plines[SelectPlanPathIndex][j].y - (double)xyafterins.y)*((double)plines[SelectPlanPathIndex][j].y - (double)xyafterins.y);
				if(distance1<distancemin1)
				{
					distancemin1 = distance1;
					nearcarindextemp = j;
				}
			}
			xytemp.x = (plines[SelectPlanPathIndex][nearcarindextemp].x * lanechangespeed + xyafterins.x * (100-lanechangespeed) ) / 100;
			xytemp.y = (plines[SelectPlanPathIndex][nearcarindextemp].y * lanechangespeed + xyafterins.y * (100-lanechangespeed) ) / 100;
				
			cvCircle(PlanMapShow,cvPoint(xytemp.x,3000-xytemp.y),6,CV_RGB(255,255,0),3);
			if(0 && strategemode==3)
			{	
				for(int j=0; j<301;j++)
				{
					distancetemp2 = ((double)xytemp.x - (double)plines[SelectPlanPathIndex][j].x)*((double)xytemp.x - (double)plines[SelectPlanPathIndex][j].x)+((double)xytemp.y - (double)plines[SelectPlanPathIndex][j].y)*((double)xytemp.y - (double)plines[SelectPlanPathIndex][j].y);
					if(distancetemp2<distancemintemp2)
					{
						distancemintemp2 = distancetemp2;
					}
				}	
				double distancetemptobest;	
				distancetemptobest = sqrt(distancemintemp2);
				if(distancetemptobest>10.0&&distancetemptobest<50.0)
				{
					xytemp.x = (int)(((double)plines[SelectPlanPathIndex][nearcarindextemp].x * 15.0 + (double)xytemp.x * 85.0 ) / 100.0);
					xytemp.y = (int)(((double)plines[SelectPlanPathIndex][nearcarindextemp].y * 15.0 + (double)xytemp.y * 85.0 ) / 100.0);
					tempnum = getParaLine(Gis_Lane, xytemp, Laneparatemp);		
					for(int i=0;i<301;i++)
					{
						if(tempnum>-1)
						{
							Select_sentlane[i] = Laneparatemp[tempnum][i];
							Select_bestlanelast[i] = Select_sentlane[i];
						}	
						else
						{
							Select_sentlane[i] = plines[SelectPlanPathIndex][i];
							Select_bestlanelast[i] = Select_sentlane[i];
						}	
					}
				}
				else if(distancetemptobest>=50.0)
				{
					tempnum = getParaLine(Gis_Lane, xytemp, Laneparatemp);		
					for(int i=0;i<301;i++)
					{
						if(tempnum>-1)
						{
							Select_sentlane[i] = Laneparatemp[tempnum][i];
							Select_bestlanelast[i] = Select_sentlane[i];
						}	
						else
						{
							Select_sentlane[i] = plines[SelectPlanPathIndex][i];
							Select_bestlanelast[i] = Select_sentlane[i];
						}	
					}
				}
				else		
				{
					for(int i=0;i<301;i++)
					{
						Select_sentlane[i] = plines[SelectPlanPathIndex][i];
						Select_bestlanelast[i] = Select_sentlane[i];
					}
				}
			}
			else
			{
				for(int j=0; j<301;j++)
				{
					distancetemp2 = ((double)xytemp.x - (double)plines[SelectPlanPathIndex][j].x)*((double)xytemp.x - (double)plines[SelectPlanPathIndex][j].x)+((double)xytemp.y - (double)plines[SelectPlanPathIndex][j].y)*((double)xytemp.y - (double)plines[SelectPlanPathIndex][j].y);
					if(distancetemp2<distancemintemp2)
					{
						distancemintemp2 = distancetemp2;
					}
				}	
				double distancetemptobest;	
				distancetemptobest = sqrt(distancemintemp2);
				no_filter = false;
				if(distancetemptobest>6.0&&distancetemptobest<50.0)
				{
					xytemp.x = (int)(((double)plines[SelectPlanPathIndex][nearcarindextemp].x * 15.0 + (double)xytemp.x * 85.0 ) / 100.0);
					xytemp.y = (int)(((double)plines[SelectPlanPathIndex][nearcarindextemp].y * 15.0 + (double)xytemp.y * 85.0 ) / 100.0);
					paralane(plines[SelectPlanPathIndex],xytemp,Select_sentlane);
					
					filter_speed_up = true;
					
					//if (distancetemptobest < 25.0) no_filter = true;
				}
				else if(distancetemptobest>=50.0)
				{
					paralane(plines[SelectPlanPathIndex],xytemp,Select_sentlane);
					filter_speed_up = false;
				}
				else		
				{
					for(int i=0;i<301;i++)
					{
						Select_sentlane[i] = plines[SelectPlanPathIndex][i];
					}
					filter_speed_up = true;
					no_filter = true;
				}
								
				for(int i=0;i<301;i++)
				{
					Select_bestlanelast[i] = Select_sentlane[i];
				}	
			}
		}
		//else
		if (1)
		{
			//ROS_ERROR("No filter because out 3000!");
			for(int i=0;i<301;i++)
			{		
				Select_sentlane[i] = plines[SelectPlanPathIndex][i];
				Select_bestlanelast[i] = Select_sentlane[i];
			}
		}
	}*/	

	if (1)
	{
		//ROS_ERROR("No filter because out 3000!");
		for(int i=0;i<301;i++)
		{		
			Select_sentlane[i] = plines[SelectPlanPathIndex][i];
			Select_bestlanelast[i] = Select_sentlane[i];
		}
	}
}

void processTrafficLane()
{
	int StopLineNum = 0;
	int* StopLine = new int[20];
	pathwaynum = 0;	

	//due delay
	/*C_InsFrame matchinsframe;
	bool iffind = InsFrameHis.getMatchInsFrame((long)Timer_lanemodel,matchinsframe);
	double afterinsx,afterinsy;
	fprintf(stdout,"X: %f, Y: %f, Z: %f\n",matchinsframe.position[0],matchinsframe.position[1],matchinsframe.attitude[2]);
	if(iffind)
	{
		fprintf(stderr,"find and correct!\n");
		for(int i=0;i<620;i++)
		{
			doRT2((double)laneModelInfo.points_x[i],(double)laneModelInfo.points_y[i],afterinsx,afterinsy,matchinsframe.position[0],matchinsframe.position[1],XX,YY,matchinsframe.attitude[2],ZZ);
			laneModelInfo.points_x[i] = (int)afterinsx;
			laneModelInfo.points_y[i] = (int)afterinsy;
		}
	}*/

	for(int i=0;i<20;i++)
	{
		for(int j=0;j<31;j++)
		{
			traffic_lane[i][j].x = laneModelInfo.points_x[i*31+j];
			traffic_lane[i][j].y = laneModelInfo.points_y[i*31+j];
		}
	}
	for(int i=0;i<20;i++)
	{
		if((laneModelInfo.type[i]==1||laneModelInfo.type[i]==3)&&laneModelInfo.isCurve[i]==0)//zhi xian
		{
			for(int j=0; j<30; j++)
			{
				traffic_lanePro[pathwaynum][j*10] = traffic_lane[i][j];
				for(int k=1; k<=9; k++)
				{
					int deltax = traffic_lane[i][j+1].x - traffic_lane[i][j].x;
					int deltay = traffic_lane[i][j+1].y - traffic_lane[i][j].y;
					traffic_lanePro[pathwaynum][j*10+k].x = traffic_lane[i][j].x + deltax*k/9.0;
					traffic_lanePro[pathwaynum][j*10+k].y = traffic_lane[i][j].y + deltay*k/9.0;
				}
			}
			traffic_lanePro[pathwaynum][300] = traffic_lane[i][30];
			pathwaynum++;//this is the number of qualified traffic lines store in the format of 301
		}

		if(laneModelInfo.type[i]==5&&laneModelInfo.isCurve[i]==0)//ting zhi xian
		{
			StopLine[StopLineNum]=i;
			StopLineNum++;
		}
	}
	trafficlanenum = pathwaynum;//this is the number of qualified traffic lines detected by wangxinyu
	for(int i=0;i<(trafficlanenum-1);i++)
	{
		for(int k=1;k<8;k++)
		{
			for(int j=0;j<301;j++)
			{
				traffic_lanePro[pathwaynum][j].x = (traffic_lanePro[i][j].x * (8-k) + traffic_lanePro[i+1][j].x * k) / 8;
				traffic_lanePro[pathwaynum][j].y = (traffic_lanePro[i][j].y * (8-k) + traffic_lanePro[i+1][j].y * k) / 8;
			}
			pathwaynum++;
		}
	}//the 8 center lines of traffic lanes

	//crossdistance = 10000000000.0;
	crossdistancetemp = 10000000000.0;
	double dcar;
	
	dcar = Cur_EV_Odometer - Cur_EV_Odometerlast;
	if(dcar>10000.0)
		dcar = 0.0;	
	if(cross_dist>0.0&&cross_dist<30.0)//25m
	{
		if(!INSflag)
		{
			if(StopLineNum>0)
			{
				for(int i=0;i<StopLineNum;i++)
				{
					for(int j=0;j<31;j++)//ŒÆËãÃ¿žöÍ£Ö¹ÏßµÄYµÄÆœŸùÖµ
					{	
						tempdist += (double)traffic_lane[StopLine[i]][j].y - 1000.0;
					}
					tempdist = tempdist / 31.0;
					if(tempdist<0)
						tempdist = 0.0;
					//fprintf(stdout,"tempdist: %f\n",tempdist);

					if(tempdist<crossdistancetemp)
						crossdistancetemp = tempdist;
					
					//fprintf(stdout,"crossdistancetemp: %f\n",crossdistancetemp);
				}
				coutfilter = coutfilter + filterup;
			}
			else
			{
				coutfilter = coutfilter - filterdown;
				//crossdistancetemp = 10000000000.0;
				crossdistance = cross_dist * 50.0;
			}
			if(coutfilter<0)
				coutfilter = 0;
						
			if(coutfilter>70)
				coutfilter = 70;

			//fprintf(stdout,"crossdistanceins: %f\n",crossdistanceins);
			if(coutfilter>stoplanefilter)
			{		
				INSflag = true;
				crossdistanceins = crossdistancetemp;		
			}
			else
			{		
				INSflag = false;
				crossdistanceins = 10000000000.0;
				crossdistance = cross_dist * 50.0;
			}
		}
		else
		{
			insdistance += dcar * 0.5;
			crossdistance = crossdistanceins - insdistance;
		}		
	}
	else if(cross_dist<=0.0)
	{
		crossdistance = cross_dist * 50.0;		
		INSflag = false;
		coutfilter = 0;
		crossdistancetemp = 10000000000.0;
	}	
	else
	{
		crossdistance = cross_dist * 50.0;
		if(cross_dist>35.0)
		{
			//crossdistance = 10000000000.0;
			INSflag = false;
			insdistance = 0.0;
			coutfilter = 0;
			//crossdistancetemp = 10000000000.0;
		}	
	}
	//fprintf(stdout,"coutfilter: %d\n",coutfilter);

	endlinedistance = 10000000000.0;
	double tempdist2;
	if(end_dist>0&&end_dist<40)//40m
	{
		if(StopLineNum>0)
		{
			for(int i=0;i<StopLineNum;i++)
			{
				for(int j=0;j<31;j++)//ŒÆËãÃ¿žöÍ£Ö¹ÏßµÄYµÄÆœŸùÖµ
				{	
					tempdist2 += (double)traffic_lane[StopLine[i]][j].y -1000.0;
				}
				tempdist2 = tempdist2 / 31.0;
				if(tempdist2<0)
					tempdist2 = 10000000000.0;

				if(tempdist2<endlinedistance)
					endlinedistance = tempdist2;
			}
			if(endlinedistance>=10000000000.0)
				endlinedistance = end_dist * 50.0;
		}
		else
			endlinedistance = end_dist * 50.0;
	}
	else if(end_dist<=0)
	{
		endlinedistance = end_dist * 50.0;
	}
	else
		endlinedistance = 10000000000.0;
	
	if(endlinedistance<0)
		endlinedistance = 0;

	if(trafficlanenum==0)
	{
		for(int i=0;i<301;i++)
		{
			LaneIncarforlast[0][i].x = 1500;
			LaneIncarforlast[0][i].y = i*9+1;
		}
	}
	else
	{
		paralane(traffic_lanePro[0], cvPoint(1500,1000), LaneIncarforlast[0]);
	}
	
	delete[] StopLine;
}

void paralane(CvPoint* lanetopara, CvPoint point, CvPoint* outputlane)
{
	double distancemin1 = 10000000000000000.0;
	double distance1;
	int nearcarindextemp;
	int xoffset,yoffset;	
	for(int j=0; j<301;j++)
	{
		distance1 = ((double)lanetopara[j].x - (double)point.x)*((double)lanetopara[j].x - (double)point.x)+((double)lanetopara[j].y - (double)point.y)*((double)lanetopara[j].y - (double)point.y);
		if(distance1<distancemin1)
		{
			distancemin1 = distance1;
			nearcarindextemp = j;
		}
	}
	xoffset = point.x - lanetopara[nearcarindextemp].x;
	yoffset = point.y - lanetopara[nearcarindextemp].y;
	
	for(int i=0;i<301;i++)
	{
		outputlane[i].x = lanetopara[i].x +  xoffset;
		outputlane[i].y = lanetopara[i].y +  yoffset;
		/*if(outputlane[i].x<=1)
			outputlane[i].x = 1;
		if(outputlane[i].x>2999)
			outputlane[i].x=2999;
		if(outputlane[i].y<=1)
			outputlane[i].y = 1;
		if(outputlane[i].y>2999)
			outputlane[i].y=2999;*/
	}
}

void processGisLane()
{	
	/*laneincarnum = getParaLine(Gis_Lane, cvPoint(1500,1000), LaneIncar);
	if(laneincarnum>=0)
	{
		for(int i=0;i<301;i++)
		{
			LaneIncarforlast[2][i] = LaneIncar[laneincarnum][i];
		}	
	}*/	
}

/*void processTFL()
{
	lturn = ((tfl.data[1]&(0x0f<<6*4))>>6*4);
	forwd = ((tfl.data[1]&(0x0f<<5*4))>>5*4);
	rturn = ((tfl.data[1]&(0x0f<<4*4))>>4*4);
	lalarm = ((tfl.data[1]&(0x0f<<3*4))>>3*4);
	ralarm = ((tfl.data[1]&(0x0f<<2*4))>>2*4);
	upalarm = ((tfl.data[1]&(0x0f<<1*4))>>1*4);
}*/

void processTFSN()
{
	tfsn_num = (int)tfsn.data[0x01];
	long tfsn_info;
	if(tfsn_num>0)
	{
		for(int i=2;i<tfsn_num+2;i++)
		{
			tfsn_info = tfsn.data[i];
			tfsn_X[i-2] = (tfsn_info>>20)&0xFFF;
			tfsn_Y[i-2] = (tfsn_info>>8)&0xFFF;
			tfsn_type[i-2] = (tfsn_info)&0xFF;
			//fprintf(stderr,"TFSN %d: (%d,%d,%d)\n",i-1, tfsn_X[i-2],tfsn_Y[i-2],tfsn_type[i-2]);
		}
	}
}

void processRoadLane()
{
	double rightlane_x0;
	double rightlane_y0;
	double rightlane_x1;
	double rightlane_y1;
	double leftedge_x0;
	double leftedge_y0;
	double leftedge_x1;
	double leftedge_y1;
	double rightedge_x0;
	double rightedge_y0;
	double rightedge_x1;
	double rightedge_y1;
	

	FindEdgePoint(roadedge.Angle, roadedge.leftdis + Offset * ((-roadedge.rightdis)-roadedge.leftdis), &rightlane_x0, &rightlane_y0, &rightlane_x1, &rightlane_y1);
	FindEdgePoint(roadedge.Angle, roadedge.leftdis, &leftedge_x0, &leftedge_y0, &leftedge_x1, &leftedge_y1);
	FindEdgePoint(roadedge.Angle, -roadedge.rightdis, &rightedge_x0, &rightedge_y0, &rightedge_x1, &rightedge_y1);
	
	if(leftedge_x0!=leftedge_x1)
	{
		double distanceX1 = leftedge_x1 - leftedge_x0;
		double Xstep1 = distanceX1 / 300.0;
		double distanceY1 = leftedge_y1 - leftedge_y0;
		double k1 = distanceY1 / distanceX1;
		for(int j = 0 ; j<301 ; j++)
		{
			roadpts1_lane[j].x = (int)(leftedge_x0 + (double)j * Xstep1);
			roadpts1_lane[j].y = (int)( k1 * ((double)roadpts1_lane[j].x-leftedge_x0) + leftedge_y0 );
		}
	}
	else
	{
		double distanceY1 = leftedge_y1 - leftedge_y0;
		double Ystep1 = distanceY1 / 300.0;
		for(int j = 0 ; j<301 ; j++)
		{
			roadpts1_lane[j].x = (int)(leftedge_x0);
			roadpts1_lane[j].y = (int)(leftedge_y0 + (double)j * Ystep1);
		}
	}
	
	if(rightedge_x0!=rightedge_x1)
	{
		double distanceX2 = rightedge_x1 - rightedge_x0;
		double Xstep2 = distanceX2 / 300.0;
		double distanceY2 = rightedge_y1 - rightedge_y0;
		double k2 = distanceY2 / distanceX2;
		for(int j = 0 ; j<301 ; j++)
		{
			roadpts2_lane[j].x = (int)(rightedge_x0 + (double)j * Xstep2);
			roadpts2_lane[j].y = (int)( k2 * ((double)roadpts2_lane[j].x-rightedge_x0) + rightedge_y0 );
		}
	}
	else
	{
		double distanceY2 = rightedge_y1 - rightedge_y0;
		double Ystep2 = distanceY2 / 300.0;
		for(int j = 0 ; j<301 ; j++)
		{
			roadpts2_lane[j].x = (int)(rightedge_x0);
			roadpts2_lane[j].y = (int)(rightedge_y0 + (double)j * Ystep2);
		}
	}

	double dx = rightlane_x1 - rightlane_x0;
	double dy = rightlane_y1 - rightlane_y0;
	int cnt(0);
	roadpts_lane[cnt].x = rightlane_x0;
	roadpts_lane[cnt].y = rightlane_y0;
	cnt++;
	double step = 0.0;
	for(;;)
	{
		double tmpx = 0.0;
		double tmpy = 0.0;
		for(;;)
		{
			step += 0.001;
			tmpx = rightlane_x0 + step*dx;
			tmpy = rightlane_y0 + step*dy;
			double dis = (tmpx-rightlane_x0)*(tmpx-rightlane_x0) + (tmpy-rightlane_y0)*(tmpy-rightlane_y0);
			if(dis > (double)((double)cnt*(double)cnt*0.16*2500.0) )
			{
				break;
			}
		}
		if(step <= 1)
		{
			roadpts_lane[cnt].x = tmpx;
			roadpts_lane[cnt].y = tmpy;
			cnt++;
		}
		else
		{
			break;
		}
	}
	for(int i=cnt; i<301; i++)
	{
		roadpts_lane[i] = roadpts_lane[cnt-1];
	}

	//fprintf(stdout,"pathwaynum:%d\n",pathwaynum);
	if(speed_gis>=0)
	{
		for (int i=0;i<301;i++)
		{
			traffic_lanePro[pathwaynum][i]=roadpts_lane[i];
		}
	}
	else
	{
		for (int i=0;i<301;i++)
		{
			traffic_lanePro[pathwaynum][300-i]=roadpts_lane[i];
		}
	}
	pathwaynum++;
	//store the roadedge right line in the traffic_lanePro[] group
	paralane(traffic_lanePro[pathwaynum-1], cvPoint(1500,1000), LaneIncarforlast[1]);	
}

void FindEdgePoint(double Angle,double dis,double *p1x,double *p1y,double *p2x,double *p2y)
{
	double angletTemp = (90.0 - Angle) * 3.1415926 / 180.0;
	double LeftOffset;
	LeftOffset = dis * 50.0;
	double cosangle = cos(angletTemp);
	double sinangle = sin(angletTemp);
	double cosangle2 = cos( Angle*3.1415926 / 180.0);
	CvPoint *points = new CvPoint[4];
	int PointsNum = 0;
	double xx,yy;
	if( (Angle>1.0&&Angle<89.0)||(Angle<-1.0&&Angle>-89.0))
	{
		xx=0.0;
		yy = sinangle / cosangle * (xx - 1500.0 + LeftOffset/cosangle2) + 1000.0;
		if(yy>=0.0&&yy<=2999.0)
		{
			points[PointsNum].x = 0.0;
			points[PointsNum++].y = yy; 
		}
		xx=2999.0;
		yy = sinangle / cosangle * (xx - 1500.0 + LeftOffset/cosangle2) + 1000.0;
		if(yy>=0.0&&yy<=2999.0)
		{
			points[PointsNum].x = 2999.0;
			points[PointsNum++].y = yy; 
		}
		yy = 0.0;
		xx = cosangle / sinangle * ( yy - 1000.0) + 1500.0 -  LeftOffset/cosangle2;
		if(xx>0.0&&xx<2999.0)
		{
			points[PointsNum].x = xx;
			points[PointsNum++].y = 0.0; 
		}
		yy = 2999.0;
		xx = cosangle / sinangle * ( yy - 1000.0) + 1500.0 -  LeftOffset/cosangle2;
		if(xx>0.0&&xx<2999.0)
		{
			points[PointsNum].x = xx;
			points[PointsNum++].y = 2999.0; 
		}
	}
	else if(Angle<=1.0 && Angle>=-1.0)
	{
		points[0].y = 0.0;
		points[0].x = 1500.0 - LeftOffset;
		points[1].y = 2999.0;
		points[1].x = 1500.0 - LeftOffset;
	}
	else
	{
		points[0].x = 0.0;
		points[0].y = 1000.0 + LeftOffset;
		points[1].x = 2999.0;
		points[1].y = 1000.0 + LeftOffset;
	}
	if(points[0].y>=points[1].y)
	{
		*p1x = points[1].x;
		*p1y = points[1].y;
		*p2x = points[0].x;
		*p2y = points[0].y;
	}
	else
	{
		*p1x = points[0].x;
		*p1y = points[0].y;
		*p2x = points[1].x;
		*p2y = points[1].y;
	}
	
	delete[] points;
}

void EStop()
{
	
	for (int i = 0; i < 301; i++) {
    
		CvPoint2D64f pt_enu = conv3000toENU(roadpts_Sent[i].x, roadpts_Sent[i].y,
	    						XX_used, YY_used, ZZ_used);
		dec_msg.RoadPoints[i].x = pt_enu.x;
		dec_msg.RoadPoints[i].y = pt_enu.y;
	}
	
	dec_msg.Mode = 3;
	dec_msg.SpeedLimit = 0;
	
	//tcp_publisher_.publish(tcp_msg);
	dec_info_pub_.publish(dec_msg);
}

void mcvSendRoadPoints(CvPoint *roadpoints,long &speed)
{
    //for(int i=0;i<301;i++)
    //{
	//tcp_msg.x[i] = (long)roadpoints[i].x;
	//tcp_msg.y[i] = (long)roadpoints[i].y;
    //}
    
    for (int i = 0; i < 301; i++) {
    
    	CvPoint2D64f pt_enu = conv3000toENU(roadpoints[i].x, roadpoints[i].y,
    						XX_used, YY_used, ZZ_used);
    	dec_msg.RoadPoints[i].x = pt_enu.x;
	dec_msg.RoadPoints[i].y = pt_enu.y;
    }
    
    //tcp_msg.data[0] = (long)1;
    //tcp_msg.data[0xFE] = (long)1;

    dec_msg.Mode = 1;
    
			if(crossdistance<4000.0 && crossdistance >= 3000.0) {
				if (Gis_TurnDirection == 0) {
					if (speed > 34000) {
						speed = 34000;
					}
				} else {
					if (speed > 28000) {
						speed = 28000;
					}
				}
			} else if(crossdistance<3000.0 && crossdistance >= 2000.0) {
				if (Gis_TurnDirection == 0) {
					if (speed > 31000) {
						speed = 31000;
					}
				} else {
					if (speed > 25000) {
						speed = 25000;
					}
				}
			} else if (crossdistance < 2000.0 && crossdistance >= 1000.0) {
				if (Gis_TurnDirection == 0) {
					if (speed > 28000) {
						speed = 28000;
					}
				} else {
					if (speed > 22000) {
						speed = 22000;
					}
				}
			} else if (crossdistance < 1000.0) {
				if (Gis_TurnDirection == 0) {
					if (speed > 25000) {
						speed = 25000;
					}
				} else {
					if (speed > 18000) {
						speed = 18000;
					}
				}
			}

    if(speed_gis>=0)
    {
	if(speed>speed_gis)
		speed = speed_gis;	
    }    
    else
    {
	if(speed<speed_gis)
		speed = speed_gis;	
    }
	
    //speed = 100 * 1000;
    sprintf(strtemp,"speed: %ld",speed);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,700),&font,CV_RGB(255,0,0));

    //tcp_msg.data[0xFD] = 12 * 1000; //speed;//(long)40000;
    
    dec_msg.SpeedLimit = speed / 1000.0; // no 1000
    
    dec_info_pub_.publish(dec_msg);

}

void doRT2(double src_x,double src_y,double &dst_x,double &dst_y,double T_NEx_src,double T_NEy_src,double T_NEx_dst,double T_NEy_dst,double azimuth_src,double azimuth_dst)
{
	azimuth_src = azimuth_src*PI/180;
	azimuth_dst = azimuth_dst*PI/180;
	double T_NEx = T_NEx_dst - T_NEx_src;
	double T_NEy = T_NEy_dst - T_NEy_src;
	double Tx = 1500;
	double Ty = 1000;
	double UVx = src_x-Tx;
	double UVy = src_y-Ty;
	double NEx = cos(azimuth_src)*UVx + sin(azimuth_src)*UVy;
	double NEy = -sin(azimuth_src)*UVx + cos(azimuth_src)*UVy;
	double NEx_ = NEx - T_NEx*50;
	double NEy_ = NEy - T_NEy*50;
	double UVx_ = cos(azimuth_dst)*NEx_ - sin(azimuth_dst)*NEy_;
	double UVy_ = +sin(azimuth_dst)*NEx_ + cos(azimuth_dst)*NEy_;
	dst_x = (UVx_ + Tx);
	dst_y = (UVy_ + Ty);
}

void EstopWatchDog()
{
	if(IsEstop==true)	
		WatchDogcount++;
	if(WatchDogcount>1000000)
		WatchDogcount = 1000000;	
	if(WatchDogcount>WatchDogamount)
	{
		IsEstop = false;
		CrossDetFlag = false;					
	}	
	if(PassengerDetectednum==1&&ManDetFlag)
	{
		WatchDogcount4++;
		PassengerDetectednumDel = 1;	
	}	
	else
	{
		PassengerDetectednumDel = 0;	
		WatchDogcount4=0;	
	}	
	if(WatchDogcount4>10000)
		WatchDogcount4 = 10000;
	if(WatchDogcount4>WatchDogamount4)
	{
		PassengerDetectednumDel = 0;
		ManDetFlag = false;				
	}
}

void display()
{
	for(int i=0;i<300;i++)
	{
		cvLine(PlanMapShow,cvPoint(Gis_Lane[i].x,(3000-Gis_Lane[i].y)),cvPoint(Gis_Lane[i+1].x,(3000-Gis_Lane[i+1].y)),CV_RGB(255,0,255),10);
		//cvLine(PlanMapShow,cvPoint(Gis_LaneBefore[i].x,(3000-Gis_LaneBefore[i].y)),cvPoint(Gis_LaneBefore[i+1].x,(3000-Gis_LaneBefore[i+1].y)),CV_RGB(0,255,255),10);
	}
	
	int objects_num = ibeo_objects.Object.size();
	for (int i = 0; i < objects_num; i++) {
		int y_3000 = 1000.0 + 0.01 * ibeo_objects.Object[i].ReferencePoint.x / LOCAL_COOR_RESOLUTION;
		int x_3000 = 1500.0 - 0.01 * ibeo_objects.Object[i].ReferencePoint.y / LOCAL_COOR_RESOLUTION;
		cvCircle(PlanMapShow, cvPoint(x_3000, 3000 - y_3000), 30, CV_RGB(0, 255, 255), -1);
	}

	//Draw two Road Edges
	/*for(int i=0;i<300;i++)
	{
		cvLine(PlanMapShow,cvPoint(roadpts1_lane[i].x,(3000-roadpts1_lane[i].y)),cvPoint(roadpts1_lane[i+1].x,(3000-roadpts1_lane[i+1].y)),CV_RGB(0,0,0),20);
		cvLine(PlanMapShow,cvPoint(roadpts2_lane[i].x,(3000-roadpts2_lane[i].y)),cvPoint(roadpts2_lane[i+1].x,(3000.0-roadpts2_lane[i+1].y)),CV_RGB(0,0,0),20);
		//Drwa the right line
		cvLine(PlanMapShow,cvPoint(roadpts_lane[i].x,(3000-roadpts_lane[i].y)),cvPoint(roadpts_lane[i+1].x,(3000.0-roadpts_lane[i+1].y)),CV_RGB(0,0,0),2);
	}*/
	
	sprintf(strtemp,"cross_dist: %f",cross_dist);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,300),&font,CV_RGB(0,255,0));
	//sprintf(strtemp,"crossdistance: %f",crossdistance);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(50,300),&font,CV_RGB(255,0,0));
	if (endlinedistance < 99999.9) {
		sprintf(strtemp,"endlinedistance: %.3lf",endlinedistance / 50.0);
			cvPutText(PlanMapShow,strtemp,cvPoint(50,500),&font,CV_RGB(255,0,0));
	} else {
		sprintf(strtemp,"endlinedistance: DBL_MAX");
			cvPutText(PlanMapShow,strtemp,cvPoint(50,500),&font,CV_RGB(255,0,0));
	}
	//sprintf(strtemp,"strategemodelast: %d",strategemodelast);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(2200,100),&font,CV_RGB(0,255,0));
	sprintf(strtemp,"strategemode: %d",strategemode);
		cvPutText(PlanMapShow,strtemp,cvPoint(2200,500),&font,CV_RGB(255,0,0));
	sprintf(strtemp,"match_state: %d",match_state);
		cvPutText(PlanMapShow,strtemp,cvPoint(2200,700),&font,CV_RGB(0,255,0));
	objects_num = ibeo_objects.Object.size();
	sprintf(strtemp, "objects_num: %d", objects_num);
		cvPutText(PlanMapShow, strtemp, cvPoint(2200,900), &font, CV_RGB(255,0,0));
	//sprintf(strtemp,"Gis_SelectLanenum: %d",Gis_SelectLanenum);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(2200,900),&font,CV_RGB(255,0,0));
	//sprintf(strtemp,"variance: %f",variance);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(50,700),&font,CV_RGB(0,255,0));
	//sprintf(strtemp,"WatchDogcount: %ld",WatchDogcount);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(2200,1100),&font,CV_RGB(255,0,0));
	//sprintf(strtemp,"WatchDogcount4: %ld",WatchDogcount4);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(2200,1200),&font,CV_RGB(255,0,0));
	//sprintf(strtemp,"blockpercent: %f",blockpercent);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(50,1100),&font,CV_RGB(255,0,0));
	if (g_distancefront < 99999.999) {
		sprintf(strtemp,"g_distancefront: %.3lfm", g_distancefront*0.02);
			cvPutText(PlanMapShow,strtemp,cvPoint(50,900),&font,CV_RGB(255,0,0));
	} else {
		sprintf(strtemp,"g_distancefront: MAX");
			cvPutText(PlanMapShow,strtemp,cvPoint(50,900),&font,CV_RGB(255,0,0));
	}
	sprintf(strtemp, "Cur_EV_Speed: %.3lfkm/h", Cur_EV_Speed);
		cvPutText(PlanMapShow, strtemp, cvPoint(50,1100), &font, CV_RGB(255,0,0));
	sprintf(strtemp, "force_move: %d", force_move);
		cvPutText(PlanMapShow, strtemp, cvPoint(50,1300), &font, CV_RGB(255,0,0));
	sprintf(strtemp, "estop_3_cnt: %.1lfs", estop_3_cnt/10.0);
		cvPutText(PlanMapShow, strtemp, cvPoint(50,1500), &font, CV_RGB(255,0,0));
	//sprintf(strtemp,"blockpercent2: %f",blockpercent2);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(50,1200),&font,CV_RGB(255,0,0));

	sprintf(strtemp,"lanechangespeed-1_0: %d",lanechangespeed);
		cvPutText(PlanMapShow,strtemp,cvPoint(1900,2400),&font,CV_RGB(0,255,0));
	if(strategemode==1)
	{
		sprintf(strtemp,"DistancecostMax-1_1: %f",DistancecostMax);
			cvPutText(PlanMapShow,strtemp,cvPoint(1900,2500),&font,CV_RGB(255,0,0));
	}
	else
	{
		sprintf(strtemp,"DistancecostMax-1_1: %f",DistancecostMax2);
			cvPutText(PlanMapShow,strtemp,cvPoint(1900,2500),&font,CV_RGB(255,0,0));
	}
	
	/*sprintf(strtemp,"curlanedischage-1_0: %f",curlanedischage);
		cvPutText(PlanMapShow,strtemp,cvPoint(1900,2600),&font,CV_RGB(0,255,0));
	sprintf(strtemp,"linecostdistance-1_0: %f",linecostdistance);
		cvPutText(PlanMapShow,strtemp,cvPoint(1900,2700),&font,CV_RGB(255,0,0));
	sprintf(strtemp,"changelanedueper: %f",changelanedueper);
		cvPutText(PlanMapShow,strtemp,cvPoint(1900,2800),&font,CV_RGB(0,255,0));*/

	sprintf(strtemp,"is_in_fee: %d", is_in_fee);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,2100),&font,CV_RGB(0,255,0));
	sprintf(strtemp,"islight: %d", islight);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,2200),&font,CV_RGB(0,255,0));
	sprintf(strtemp,"Gis_TurnDirection: %d",Gis_TurnDirection);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,2300),&font,CV_RGB(0,255,0));
	sprintf(strtemp,"forwd: %d",forwd);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,2400),&font,CV_RGB(255,0,0));
	sprintf(strtemp,"lturn: %d",lturn);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,2500),&font,CV_RGB(0,255,0));
	sprintf(strtemp,"rturn: %d",rturn);
		cvPutText(PlanMapShow,strtemp,cvPoint(50,2600),&font,CV_RGB(255,0,0));
	//sprintf(strtemp,"moveobs_num: %d",moveobs_num);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(50,2700),&font,CV_RGB(0,255,0));
	//sprintf(strtemp,"PassengerDetectednum: %d",PassengerDetectednum);
	//	cvPutText(PlanMapShow,strtemp,cvPoint(50,2800),&font,CV_RGB(0,255,0));	
	switch(strategemode)
	{
	case 1:
		if(Laneuseful)
		{
			for(int i=0;i<(pathwaynum-1);i++)
			{
				if(i==SelectPathIndex)
				{
					for(int j=0;j<300;j++)
					{
						cvLine(PlanMapShow,cvPoint(traffic_lanePro[i][j].x,(3000-traffic_lanePro[i][j].y)),cvPoint(traffic_lanePro[i][j+1].x,(3000-traffic_lanePro[i][j+1].y)),CV_RGB(0,255,0),10);
					}
				}
				else
				{
					for(int j=0;j<300;j++)
					{
						cvLine(PlanMapShow,cvPoint(traffic_lanePro[i][j].x,(3000-traffic_lanePro[i][j].y)),cvPoint(traffic_lanePro[i][j+1].x,(3000-traffic_lanePro[i][j+1].y)),CV_RGB(0,255,255),2);
					}
				}
			}
		}
		else
		{
			for(int i=0;i<(pathwaynum-1);i++)
			{
				if(i==SelectPathIndex)
				{
					for(int j=0;j<300;j++)
					{
						cvLine(PlanMapShow,cvPoint(traffic_lanePro[i][j].x,(3000-traffic_lanePro[i][j].y)),cvPoint(traffic_lanePro[i][j+1].x,(3000-traffic_lanePro[i][j+1].y)),CV_RGB(0,255,0),10);
					}
				}
				else
				{
					for(int j=0;j<300;j++)
					{
						cvLine(PlanMapShow,cvPoint(traffic_lanePro[i][j].x,(3000-traffic_lanePro[i][j].y)),cvPoint(traffic_lanePro[i][j+1].x,(3000-traffic_lanePro[i][j+1].y)),CV_RGB(0,255,255),2);
					}
				}
			}		
			for(int i=0;i<planlanenumber;i++)
			{
				if(i==SelectPlanPathIndex)
				{
					for(int j=0;j<300;j++)
					{
						cvLine(PlanMapShow,cvPoint(plines[i][j].x,(3000-plines[i][j].y)),cvPoint(plines[i][j+1].x,(3000-plines[i][j+1].y)),CV_RGB(0,255,255),2);
					}
				}
				else
				{
					for(int j=0;j<300;j++)
					{
						cvLine(PlanMapShow,cvPoint(plines[i][j].x,(3000-plines[i][j].y)),cvPoint(plines[i][j+1].x,(3000-plines[i][j+1].y)),CV_RGB(0,255,0),2);
					}
				}
			}
		}
	break;
	case 2:
		for(int i=0;i<planlanenumber;i++)
		{
			if(i==SelectPlanPathIndex)
			{
				for(int j=0;j<300;j++)
				{
					cvLine(PlanMapShow,cvPoint(plines[i][j].x,(3000-plines[i][j].y)),cvPoint(plines[i][j+1].x,(3000-plines[i][j+1].y)),CV_RGB(0,255,255),10);
				}
			}
			else
			{
				for(int j=0;j<300;j++)
				{
					cvLine(PlanMapShow,cvPoint(plines[i][j].x,(3000-plines[i][j].y)),cvPoint(plines[i][j+1].x,(3000-plines[i][j+1].y)),CV_RGB(0,255,0),2);
				}
			}
		}
	break;
	case 3:
		//for(int i=0;i<planlanenumber;i++)
		for(int i=planst;i<planed;i++)
		{
			if(i==SelectPlanPathIndex)
			{
				for(int j=0;j<300;j++)
				{
					cvLine(PlanMapShow,cvPoint(plines[i][j].x,(3000-plines[i][j].y)),cvPoint(plines[i][j+1].x,(3000-plines[i][j+1].y)),CV_RGB(0,255,255),10);
				}
			}
			else
			{
				for(int j=0;j<300;j++)
				{
					cvLine(PlanMapShow,cvPoint(plines[i][j].x,(3000-plines[i][j].y)),cvPoint(plines[i][j+1].x,(3000-plines[i][j+1].y)),CV_RGB(0,255,0),2);
				}
			}
		}
	break;
	case 4:
	break;
	case 5:
	break;
	}
	for(int j=0;j<300;j++)
	{
		cvLine(PlanMapShow,cvPoint(roadpts_Sent[j].x,(3000-roadpts_Sent[j].y)),cvPoint(roadpts_Sent[j+1].x,(3000-roadpts_Sent[j+1].y)),CV_RGB(255,0,0),20);
		//cvCircle(PlanMapShow, cvPoint(roadpts_Sent[j].x, 3000-roadpts_Sent[j].y), 20, CV_RGB(255, 0, 0), -1);
	}
	if(IsEstop||IsEstop2||IsEstop3||PassengerDetectednumDel==1)
	{		
		if(IsEstop&&(!IsEstop2)&&(!IsEstop3))
			cvPutText(PlanMapShow,"Cross-EStop!",cvPoint(2200,300),&font,CV_RGB(255,0,0));
		else if(IsEstop2&&(!IsEstop)&&(!IsEstop3))
			cvPutText(PlanMapShow,"End-EStop!",cvPoint(2200,300),&font,CV_RGB(255,0,0));
		else if(IsEstop3&&(!IsEstop2)&&(!IsEstop))
			cvPutText(PlanMapShow,"Follow-EStop!",cvPoint(2200,300),&font,CV_RGB(255,0,0));
		else if(PassengerDetectednumDel==1)
			cvPutText(PlanMapShow,"Pass-EStop!",cvPoint(2200,300),&font,CV_RGB(255,0,0));
		else
			cvPutText(PlanMapShow,"Muti-EStop!",cvPoint(2200,300),&font,CV_RGB(255,0,0));	
	} else {
		cvPutText(PlanMapShow,"Running!",cvPoint(2200,300),&font,CV_RGB(255,0,0));
	}
}

void transferDecisionResult(const std_msgs::Int8::ConstPtr &decision_result_ptr) {
	decision_result = *decision_result_ptr;
	is_decision_result_rcv = true;
}

void transferScanInfoV2(const in2_msgs::ScanInfoV2::ConstPtr &scan_info_v2_ptr) {

	for (int i = 0; i < 720; i++) {
		sqToMeter(scan_info_v2_ptr->ScanInfoX[i], scan_info_v2_ptr->ScanInfoY[i],
			fused_scan[i].x, fused_scan[i].y);
	}
}

void transferVehicleInfo(const in2_msgs::VehicleInfo::ConstPtr &vehicle_info_ptr) {

	vehicle_info = *vehicle_info_ptr;
	is_vehicle_info_rcv = true;
	Cur_EV_Odometer = vehicle_info.Mileage * 100.0; // cm
	Cur_EV_Speed = vehicle_info.Speed; // km/h
	Cur_EV_Orient = vehicle_info.SteeringWheel; // degree
	// fprintf(stdout,"Cur_EV_Speed: %fkm/h\n",Cur_EV_Speed);
	double speedtemp;
	speedtemp = Cur_EV_Speed;
	//double max_speed_temp = 40.0;
	if(speedtemp>max_speed_temp)
		speedtemp = max_speed_temp;
	if(speedtemp<2.0)
		speedtemp = 2.0;
	
	lanechangespeed = (int)((max_speed_temp-speedtemp)/(max_speed_temp-2.0)*((double)lanechangespeedMax-(double)lanechangespeedMin) + (double)lanechangespeedMin);
        lanechangespeed2 = (int)((max_speed_temp-speedtemp)/(max_speed_temp-2.0)*((double)lanechangespeed2Max-(double)lanechangespeed2Min) + (double)lanechangespeed2Min);

	DistancecostMax = (speedtemp - 2.0)*(speedtemp - 2.0)/((max_speed_temp-2.0)*(max_speed_temp-2.0))*(DistancecostMax_Max-DistancecostMax_Min) + DistancecostMax_Min;	
	DistancecostMax2 = (speedtemp - 2.0)*(speedtemp - 2.0)/((max_speed_temp-2.0)*(max_speed_temp-2.0))*(DistancecostMax2_Max-DistancecostMax2_Min) + DistancecostMax2_Min;	

	curlanedischage = (max_speed_temp-speedtemp)/(max_speed_temp-2.0)*(curlanedischageMax-curlanedischageMin) + curlanedischageMin;

	linecostdistance = (max_speed_temp-speedtemp)/(max_speed_temp-2.0)*(linecostdistanceMax-linecostdistanceMin) + linecostdistanceMin;
	
	if(lanechangespeed>lanechangespeedMax)
		lanechangespeed = lanechangespeedMax;
	if(lanechangespeed<lanechangespeedMin)
		lanechangespeed = lanechangespeedMin;
	if(lanechangespeed2>lanechangespeed2Max)
		lanechangespeed2 = lanechangespeed2Max;
	if(lanechangespeed2<lanechangespeed2Min)
		lanechangespeed2 = lanechangespeed2Min;

	if(DistancecostMax>DistancecostMax_Max)
		DistancecostMax = DistancecostMax_Max;
	if(DistancecostMax<DistancecostMax_Min)
		DistancecostMax = DistancecostMax_Min;
	if(DistancecostMax2>DistancecostMax2_Max)
		DistancecostMax2 = DistancecostMax2_Max;
	if(DistancecostMax2<DistancecostMax2_Min)
		DistancecostMax2 = DistancecostMax2_Min;
	if(curlanedischage>curlanedischageMax)
		curlanedischage = curlanedischageMax;
	if(curlanedischage<curlanedischageMin)
		curlanedischage = curlanedischageMin;
	if(linecostdistance>linecostdistanceMax)
		linecostdistance = linecostdistanceMax;
	if(linecostdistance<linecostdistanceMin)
		linecostdistance = linecostdistanceMin;
}

void transferTrafficLightsInfo(const in2_msgs::TrafficLights::ConstPtr &traffic_lights_ptr) {

	traffic_lights_dog = 0;
	
	traffic_lights = *traffic_lights_ptr;
	is_traffic_lights_rcv = true;
	
	lturn = (traffic_lights.TurnLeft.Color == traffic_lights.TurnLeft.COLOR_RED) ? 0 : 1;
	forwd = (traffic_lights.Forward.Color == traffic_lights.TurnLeft.COLOR_RED) ? 0 : 1;
	rturn = (traffic_lights.TurnRight.Color == traffic_lights.TurnLeft.COLOR_RED) ? 0 : 1;
}

void trafficLightsDogCallback(const ros::TimerEvent &event) {
	traffic_lights_dog++;
}

void transferLaneMarks(const in2_msgs::LaneMarks::ConstPtr &lane_marks_ptr) {
	// TODO: transfer dis_to_stop_line
	is_lane_marks_rcv = true;
}

void transferIbeoObjects(const in2_msgs::IbeoObjects::ConstPtr &ibeo_objects_ptr) {
	ibeo_objects = *ibeo_objects_ptr;
	is_ibeo_objects_rcv = true;
	//cout << "I get ibeo!" << endl;
}

void transferFarObs(const std_msgs::Float64::ConstPtr &far_obs_ptr) {
	dis_to_far_obs = far_obs_ptr->data;
	is_far_obs_rcv = true;
}

void transferFarObsXy(const in2_msgs::Point2D::ConstPtr &xy_obs_ptr) {
	obs_xy = *xy_obs_ptr;
}

void init() {

	// ****** generate a straight gis line manually for debug ****** //
	int pts_num = 600;
	for (int i = 0; i < pts_num; i++) {
	
		Gis_Lane[i].x = 2.5 * LOCAL_COOR_WIDTH / 5;
		Gis_Lane[i].y = i * ((2 * LOCAL_COOR_LENGTH-1) / (pts_num-1));
	}
}

void sendEmptyDec() {
	dec_msg.SpeedLimit = 0.0;
	dec_msg.Mode = dec_msg.STOP_MODE;
	dec_msg.TurnLight = dec_msg.TURN_LIGHT_NONE;
	dec_msg.Horn = dec_msg.HORN_OFF;
	dec_info_pub_.publish(dec_msg);
}

/****************************************************************************************************/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "plan_node_3");
	ros::NodeHandle nh_;
	
	init();
	
	image_transport::ImageTransport it_obs(nh_);
	image_transport::ImageTransport it_lane(nh_);
	image_transport::Subscriber image_sub_obs;
	image_transport::Subscriber image_sub_lane;
	image_sub_obs = it_obs.subscribe("obs_cost_map", 1, &imageObsConvert);
	image_sub_lane = it_lane.subscribe("lane_cost_map", 1, &imageLaneConvert);
	ins_sub_ = nh_.subscribe("InsInfo",1,&transferINS);
	//GIS_sub_ = nh_.subscribe("GISudp",10,&transferGIS,ros::TransportHints().tcpNoDelay(true)); // earth frame
	gisRect_sub_ = nh_.subscribe("gis_info",10,&transferGISRect,ros::TransportHints().tcpNoDelay(true)); // earth frame
	//TREK_sub_ = nh_.subscribe("TREK",10,&transferTREK,ros::TransportHints().tcpNoDelay(true));
	vehicle_info_sub_ = nh_.subscribe("vehicle_info", 10, &transferVehicleInfo, ros::TransportHints().tcpNoDelay(true));
	//roadedge_sub_ = nh_.subscribe("RoadEdge",10,&transferRoadEdge,ros::TransportHints().tcpNoDelay(true));
	//lanemodelinfo_sub_ = nh_.subscribe("LaneModelInfo",10,&transferLaneModelInfo,ros::TransportHints().tcpNoDelay(true));
	//scaninfo_sub_ = nh_.subscribe("velodyne_scan",10,&transferScanInfo,ros::TransportHints().tcpNoDelay(true));
	scan_info_v2_sub_ = nh_.subscribe("ScanInfoV2", 10, &transferScanInfoV2,ros::TransportHints().tcpNoDelay(true));
	//ParkingFinish_sub_ = nh_.subscribe("ParkingFinished",10,&transferParkingFinish,ros::TransportHints().tcpNoDelay(true));
	//PassengerDetected_sub_ = nh_.subscribe("PassengerDtected",10,&PassengerDetectedDel,ros::TransportHints().tcpNoDelay(true));
	//astar_publisher_ = nh_.advertise<in2_msgs::UdpGeneral>("astar", 10);
	tfsn_sub_ = nh_.subscribe("TFSN",10,&transferTFSN,ros::TransportHints().tcpNoDelay(true));
	//tfl_sub_ = nh_.subscribe("TFL",10,&transferTFL,ros::TransportHints().tcpNoDelay(true));
	movtgtnum_sub_ = nh_.subscribe("MovTgtNum",10,&transferMovTgtNum,ros::TransportHints().tcpNoDelay(true));
	decision_sub_ = nh_.subscribe("/decision_result", 10, &transferDecisionResult, ros::TransportHints().tcpNoDelay(true));
	traffic_lights_sub_ = nh_.subscribe("TrafficLightsInfo", 10, &transferTrafficLightsInfo, ros::TransportHints().tcpNoDelay(true));
	lane_marks_sub = nh_.subscribe("LaneMarks", 10, &transferLaneMarks, ros::TransportHints().tcpNoDelay(true));
	ibeo_objects_sub_ = nh_.subscribe("objVehicle", 10, &transferIbeoObjects, ros::TransportHints().tcpNoDelay(true));
	dis_to_far_obs_sub_ = nh_.subscribe("dis_to_farobj", 10, &transferFarObs, ros::TransportHints().tcpNoDelay(true));
	xy_of_far_obs_sub_ = nh_.subscribe("xy_of_farobj", 10, &transferFarObsXy, ros::TransportHints().tcpNoDelay(true));
	
	tfsntfl_pub_ = nh_.advertise<std_msgs::Int16>("TFSN_TFL", 10);
	parkingcontrol_pub_ = nh_.advertise<std_msgs::Int16>("ParkingControl", 10);
	gisback_pub_ = nh_.advertise<std_msgs::Int16>("GIS_back", 10);
	controlpanel_sub_ = nh_.subscribe("ControlPanelInfo",10,&transferControlPanel,ros::TransportHints().tcpNoDelay(true));
	//tcp_publisher_ = nh_.advertise<in2_msgs::TcpGeneral>("FusDec", 10); // earth frame
	dec_info_pub_ = nh_.advertise<in2_msgs::DecInfo>("dec_info", 10); // earth frame
	cvStartWindowThread();
	//cv::namedWindow(OPENCV_WINDOW,0);
	
	dec_msg.RoadPoints.resize(301);
	
	lane_mat = Mat::zeros(300, 300, CV_8UC1);
	img_show = Mat::zeros(3000, 3000, CV_8UC3);
	
	ros::Timer r_tmr = nh_.createTimer(ros::Duration(0.1), trafficLightsDogCallback);
	
	
	//ros::Rate r(100);
	while(ros::ok())
	{
		if(PlanMapShow!=NULL)
		{
			cvResize(PlanMapShow,PlanMapShow_S);
			cv::namedWindow("PLANMAPSHOW",0);
			cvShowImage("PLANMAPSHOW",PlanMapShow_S);
			//cv::namedWindow("PLANPROCESS",0);
			//cvShowImage("PLANPROCESS",MoPlanMap);
			cvWaitKey(3);
		}
		else {
			cout << "PlanMapShow == NULL" << endl;
			sendEmptyDec();
			cvWaitKey(100);
		}
		ros::spinOnce();
		//r.sleep();
	}
	ros::shutdown(); 
	return 0;
}

CvPoint convENUto3000( double x, double y, double x0, double y0, double azimuth )
{
	double dx = x - x0;
	double dy = y - y0;
	double cosa = cos(azimuth * PI / 180.0);
	double sina = sin(azimuth * PI / 180.0);
	double resultX = dx * cosa - dy * sina;
	double resultY = dx * sina + dy * cosa;
	int finalX = (int)(resultX * 50.0 + 1500.0);
	int finalY = (int)(resultY * 50.0 + 1000.0);
	/*if(finalX < 0) finalX = 0;
	else if(finalX > 2999) finalX = 2999;
	if(finalY < 0) finalY = 0;
	else if(finalY > 2999) finalY = 2999;*/
	CvPoint pt;
	pt.x = finalX;
	pt.y = finalY;
	return pt;
}

CvPoint2D64f conv3000toENU(int x, int y, double x0, double y0, double azimuth )
{
	double resultX = (double(x) - 1500.0)/50.0;
	double resultY = (double(y) - 1000.0)/50.0;
	double cosa = cos(azimuth * PI / 180.0);
	double sina = sin(azimuth * PI / 180.0);
	double dx = resultX * cosa + resultY * sina;
	double dy = -resultX * sina + resultY * cosa;
	double finalX = dx + x0;
	double finalY = dy + y0;
	CvPoint2D64f pt;
	pt.x = finalX;
	pt.y = finalY;
	return pt;
}

void sqToMeter(int a, int b, float &x, float &y) {

	y = -(a - 1500) * 0.02;
	x =  (b - 1000) * 0.02;
}

void meterToSq(double x, double y, int &a, int &b) {

	a = 1500.0 - y * 50.0;
	b = 1000.0 + x * 50.0;
}
