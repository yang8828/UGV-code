#ifndef COSTER_H
#define COSTER_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <in2_msgs/ScanInfoV2.h>
#include <in2_msgs/LaneMarks.h>
//#include <in2_ins/InsInfo.h>

using namespace cv;
using namespace std;

#define   L     600
#define   gridsize     (60.0/L)

class Coster
{
	private:
		
	public:
		in2_msgs::ScanInfoV2 oScanInfo;
		in2_msgs::LaneMarks  oLaneMarks;
		Point3d Ins;
		Point3d CurIns;
		Point3d LastIns;
		vector<Point> DataPts3000;
		Mat LidarMask;
		Mat LidarMask1m;
		Mat LidarMask2m;
		Mat LidarMaskBlur;
		Mat LastLidarMaskBlur;
		Mat LidarMaskFilt;
		Mat LaneMaskFilt;
		
		void GenerateObsCostMap();
		void GenerateLaneCostMap();
		void Prepare();
		void Process();
		void Debug();
		void Fill();
		void Connect();
		void Dilate();
		void Gaussian();
		Coster();
		~Coster(){};
};

#endif

