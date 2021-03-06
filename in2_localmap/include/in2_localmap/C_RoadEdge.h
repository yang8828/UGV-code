#ifndef C_ROADEDGE_H
#define C_ROADEDGE_H

#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <in2_msgs/InsInfo.h>
#include <in2_msgs/RoadEdge.h>
#include <in2_msgs/LaneInfoV2.h>
#include <in2_msgs/ScanInfoV2.h>
#include <in2_msgs/UdpGeneral.h>
#include <in2_msgs/UdpGeneralShort.h>
#include <math.h>

using namespace cv;

class C_RoadEdgeFrame
{
public:
	long timestamp;

	double Angle;
	double leftdis;
	double rightdis;
	double leftb;
	double rightb;

	double AngleLeft;
	double AngleRight;
	double left_fwd_dist;
	double right_fwd_dist;
	double left_fwd_b;
	double right_fwd_b;

	double k_edge;
	double left_intce;
	double right_intce;
public:
	C_RoadEdgeFrame()
	{
		timestamp = 0;

		Angle = 0;
		leftdis = 0;
		rightdis = 0;
		leftb = 0;
		rightb = 0;

		AngleLeft = 0;
		AngleRight = 0;
		left_fwd_dist = 0;
		right_fwd_dist = 0;
		left_fwd_b = 0;
		right_fwd_b = 0;

		k_edge = 0;
		left_intce = 0;
		right_intce = 0;
	}

	long setWithSample(const in2_msgs::RoadEdge::ConstPtr &roadedge_sub)
	{
		timestamp = 0;

		Angle = roadedge_sub->Angle;
		leftdis = roadedge_sub->leftdis;
		rightdis = roadedge_sub->rightdis;
		leftb = roadedge_sub->leftb;
		rightb = roadedge_sub->rightb;

		AngleLeft = roadedge_sub->AngleLeft;
		AngleRight = roadedge_sub->AngleRight;
		left_fwd_dist = roadedge_sub->left_fwd_dist;
		right_fwd_dist = roadedge_sub->right_fwd_dist;
		left_fwd_b = roadedge_sub->left_fwd_b;
		right_fwd_b = roadedge_sub->right_fwd_b;

		double tmpdd;
		k_edge = tan(Angle*3.14/180);
		tmpdd = 50 * leftdis * sqrt(k_edge*k_edge+1);
		left_intce = 1500 - 1000*k_edge - tmpdd;
		tmpdd = 50 * rightdis * sqrt(k_edge*k_edge+1);
		right_intce = 1500 - 1000*k_edge + tmpdd;

		return timestamp;
	}

	void drawRoadEdge(Mat &img, Scalar color = Scalar(0,0,0), int thickness = 15)
	{
		Point pt1,pt2;

		pt1 = Point(left_intce, 3000);
		pt2 = Point(k_edge*3000+left_intce, 0);
		line(img, pt1, pt2, color, thickness);

		pt1 = Point(right_intce, 3000);
		pt2 = Point(k_edge*3000+right_intce, 0);
		line(img, pt1, pt2, color, thickness);
	}
};

#endif
