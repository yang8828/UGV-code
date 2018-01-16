#ifndef FILTER_H
#define FILTER_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <in2_msgs/ScanInfoV2.h>

using namespace cv;
using namespace std;

int obsCostMapFilter(Mat CurFrm, Mat LastFrm, Mat &out, Point3d CurIns, Point3d LastIns);
void doRT(int src_x, int src_y, int &dst_x, int &dst_y, double T_NEx, double T_NEy, double azimuth_src, double azimuth_dst);
Point convMapTo3000(Point src_pt, int MapSize);
Point conv3000ToMap(Point src_pt, int MapSize);
CvPoint convENUto3000(double x, double y, double x0, double y0, double azimuth);
CvPoint2D64f conv3000toENU(int x, int y, double x0, double y0, double azimuth);

#endif

