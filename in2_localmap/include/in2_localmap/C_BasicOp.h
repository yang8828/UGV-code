/*
 * C_BasicOp.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_BASICOP_H_
#define C_BASICOP_H_

#include <in2_localmap/C_Param.h>
#include "stdio.h"
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
using namespace cv;

bool regionGrowing(double list[],int len,int i,int &edge_1,int &edge_2,double reg_maxdist);

Mat regionGrowing(Mat I,int x,int y,double reg_maxdist);

double normof(double x,double y);

double vectorangle3K(double x,double y);
double vectorangle3K(int x,int y);

double angle3KDiff(double angle3K_1,double angle3K_2);

double byangle3KDiff(double byangle3K_1,double byangle3K_2);

double azimuthDiff(double azimuth_1,double azimuth_2);

double lineAngleDiff(double azimuth_1,double azimuth_2);

double interpolation(double t1,double t2,double y1,double y2,double t);

double interpolation_for_periodic(double t1,double t2,double y1,double y2,double t);
void doRT(int src_x,int src_y,int &dst_x,int &dst_y,double T_NEx,double T_NEy,double azimuth_src,double azimuth_dst);

void doRT2(double src_x,double src_y,double &dst_x,double &dst_y,double T_NEx_src,double T_NEy_src,double T_NEx_dst,double T_NEy_dst,double azimuth_src,double azimuth_dst);
void getPerpendicularFoot(double a,double b,double c,double src_x,double src_y,double &foot_x,double &foot_y);
void getPerpendicularFoot(double a,double b,double c,int src_x,int src_y,int &foot_x,int &foot_y);
#endif /* C_BASICOP_H_ */
