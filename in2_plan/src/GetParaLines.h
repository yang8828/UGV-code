#ifndef GETPARALINES_H
#define GETPARALINES_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <list>

using namespace std;
using namespace cv;

int getGisStEd(CvPoint *GISRectify, bool &GISIndxFlg, int &GISStrtIndx, int &GISEndIndx);
double calcTurnRadius(CvPoint2D64f P1, CvPoint2D64f P2, CvPoint2D64f P3);
cv::Point2d calcTwoMinTurnRadii(CvPoint* RoadPts);
int FindMinDisID(CvPoint2D64f* pts, int minID, int maxID, double centerX, double centerY);
int FindMinDisID(CvPoint* pts, int minID, int maxID, int centerX, int centerY);
int getParaLines(CvPoint cline[1000], int leftlen, int rightlen, CvPoint plines[500][1000], int &num, int &id, int &speedlimit, int &st, int &ed);
int getParaLinesV2(CvPoint cline[1000], int leftlen, int rightlen, CvPoint plines[100][1000], int &num, int &id, int &speedlimit);
int getParaLine(CvPoint cline[1000], CvPoint pt, CvPoint plines[3][1000]);
int getOnePara(CvPoint clineV2[1000], double dis, CvPoint *out, int MoveThre);

#endif
