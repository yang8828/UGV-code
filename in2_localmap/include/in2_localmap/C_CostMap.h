/*
 * C_CostMap.h
 *
 *  Created on: 2013-10-20
 *      Author: greensky
 */

#ifndef C_COSTMAP_H_
#define C_COSTMAP_H_

#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <in2_localmap/C_Scan.h>

using namespace cv;

#define COSTMAP_SIZE 300

class C_CostMap
{
	long timeStamp;
	Mat CostMap;

	Point points_list[1][720];
public:
	C_CostMap()
	{
		timeStamp = 0;
		CostMap = Mat::ones(COSTMAP_SIZE,COSTMAP_SIZE,CV_8UC4);
		CostMap = Scalar(180,100,100,0);
	}
	void SetCostMat(C_ScanFrame ScanFrame)
	{
		timeStamp = ScanFrame.timestamp;
		CostMap = Mat::ones(COSTMAP_SIZE,COSTMAP_SIZE,CV_8UC4);
		CostMap = Scalar(180,100,100,0); // Channel 3 must be 255-255!!!

		double temp_x;
		double temp_y;

		int point_num_list[1];
		int counter = 0;
		for(int i=0;i<720;i++)
		{
			if(ScanFrame.x[i] == -1 || ScanFrame.y[i] == -1)
			{
				continue;
			}
			temp_x = (double)ScanFrame.x[i];
			temp_y = (double)ScanFrame.y[i];
			temp_x = temp_x /3000.0 * ((double)COSTMAP_SIZE);
			temp_y = temp_y /3000.0 * ((double)COSTMAP_SIZE);
			points_list[0][counter].x = (int)floor(temp_x);
			points_list[0][counter].y = (int)floor(temp_y);
			counter++;
		}
		const Point* ppt[1] = {points_list[0]};
		point_num_list[0] = counter;
		fillPoly(CostMap,ppt,point_num_list,1,Scalar(255,255,255,255)); // Channel 3 must be 255-0!!!
		//Mat singleChannel[4];
		//split(CostMap,singleChannel);
		//erode(singleChannel[3],singleChannel[4],Mat(),Point(-1, -1),10);
#if VERSION_SUB_URBAN
		dilate(CostMap,CostMap,Mat(),Point(-1, -1),2);
#endif
#if VERSION_URBAN
		dilate(CostMap,CostMap,Mat(),Point(-1, -1),1);
#endif
		//cv::dilate(LidarMask1m,LidarMask2m,kernel,cv::Point(-1,-1),1);
		//cv::erode(LidarMask,LidarMask,kernel,cv::Point(12,12),2);
	}
	void DrawMap()
	{
		Mat temp;
		temp.create(3000,3000,CV_8UC3);
		flip(CostMap,temp,0);
	}
	void DrawMap(Mat &image)
	{
		if(image.rows == 3000 && image.cols == 3000)
		{
			Mat temp;
			temp.create(3000,3000,CV_8UC3);
			flip(CostMap,temp,0);
			resize(temp,image,Size(3000,3000));
			//image = temp;
		}
		else
			return;
	}
	double getCostAt(int x,int y)
	//return 0~1, input in 3K coordinate system
	{
		double temp_x = (double)x;
		double temp_y = (double)y;
		temp_x = temp_x /3000.0 * ((double)COSTMAP_SIZE);
		temp_y = temp_y /3000.0 * ((double)COSTMAP_SIZE);
		x = (int)floor(temp_x);
		y = (int)floor(temp_y);

		if(x<0 || x>=COSTMAP_SIZE || y<0 || y >=COSTMAP_SIZE)
		{
			return 1.0;
		}
		else
		{
			Vec4b v;
			v = CostMap.at<Vec4b>(y,x);
			uchar c;
			c = v[3];//Store Cost in channel 3
			int temp = 255 - (int)c;
			double cost = (double)temp;
			cost = cost/255.0;
			return cost;
		}
	}
};

#endif /* C_COSTMAP_H_ */
