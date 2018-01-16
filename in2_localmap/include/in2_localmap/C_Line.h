/*
 * C_Line.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_LINE_H_
#define C_LINE_H_

#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>


#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_Ins.h>

using namespace cv;

class C_LineSample
{
public:
	bool valid;
	long timestamp;
	int r;
	int theta;
	int type;
	int beliefe;
	int isCurve;
	int points_x[31];
	int points_y[31];
	bool withIns;
	C_InsFrame InsData;
	C_LineSample()
	{
		valid = false;
		timestamp = -1;
		r = 0;
		theta = 0;
		type  = 0;
		beliefe = 0;
		isCurve = -1;
		for (int i=0;i<31;i++)
		{
			points_x[i] = 0;
			points_y[i] = 0;
		}
		withIns = false;
	}
	void init()
	{
		valid = false;
		timestamp = -1;
		r = 0;
		theta = 0;
		type  = 0;
		beliefe = 0;
		isCurve = -1;
		for (int i=0;i<31;i++)
		{
			points_x[i] = 0;
			points_y[i] = 0;
		}
		withIns = false;
	}
	void copyfrom(C_LineSample src)
	{
		valid = src.valid;
		timestamp = src.timestamp;
		r = src.theta;
		theta = src.theta;
		type = src.type;
		beliefe = src.beliefe;
		isCurve = src.isCurve;
		for (int i=0;i<31;i++)
		{
			points_x[i] = src.points_x[i];
			points_y[i] = src.points_y[i];
		}
		withIns = src.withIns;
		InsData.copyfrom(src.InsData);
	}
	Mat getXYat(C_InsFrame dst_Ins)
	{
		if(type != 0 && withIns == true)
		{
			double x1 = (double) points_x[0];
			double y1 = (double) points_y[0];
			double x2 = (double) points_x[30];
			double y2 = (double) points_y[30];
			double dst_x1;
			double dst_y1;
			double dst_x2;
			double dst_y2;
			doRT2(x1,y1,dst_x1,dst_y1,InsData.position[0],InsData.position[1],dst_Ins.position[0],dst_Ins.position[1],InsData.attitude[2],dst_Ins.attitude[2]);
			doRT2(x2,y2,dst_x2,dst_y2,InsData.position[0],InsData.position[1],dst_Ins.position[0],dst_Ins.position[1],InsData.attitude[2],dst_Ins.attitude[2]);
			Mat XY =  (Mat_<double>(2,2)<<dst_x1,dst_y1,dst_x2,dst_y2);
			return XY;
		}
		else
		{
			Mat NoData;
			return NoData;
		}
	}
	void RTto(C_LineSample &dst,C_InsFrame dst_InsFrame)
	{
		dst.timestamp = dst_InsFrame.timestamp;
		dst.InsData.copyfrom(dst_InsFrame);
		dst.withIns = withIns;
		double TNE_x;
		double TNE_y;
		for(int i=0;i<31;i++)
		{
			TNE_x = dst_InsFrame.position[0] - InsData.position[0];
			TNE_y = dst_InsFrame.position[1] - InsData.position[1];
			doRT(points_x[i],points_y[i],dst.points_x[i],dst.points_y[i],TNE_x,TNE_y,InsData.attitude[2],dst_InsFrame.attitude[2]);
		}
	}
};

class C_LineSampleHis
{
public:
	bool withSample;
	C_LineSample line_his[MAX_LINE_HIS_LEN];
	int current_len;
	int age;
	double strength;
	double beliefe;
	C_LineSampleHis()
	{
		current_len = 0;
		age = 0;
		strength = 0;
		beliefe = 0;
	}
	void copyfrom(C_LineSampleHis src)
	{
		withSample = src.withSample;
		for (int i=0;i<MAX_LINE_HIS_LEN;i++)
		{
			line_his[i].copyfrom(src.line_his[i]);
		}
		current_len = src.current_len;
		age = src.age;
		strength = src.strength;
		beliefe = src.beliefe;
	}
	void init()
	{
		current_len = 0;
		age = 0;
		strength = 0.0;
		beliefe = 0.0;
		for(int i=0;i<MAX_LINE_HIS_LEN;i++)
			line_his[i].init();
	}
	void addNewFrame(C_LineSample newframe)
	{
		if(newframe.type != 0)
		{
			strength ++;
			beliefe += newframe.beliefe;
		}
		age++;
		if(age>MAX_LINE_HIS_LEN) age = MAX_LINE_HIS_LEN;

		if(line_his[MAX_LINE_HIS_LEN-1].type != 0)
		{
			strength --;
			beliefe -= line_his[MAX_LINE_HIS_LEN-1].beliefe;
		}

		for(int i=MAX_LINE_HIS_LEN-1;i>0;i--)
		{
			line_his[i].copyfrom(line_his[i-1]);
		}
		line_his[0].copyfrom(newframe);
		current_len ++;
		if (current_len > MAX_LINE_HIS_LEN)
			current_len = MAX_LINE_HIS_LEN;
	}
	Mat getXYat(C_InsFrame dst_Ins)
	{
		Mat m_list[MAX_LINE_HIS_LEN];
		int counter = 0;
		for (int i=0;i<current_len;i++)
		{
			if (line_his[i].valid == true)
			{
				m_list[counter] = line_his[i].getXYat(dst_Ins);
				counter++;
			}
		}
		Mat XY(counter * 2, 2, CV_64F);
		for (int i = 0; i < counter; i++)
		{
			m_list[i].copyTo(XY(Range(2*i, 2 * i + 2), Range(0, 2)));
		}
		return XY;
	}
};



#endif /* C_LINE_H_ */
