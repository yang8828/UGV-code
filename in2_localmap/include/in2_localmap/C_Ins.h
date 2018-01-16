/*
 * C_Ins.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_INS_H_
#define C_INS_H_

#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_BasicOp.h>
#include <in2_msgs/InsInfo.h>
#include <in2_msgs/LaneInfoV2.h>
#include <in2_msgs/ScanInfoV2.h>
#include <in2_msgs/UdpGeneral.h>
#include <in2_msgs/UdpGeneralShort.h>

using namespace cv;

class C_InsFrame
{
public:
	//Valuables
	//Time Stamp
	long timestamp;
	//InsInfo
	double second;
	double longtitude;
	double latitude;
	double position[3];
	double attitude[3];
	double velocity[3];
	//Functions
	C_InsFrame()
	{
		second = 0;
		longtitude = 0;
		latitude = 0;
		for(int i=0;i<3;i++)
		{
			position[i] = 0;
			attitude[i] = 0;
			velocity[i] = 0;
		}
	}
	C_InsFrame(const int32_t _ms, const double s, const double lngt, const double lati, const geometry_msgs::Vector3_<std::allocator<void> > pos, const geometry_msgs::Vector3_<std::allocator<void> > att, const geometry_msgs::Vector3_<std::allocator<void> > vel)
	{
		timestamp = _ms;
		second = s;
		longtitude = lngt;
		latitude = lati;
		position[0] = pos.x;
		attitude[0] = att.x;
		velocity[0] = vel.x;
		position[1] = pos.y;
		attitude[1] = att.y;
		velocity[1] = vel.y;
		position[2] = pos.z;
		attitude[2] = att.z;
		velocity[2] = vel.z;
	}
	void copyfrom(C_InsFrame src)
	{
		timestamp = src.timestamp;
		second = src.second;
		longtitude = src.longtitude;
		latitude	= src.latitude;
		for (int i=0;i<3;i++)
		{
			position[i] = src.position[i];
			attitude[i] = src.attitude[i];
			velocity[i] = src.velocity[i];
		}
	}
	void PrintTimeStamp()
	{
		fprintf(stderr,"INS Frame:\t\t\tINS TS:\t%ld\n",timestamp);
	}
};

class C_InsFrameHis
{
public:
	C_InsFrame InsFrameList[INS_FRAME_HIS_LEN];//Sored by decreased order
	int current_len;

	C_InsFrameHis()
	{
		current_len = 0;
	}
	long addInsFrameHis(C_InsFrame newframe)
		// Add successful return true, Error return false;
	{
		for(int i=INS_FRAME_HIS_LEN-1;i>0;i--)
		{
			InsFrameList[i].copyfrom(InsFrameList[i-1]);
		}
		InsFrameList[0].copyfrom(newframe);
		current_len ++;
		if (current_len > INS_FRAME_HIS_LEN)
			current_len = INS_FRAME_HIS_LEN;
		if (current_len>=2)
		{
			if (InsFrameList[0].timestamp < InsFrameList[1].timestamp)
			{
				return -1;
			}
		}
		return newframe.timestamp;
	}
	void printHisTimeStamp()
	{
		for(int i=0;i<current_len;i++)
		{
			fprintf(stderr,"%ld ",InsFrameList[i].timestamp);
		}
		fprintf(stderr,"\n");
	}
	bool getMatchInsFrame(long dst_timestamp,C_InsFrame &MatchFrame)
		//return the time = TgtTimeStamp - MatchTimeStamp,
	{
		if (current_len == 0)
		{
			return false;
		}
		else // current_len >0
		{
			if(current_len == 1)
			{
				MatchFrame.copyfrom(InsFrameList[0]);
				return true;
			}
			else //current_len >1
			{
				int match_index = current_len;
				for (int i=0;i<current_len;i++)
				{
					if (dst_timestamp > InsFrameList[i].timestamp)
					{
						match_index = i;
						break;
					}
				}
				//Case 1: match the first one
				int another_match_index = -1;
				if (match_index == 0)
				{
					match_index = 0;
					another_match_index = 1;
				}
				else
				{
					//Case 2: match the last one
					if (match_index == current_len)
					{
						match_index = current_len -1;
						another_match_index = match_index - 1;
					}
					//Case 3: match the middle one
					else
					{
						//match_index  = match_index;
						another_match_index = match_index - 1;
					}
				}
				//insert value
				//t1>t2;
				double t1 = InsFrameList[match_index].timestamp;
				double t2 = InsFrameList[another_match_index].timestamp;
				//Time Stamp
				MatchFrame.timestamp = dst_timestamp;
				double t  = MatchFrame.timestamp;
				//InsInfo
				MatchFrame.second = interpolation(t1,t2,InsFrameList[match_index].second,InsFrameList[another_match_index].second,t);
				MatchFrame.longtitude = interpolation(t1,t2,InsFrameList[match_index].longtitude,InsFrameList[another_match_index].longtitude,t);
				MatchFrame.latitude = interpolation(t1,t2,InsFrameList[match_index].latitude,InsFrameList[another_match_index].latitude,t);
				for(int i=0;i<3;i++)
				{
					MatchFrame.position[i] = interpolation(t1,t2,InsFrameList[match_index].position[i],InsFrameList[another_match_index].position[i],t);
					MatchFrame.velocity[i] = interpolation(t1,t2,InsFrameList[match_index].velocity[i],InsFrameList[another_match_index].velocity[i],t);
				}
				for(int i=0;i<3;i++)
				{
					MatchFrame.attitude[i] = interpolation_for_periodic(t1,t2,InsFrameList[match_index].attitude[i],InsFrameList[another_match_index].attitude[i],t);
				}
				return true;
			} //end: if(current_len == 1) ... else...
		} //end: if (current_len == 0)... else...
		return true;
	}
	bool GetLastInsFrame(C_InsFrame &lastInsframe)
	{
		if (current_len>0)
		{
			lastInsframe.copyfrom(InsFrameList[0]);
			return true;
		}
		else
			return false;
	}
	void DrawCompass(Mat &Image,Scalar color = Scalar(0,0,0))
	{
		C_InsFrame LastIns;
		Point xy(50,50);
		circle(Image,xy,20,color,-1);
		if(GetLastInsFrame(LastIns) == true)
		{
			double azimuth = LastIns.attitude[2]*PI/180;
			Point start(50,50);
			int dx = 0*cos(azimuth) - 40*sin(azimuth);
			int dy = 0*sin(azimuth) + 40*cos(azimuth);
			Point end(50+dx,50-dy);
			line(Image,start,end,Scalar(255,0,0),5);
		}
	}
};




#endif /* C_INS_H_ */
