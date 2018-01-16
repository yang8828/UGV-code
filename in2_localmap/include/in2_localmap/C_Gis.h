/*
 * C_Gis.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_GIS_H_
#define C_GIS_H_

#include <ros/ros.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>


#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_Ins.h>
#include <in2_localmap/C_Stopline.h>
#include <in2_localmap/C_GisError.h>

class C_StoplineModel;
class C_GisFrame
{
	//Error = Corret - Now
	double North_Error_init;
	double East_Error_init;
	double North_Error_local;
	double East_Error_local;
public:
	long timestamp;
	int  laneNum;
	double  angle; //-180.0~180.0
	long  speedLimit;
	double dist_to_intersection;
	int match_state;
	int curveLevel;

	int CornerInStatus;
	int CornerInID;
	int CornerOutStatus;
	int CornerOutID;

	int InitGisEroLevel;
	int LastInitGisEroLevel;
	bool isInitGisError;
	int InitGisErrorCnter;

	int CurLvl;
	int LastLvl;
	int SendLvl;
	bool isDeadHis;
	bool isInit;
	bool isSendHigh;
	bool isSendLow;
	int DeadHisCnter;
	int InitCnter;
	int SendHighCnter;
	int SendLowCnter;
	bool haveLastGis;
	int DelayDeadCnter;

	int iGisLeftWidth;
	int iGisRightWidth;
	double dGisLeftWidth;
	double dGisRightWidth;
	bool GisWidthLegal;

	int x[301];
	int y[301];
	C_InsFrame ins;
	bool withIns;
public:
	C_GisFrame()
	{
		timestamp = 0;
		for(int i=0;i<301;i++)
		{
			x[i] = 0;
			y[i] = 0;
		}
		// Error = Aligned - Raw;


		laneNum = 0;
		angle = 0.0;
		speedLimit = 0.0;
		dist_to_intersection = 0.0;
		match_state = 0;
		curveLevel = -1;

		CornerInStatus = 0;
		CornerInID = 0;
		CornerOutStatus = 0;
		CornerOutID = 0;

		InitGisEroLevel = LOW_LVL;
		LastInitGisEroLevel = LOW_LVL;
		isInitGisError = false;
		InitGisErrorCnter = 0;

		CurLvl = LOW_LVL;
		LastLvl = LOW_LVL;
		SendLvl = SEND_LOW_LVL;
		isDeadHis = false;
		isInit = false;
		isSendHigh = false;
		isSendLow = false;
		DeadHisCnter = 0;
		InitCnter = 0;
		SendHighCnter = 0;
		SendLowCnter = 0;
		haveLastGis = false;
		DelayDeadCnter = 0;

		iGisLeftWidth = 0;
		iGisRightWidth = 0;
		dGisLeftWidth = 0.0;
		dGisRightWidth = 0.0;
		GisWidthLegal = false;

		North_Error_init = 0.0;//3*50;
		East_Error_init = 0.0;//-1.5*50;
		North_Error_local = 0.0;
		East_Error_local = 0.0;
		withIns = false;
	}
	//void updateTo(C_InsFrame dst_Ins)
	//{
	//
	//}
	void copyFrom(C_GisFrame src)
	{
		//Error = Corret - Now
		North_Error_init = src.North_Error_init;
		East_Error_init = src.East_Error_init;
		North_Error_local = src.North_Error_local;
		East_Error_local = src.East_Error_local;
		timestamp = src.timestamp;
		laneNum = src.laneNum;
		angle = src.angle;
		speedLimit = src.speedLimit;
		dist_to_intersection = src.dist_to_intersection;
		match_state = src.match_state;
		curveLevel = src.curveLevel;

		CornerInStatus = src.CornerInStatus;
		CornerInID = src.CornerInID;
		CornerOutStatus = src.CornerOutStatus;
		CornerOutID = src.CornerOutID;

		for(int i=0;i<301;i++)
		{
			x[i] = src.x[i];
			y[i] = src.y[i];
		}
		ins.copyfrom(src.ins);
		withIns = src.withIns;
	}

	void RTto(C_GisFrame &dst,C_InsFrame dst_InsFrame)
	{
		dst.copyFrom(*this);
		double TNE_x;
		double TNE_y;
		for(int i=0;i<301;i++)
		{
			TNE_x = dst_InsFrame.position[0] - ins.position[0];
			TNE_y = dst_InsFrame.position[1] - ins.position[1];
			doRT(x[i],y[i],dst.x[i],dst.y[i],TNE_x,TNE_y,ins.attitude[2],dst_InsFrame.attitude[2]);
		}
	}
	long SetWithSample(const boost::array<int32_t,255> _data,const boost::array<int, 301u> _x, const boost::array<int, 301u> _y,C_InsFrameHis insFrameHis)
	{
		/*
		gis = *GIS_msg;
			gis_received = true;
			speed_gis = gis.data[0x13];
			//speed_gis = 40000;
			drive_state = gis.data[0x12];
			int bit[8];
			//µÚÒ»žöF£º0 ³ö·¢Ç° 1 ·¢³µºóÆô¶¯Ç° 2Õý³£ÐÐÊ»  3 ±ÈÈüœáÊø
			match_state = ((drive_state&(0x0f<<6*4))>>6*4);
			//µÚ¶þžöF£º0 ÎÞºìÂÌµÆÖ±ÐÐ 1 ÎÞºìÂÌµÆ×ó×ª 2 ÎÞºìÂÌµÆÓÒ×ª 3 ÓÐºìÂÌµÆÖ±ÐÐ 4 ÓÐºìÂÌµÆ×ó×ª 5 ÓÐºìÂÌµÆÓÒ×ª
			cross_state = ((drive_state&(0x0f<<5*4))>>5*4);
			//µÚÈýËÄÎåžöF£º±íÊŸÏÖÔÚÀëºìÂÌµÆŸàÀë£¬µ¥Î»Ã×/10
			bit[2] = ((drive_state&(0x0f<<4*4))>>4*4);
			bit[3] = ((drive_state&(0x0f<<3*4))>>3*4);
			bit[4] = ((drive_state&(0x0f<<2*4))>>2*4);
			cross_dist = (double)((bit[2]<<4*2)+(bit[3]<<4*1)+(bit[4]<<4*0))/10.0;
			dev_angle = gis.data[0x11]/1000.0 * PI/180.0;
*/
		timestamp = _data[0x00];
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		C_InsFrame tempins;
		insFrameHis.GetLastInsFrame(tempins);
		timestamp = tempins.timestamp;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		laneNum   = (_data[0x10]>>16)&0xFFFF;
		if(laneNum == 100)
		{
			laneNum = -1; // the num of lane is unclear
		}
		angle     = _data[0x11]/1000.0; //-180~180;
		speedLimit= _data[0x13]/1000.0;
		int bit[8];
		bit[2] = ((_data[0x12]&(0x0f<<4*4))>>4*4);
		bit[3] = ((_data[0x12]&(0x0f<<3*4))>>3*4);
		bit[4] = ((_data[0x12]&(0x0f<<2*4))>>2*4);
		bit[6] = ((_data[0x12]&(0x0f<<0*4))>>0*4);
		match_state  = ((_data[0x12]&(0x0f<<6*4))>>6*4);
		dist_to_intersection = (double)((bit[2]<<4*2)+(bit[3]<<4*1)+(bit[4]<<4*0))/10.0;
		curveLevel = bit[6];

		CornerInStatus = (_data[0x14]>>28)&0xF;
		CornerInID = (_data[0x14]>>16)&0xFFF;
		CornerOutStatus = (_data[0x14]>>12)&0xF;
		CornerOutID = (_data[0x14])&0xFFF;

//-----------------------------------------------------------------------------
//		int tmp_lvl = ((_data[0x12]&(0x0f<<0*4))>>0*4);
//		if(!(tmp_lvl != LOW_LVL && tmp_lvl != HIGH_LVL))
//		{ // legal
//			LastInitGisEroLevel = InitGisEroLevel;
//			InitGisEroLevel = tmp_lvl;
//			if(LastInitGisEroLevel == LOW_LVL && InitGisEroLevel == HIGH_LVL)
//			{
//				isInitGisError = true;
//			}
//		}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
		int tmp_lvl = ((_data[0x12]&(0x0f<<0*4))>>0*4);
		if(!(tmp_lvl != LOW_LVL && tmp_lvl != HIGH_LVL))
		{ // legal
			LastLvl = CurLvl;
			CurLvl = tmp_lvl;
			if(LastLvl == LOW_LVL && CurLvl == HIGH_LVL)
			{
				isDeadHis = true;
				isSendHigh = true;
			}
			else if(LastLvl == HIGH_LVL && CurLvl == LOW_LVL)
			{
				isInit = true;
				isDeadHis = false;
				isSendLow = true;
			}
		}
//-----------------------------------------------------------------------------

//-----------------------------------gis roadedge left and right width------------------------------------------
		int tmp_left = ((_data[0x15]&(0xffff<<1*16))>>1*16);
		int tmp_right = ((_data[0x15]&(0xffff<<0*16))>>0*16);
		if(tmp_left == 0 || tmp_right == 0)
		{
			GisWidthLegal = false;
		}
		else
		{ // legal
			GisWidthLegal = true;
			iGisLeftWidth = tmp_left;
			iGisRightWidth = tmp_right;
			dGisLeftWidth = tmp_left / 100.0;
			dGisRightWidth = tmp_right / 100.0;
		}
//--------------------------------------------------------------------------------------------------------------

		for(int i=0;i<301;i++)
		{
			x[i] = _x[i];
			y[i] = _y[i];
		}
		// Error = Aligned - Raw;
		withIns = insFrameHis.getMatchInsFrame(timestamp,ins);
		return timestamp;
	}
	void DrawRaw(Mat image,Scalar color = GisPathColor,bool line = false)
	{
		for (int i=0;i<301;i++)
		{
			CvPoint xy;
			xy.x = x[i];
			xy.y = 3000 - y[i];
			circle(image,xy,8,color,-1);
			if(i != 300 && line == true)
			{
				CvPoint xy_end;
				xy_end.x = x[i+1];
				xy_end.y = y[i+1];
				cv::line(image,xy,xy_end,GisPathColor,3);
			}
		}
	}
	void DrawAligned(Mat image,Scalar color = GisPathColor,bool line = false)
	{
		for (int i=0;i<301;i++)
		{
			CvPoint xy;
			xy.x = AlignedData().x[i];
			xy.y = 3000 - AlignedData().y[i];
			circle(image,xy,8,color,-1);
			if(i != 300 && line == true)
			{
				CvPoint xy_end;
				xy_end.x = x[i+1];
				xy_end.y = y[i+1];
				cv::line(image,xy,xy_end,GisPathColor,3);
			}
		}
	}
	void DrawInitialAligned(Mat image,Scalar color = GisPathColor,bool line = false)
	{
		for (int i=0;i<301;i++)
		{
			CvPoint xy;
			xy.x = initialAlignedData().x[i];
			xy.y = 3000 - initialAlignedData().y[i];
			circle(image,xy,8,color,-1);
			if(i != 300 && line == true)
			{
				CvPoint xy_end;
				xy_end.x = x[i+1];
				xy_end.y = y[i+1];
				cv::line(image,xy,xy_end,GisPathColor,3);
			}
		}
	}
	void DrawAngle(Mat Image,Scalar color = GisPathColor)
	{
		Point start(150,50);
		circle(Image,start,20,Scalar::all(0),-1);
		double angle_rad = angle*PI/180;
		int dx = 40*sin(angle_rad);
		int dy = 40*cos(angle_rad);
		Point end(150+dx,50-dy);
		line(Image,start,end,color,5);
		int fontFace = FONT_HERSHEY_SIMPLEX;
		double fontScale = 1.5;
		int thickness = 3;
		char title_data[30];
		sprintf(title_data,"LaneNum: %d",laneNum);
		putText(Image, title_data, Point(250,50), fontFace, fontScale, InsInfoColor, thickness);
		sprintf(title_data,"CornerI: %d %d",CornerInStatus,CornerInID);
		putText(Image, title_data, Point(250,100), fontFace, fontScale, InsInfoColor, thickness);
		sprintf(title_data,"CornerO: %d %d",CornerOutStatus,CornerOutID);
		putText(Image, title_data, Point(250,150), fontFace, fontScale, InsInfoColor, thickness);

	}
	C_GisFrame AlignedData()
	{
		C_GisFrame aligned;
		aligned.timestamp = timestamp;
		double azimuth = ins.attitude[2]*PI/180;
		double East_Error = East_Error_init + East_Error_local;
		double North_Error = North_Error_init + North_Error_local;
		double u_error = East_Error*cos(azimuth) - North_Error*sin(azimuth);
		double v_error = East_Error*sin(azimuth) + North_Error*cos(azimuth);
		for(int i=0;i<301;i++)
		{
			aligned.x[i] = x[i] + (int)u_error;
			aligned.y[i] = y[i] + (int)v_error;
		}
		aligned.East_Error_init = East_Error_init;
		aligned.North_Error_init = North_Error_init;
		aligned.East_Error_local = East_Error_local;
		aligned.North_Error_local = North_Error_local;

		aligned.laneNum = laneNum;
		aligned.angle = angle;
		aligned.speedLimit = speedLimit;
		aligned.dist_to_intersection = dist_to_intersection;
		aligned.match_state = match_state;
		aligned.curveLevel = curveLevel;

		aligned.CornerInStatus = CornerInStatus;
		aligned.CornerInID = CornerInID;
		aligned.CornerOutStatus = CornerOutStatus;
		aligned.CornerOutID = CornerOutID;

		aligned.ins.copyfrom(ins);
		aligned.withIns = withIns;
		return aligned;
	}
	C_GisFrame initialAlignedData()
	{
		C_GisFrame aligned;
		aligned.timestamp = timestamp;
		double azimuth = ins.attitude[2]*PI/180.0;
		double East_Error = East_Error_init;
		double North_Error = North_Error_init;
		double u_error = East_Error*cos(azimuth) - North_Error*sin(azimuth);
		double v_error = East_Error*sin(azimuth) + North_Error*cos(azimuth);
		for(int i=0;i<301;i++)
		{
			aligned.x[i] = x[i] + (int)u_error;
			aligned.y[i] = y[i] + (int)v_error;
		}
		aligned.East_Error_init = East_Error_init;
		aligned.North_Error_init = North_Error_init;
		aligned.East_Error_local = East_Error_local;
		aligned.North_Error_local = North_Error_local;

		aligned.laneNum = laneNum;
		aligned.angle = angle;
		aligned.speedLimit = speedLimit;
		aligned.dist_to_intersection = dist_to_intersection;
		aligned.match_state = match_state;

		aligned.CornerInStatus = CornerInStatus;
		aligned.CornerInID = CornerInID;
		aligned.CornerOutStatus = CornerOutStatus;
		aligned.CornerOutID = CornerOutID;

		aligned.ins.copyfrom(ins);
		aligned.withIns = withIns;
		return aligned;
	}
	void NorthInc()
	{
		North_Error_init++;
	}
	void NorthDec()
	{
		North_Error_init--;
	}
	void EastInc()
	{
		East_Error_init++;
	}
	void EastDec()
	{
		East_Error_init--;
	}

	void setInitError(double East,double North)
	{
		East_Error_init = East;
		North_Error_init= North;
	}
	bool getAbsolutePoint2(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &angle);
	bool getAbsolutePoint(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins);
	bool getRawPoint(Point CrossPoint,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins);
	bool getRawPoint(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins);
	bool getRawPoint(Point &gisRawPoint,double &angle3K);
	bool getReferencePoint(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins);
	bool AddNewError(int u,int v,C_InsFrame src_INs)
	{
		double azimuth = ins.attitude[2]*PI/180;
		double Eerror_delta = u*cos(-azimuth) - v*sin(-azimuth);
		double Nerror_delta = u*sin(-azimuth) + v*sin(-azimuth);
		double norm = sqrt(Eerror_delta*Eerror_delta + Nerror_delta*Nerror_delta);
		if(norm < 3.0*50.0)
		{
			North_Error_local += Eerror_delta;
			East_Error_local += Nerror_delta;
			return true;
		}
		return false;
	}
	bool SetLocalError(int u,int v,C_InsFrame src_INs,double Threshold)
	{
		double azimuth = ins.attitude[2]*PI/180;
		double Eerror_delta = u*cos(-azimuth) - v*sin(-azimuth);
		double Nerror_delta = u*sin(-azimuth) + v*sin(-azimuth);
		double norm = sqrt(Eerror_delta*Eerror_delta + Nerror_delta*Nerror_delta);
		if(norm < Threshold )//&& norm> 0.3*50) //with deadzone
		{
			North_Error_local = Nerror_delta;
			East_Error_local = Eerror_delta;
			return true;
		}
		return false;
	}
	void printError()
	{
		fprintf(stdout,"INIT  ERROR: %lf,%lf\n",East_Error_init,North_Error_init);
		fprintf(stdout,"LOCOL ERROR: %lf,%lf\n",East_Error_local,North_Error_local);
	}

	double getEastErrorInit()
	{
		return East_Error_init;
	}

	double getEastErrorLocal()
	{
		return East_Error_local;
	}

	double getNorthErrorInit()
	{
		return North_Error_init;
	}

	double getNorthErrorLocal()
	{
		return North_Error_local;
	}

	int initGisErrorByLevel(C_GisError &gis_error)
	{
		if(isInitGisError)
		{
			gis_error.Clear_GisErrorSample();
			setInitError(0.0,0.0);
			isInitGisError = false;
			InitGisErrorCnter++;
			return 1;
		}
		else
		{
			return 0;
		}
	}

	void disInitGisErrorOnImage(Mat &img, Point pos = Point(50,450), Scalar color = Scalar(0,0,255), int fontFace = FONT_HERSHEY_SIMPLEX,
				double fontScale = 2, int thickness = 4)
	{
		char title_data[100];
		sprintf(title_data, "Initiate GIS Error: %d, %d, %d", SendLvl, DeadHisCnter, InitCnter);
		putText(img, title_data, pos, fontFace, fontScale, color, thickness);
	}

	void disGisWidthOnImage(Mat &img, Point pos = Point(50,550), Scalar color = Scalar(0,0,255), int fontFace = FONT_HERSHEY_SIMPLEX,
			double fontScale = 2, int thickness = 4)
	{
		char title_data[100];
		sprintf(title_data, "GIS Width: %d, %d, %lf, %lf", iGisLeftWidth, iGisRightWidth, dGisLeftWidth, dGisRightWidth);
		putText(img, title_data, pos, fontFace, fontScale, color, thickness);
	}

};


#endif /* C_GIS_H_ */
