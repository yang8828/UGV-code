/*
 * C_Scan.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_SCAN_H_
#define C_SCAN_H_

#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>


#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_Ins.h>

class C_ScanFrame
{
public:
	//Time Stamp
	long timestamp;
	//Position and Attitude
	C_InsFrame ins;
	bool withInsData;
	//ScanInfo
	int x[720];
	int y[720];
	C_ScanFrame()
	{
		for (int i=0;i<720;i++)
		{
			x[i] = 0;
			y[i] = 0;
		}
	}
	C_ScanFrame(const int32_t _ms, const boost::array<int, 720u> _x, const boost::array<int, 720u> _y, C_InsFrameHis insFrameHis)
	{
		for (int i=0;i<720;i++)
		{
			x[i] = _x[i];
			y[i] = _y[i];
		}
		timestamp = _ms;
		getInsFrame(insFrameHis);
	}
	void getInsFrame(C_InsFrameHis insFrameHis)
	{
		withInsData = insFrameHis.getMatchInsFrame(timestamp,ins);
	}
	void copyfrom(C_ScanFrame src)
	{
		timestamp = src.timestamp;
		ins.copyfrom(src.ins);
		withInsData = src.withInsData;
		for(int i=0;i<720;i++)
		{
			x[i] = src.x[i];
			y[i] = src.y[i];
		}
	}
	void RTto(C_ScanFrame &dst,C_InsFrame dst_InsFrame)
	{
		dst.timestamp = dst_InsFrame.timestamp;
		dst.ins.copyfrom(dst_InsFrame);
		dst.withInsData = withInsData;
		double TNE_x;
		double TNE_y;
		for(int i=0;i<720;i++)
		{
			if(x[i]==-1 || y[i]==-1)
			{
				dst.x[i] = -1;
				dst.y[i] = -1;
			}
			else
			{
				TNE_x = dst_InsFrame.position[0] - ins.position[0];
				TNE_y = dst_InsFrame.position[1] - ins.position[1];
				doRT(x[i],y[i],dst.x[i],dst.y[i],TNE_x,TNE_y,ins.attitude[2],dst_InsFrame.attitude[2]);
			}

		}
	}
	void Draw(Mat image,int c_R,int c_G,int c_B)
	{
		for (int i=0;i<720;i++)
		{
			CvPoint xy;
			xy.x = x[i];
			xy.y = 3000 - y[i];
			circle(image,xy,6,Scalar(c_R,c_G,c_B),-1);
		}
	}
	void Draw(Mat image,Scalar color)
	{
		for (int i=0;i<720;i++)
		{
			CvPoint xy;
			xy.x = x[i];
			xy.y = 3000 - y[i];
			circle(image,xy,6,color,-1);
		}
	}
	void PrintTimeStamp()
	{
		fprintf(stderr,"ScanFrame:\tTS:%ld\tINS TS:\t%ld\n",timestamp,ins.timestamp);
	}
	void PrintXY()
	{
		for(int i=0;i<720;i++)
		{
			fprintf(stderr,"%d %d,",x[i],y[i]);
		}
		fprintf(stderr,"\n");
	}
};

class C_ScanFrameHis
{
	C_ScanFrame scanframe[MAX_SCAN_HIS_LEN];
	int Scan_his_len;
public:
	C_ScanFrameHis()
	{
		Scan_his_len=0;
	}
	long addSample(const int32_t _ms, const boost::array<int, 720u> _x, const boost::array<int, 720u> _y, C_InsFrameHis insFrameHis)
	{
		C_ScanFrame newFrame(_ms,_x,_y,insFrameHis);
		addScanFrameHis(newFrame);
		return _ms;
	}
	bool addScanFrameHis(C_ScanFrame newframe)
	// Add successful return true, Error return false;
	{
		for(int i=MAX_SCAN_HIS_LEN-1;i>0;i--)
		{
			scanframe[i].copyfrom(scanframe[i-1]);
		}
		scanframe[0].copyfrom(newframe);
		Scan_his_len++;
		if (Scan_his_len > MAX_SCAN_HIS_LEN)
			Scan_his_len = MAX_SCAN_HIS_LEN;
		if (Scan_his_len>=2)
		{
			if (scanframe[0].timestamp < scanframe[1].timestamp )
			{
				return false;
			}
		}
		return true;
	}
	void Draw(Mat Image,C_InsFrame dst_InsFrame,int WindowLen)
	{
		C_ScanFrame temp_ScanFrame;
		//fprintf(stderr,"Scan_his_len:%d\n",Scan_his_len);
		int looptimes = ((WindowLen<Scan_his_len)?WindowLen:Scan_his_len);
		for (int i=0;i<looptimes;i++)
		{
			//scanframe[0].Draw(Image,0,180,0);
			if (scanframe[i].withInsData == true)
			{
				scanframe[i].RTto(temp_ScanFrame,dst_InsFrame);
				//temp_ScanFrame.Draw(Image,0,180,0);
				temp_ScanFrame.Draw(Image,ScanInfoColor);
				//scanframe[i].PrintXY();
				//temp_ScanFrame.PrintXY();
			}
		}
	}
	bool GetLastFrame(C_ScanFrame &dst)
	{
		if(Scan_his_len > 0)
		{
			dst.copyfrom(scanframe[0]);
			return true;
		}
		else
			return false;
	}
};



#endif /* C_SCAN_H_ */
