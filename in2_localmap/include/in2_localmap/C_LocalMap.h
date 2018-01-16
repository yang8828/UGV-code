#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <ros/ros.h>

#define LANETYPE_NONE 0
#define LANETYPE_SSW 1
#define LANETYPE_SSY 2
#define LANETYPE_SDW 3
#define LANETYPE_SDY 4
#define LANETYPE_STOP 5

#define INS_FRAME_HIS_LEN 50	// C_InsFrameHis records for (INS_FRAME_LIST_LEN / Ins_Sampling_Frequence) second(s) 
#define MAX_LANE_HIS_LEN 15		// C_FrameHistory records for last MAX_LANE_HIS_LEN LaneInfoFrame
#define MAX_SCAN_HIS_LEN 15		// C_FrameHistory records for last MAX_SCAN_HIS_LEN ScanInfoFrame

#define PI 3.1415926

#define LANEINFO_DELAY 220 // or 210

using namespace cv;
double interpolation(double t1,double t2,double y1,double y2,double t)
{
	if(t1 == t2)
		return 0.5*(y1+y2);
	else
		return (y2-y1)*(t-t1)/(t2-t1) + y1;
}

void doRT(int src_x,int src_y,int &dst_x,int &dst_y,double T_NEx,double T_NEy,double azimuth_src,double azimuth_dst)
{
	azimuth_src = azimuth_src*PI/180;
	azimuth_dst = azimuth_dst*PI/180;
	double Tx = 1500;
	double Ty = 1000;
	double UVx = (double)src_x-Tx;
	double UVy = (double)src_y-Ty;
	double NEx = cos(azimuth_src)*UVx + sin(azimuth_src)*UVy;
	double NEy = -sin(azimuth_src)*UVx + cos(azimuth_src)*UVy;
	double NEx_ = NEx - T_NEx*50;
	double NEy_ = NEy - T_NEy*50;
	double UVx_ = cos(azimuth_dst)*NEx_ - sin(azimuth_dst)*NEy_; 
	double UVy_ = +sin(azimuth_dst)*NEx_ + cos(azimuth_dst)*NEy_;
	dst_x = (int)(UVx_ + Tx);
	dst_y = (int)(UVy_ + Ty);
};

class C_TimeStamp
{
public:
	long ms_since;
	C_TimeStamp()
	{
		ms_since = 0;
	}
	void setMs(long ms)
	{
		ms_since = ms;
	}
	void copyfrom(C_TimeStamp src)
	{
		ms_since = src.ms_since;
	}
	bool islargerthan(C_TimeStamp src)
	{
		if (ms_since>src.ms_since)
			return true;
		else
			return false;
	}
	int msAfter(C_TimeStamp src)
	{
		return ms_since - src.ms_since;
	} 
};

class C_InsFrame
{	
public:
//Valuables
	//Time Stamp	
	C_TimeStamp SysTimestamp;
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
		long ms = _ms;
		setSysTimestamp(ms);
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
	C_InsFrame(long _ms,double s,double lngt,double lati,double *pos,double *att,double *vel)
	{
		setInfo(s,lngt,lati,pos,att,vel);
		setSysTimestamp(_ms);
	}
	void setInfo(double s,double lngt,double lati,double *pos,double *att,double *vel)
	{
		second = s;
		longtitude = lngt;
		latitude = lati;
		for(int i=0;i<3;i++)
		{
			position[i] = pos[i];
			attitude[i] = att[i];
			velocity[i] = vel[i];
		}
	}
	void setSysTimestamp(long _ms)//ros::Time::now())//??
	{
		SysTimestamp.setMs(_ms);
	}
	void copyfrom(C_InsFrame src)
	{
		SysTimestamp.copyfrom(src.SysTimestamp);
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
		fprintf(stderr,"INS Frame:\t\t\tINS TS:\t%ld\n",SysTimestamp.ms_since);
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
	bool addInsFrameHis(C_InsFrame newframe)
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
			if (InsFrameList[0].SysTimestamp.islargerthan(InsFrameList[1].SysTimestamp) == false)
			{
				return false;
			}
		}
		return true;
	}
	void printHisTimeStamp()
	{
		for(int i=0;i<current_len;i++)
		{
			fprintf(stderr,"%ld ",InsFrameList[i].SysTimestamp.ms_since);
		}
		fprintf(stderr,"\n");
	}
	bool getMatchInsFrame(C_TimeStamp timestamp,C_InsFrame &MatchFrame)
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
					if (timestamp.islargerthan(InsFrameList[i].SysTimestamp))
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
				double t1 = InsFrameList[match_index].SysTimestamp.ms_since;
				double t2 = InsFrameList[another_match_index].SysTimestamp.ms_since;
				//Time Stamp	
				MatchFrame.SysTimestamp.ms_since = timestamp.ms_since;
				double t  = MatchFrame.SysTimestamp.ms_since;
				//InsInfo
				MatchFrame.second = interpolation(t1,t2,InsFrameList[match_index].second,InsFrameList[another_match_index].second,t);
				MatchFrame.longtitude = interpolation(t1,t2,InsFrameList[match_index].longtitude,InsFrameList[another_match_index].longtitude,t);
				MatchFrame.latitude = interpolation(t1,t2,InsFrameList[match_index].latitude,InsFrameList[another_match_index].latitude,t);
				for(int i=0;i<3;i++)
				{
					MatchFrame.position[i] = interpolation(t1,t2,InsFrameList[match_index].position[i],InsFrameList[another_match_index].position[i],t);
					MatchFrame.attitude[i] = interpolation(t1,t2,InsFrameList[match_index].attitude[i],InsFrameList[another_match_index].attitude[i],t);
					MatchFrame.velocity[i] = interpolation(t1,t2,InsFrameList[match_index].velocity[i],InsFrameList[another_match_index].velocity[i],t);
				}
				return true;
				
			} //end: if(current_len == 1) ... else...
		} //end: if (current_len == 0)... else...
		return true;
	}
};

class C_LaneFrame
{
public:
	//Time Stamp	
	C_TimeStamp SysTimestamp;
	//Position and Attitude
	C_InsFrame ins;
	bool withInsData;
	//LineInfo
	int r[20];
	int theta[20];
	int type[20];
	int beliefe[20];
	int isCurve[20];
	int points_x[620];
	int points_y[620];
	C_LaneFrame()
	{
		for(int i=0;i<20;i++)
		{
			r[i] = 0;
			theta[i] = 0;
			type[i] = 0;
			beliefe[i] = 0;
			isCurve[i] = 0;
		}
		for(int i=0;i<620;i++)
		{
			points_x[i] = 0;
			points_y[i] = 0;
		}
	}
	C_LaneFrame(const int32_t _ms, const boost::array<int, 20u> _r, const boost::array<int, 20u> _theta, const boost::array<int, 20u> _type, const boost::array<int, 20u> _beliefe, const boost::array<int, 20u> _isCurve, const boost::array<int, 620u> _points_x, const boost::array<int, 620u> _points_y, C_InsFrameHis insFrameHis)
	{
		for(int i=0;i<20;i++)
		{
			r[i] = _r[i];
			theta[i] = _theta[i];
			type[i] = _type[i];
			beliefe[i] = _beliefe[i];
			isCurve[i] = _isCurve[i];
		}
		for(int i=0;i<620;i++)
		{
			points_x[i] = _points_x[i];
			points_y[i] = _points_y[i];
		}
		long ms = _ms - LANEINFO_DELAY;
		setSysTimestamp( ms);
		getInsFrame(insFrameHis);
	}
	C_LaneFrame(long _ms,int *_r,int *_theta,int *_type,int *_beliefe,int *_isCurve,int *_points_x,int *_points_y,C_InsFrameHis insFrameHis)
	{
		setInfo(_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y);
		setSysTimestamp( _ms);
		getInsFrame(insFrameHis);
	}
	void setInfo(int *_r,int *_theta,int *_type,int *_beliefe,int *_isCurve,int *_points_x,int *_points_y)
	{
		for(int i=0;i<20;i++)
		{
			r[i] = _r[i];
			theta[i] = _theta[i];
			type[i] = _type[i];
			beliefe[i] = _beliefe[i];
			isCurve[i] = _isCurve[i];
		}
		for(int i=0;i<620;i++)
		{
			points_x[i] = _points_x[i];
			points_y[i] = _points_y[i];
		}
	}
	void setSysTimestamp(long _ms)//ros::Time::now())//??
	{
		SysTimestamp.setMs(_ms);
	}
	void getInsFrame(C_InsFrameHis insFrameHis)
	{
		withInsData = insFrameHis.getMatchInsFrame(SysTimestamp,ins);
	}
	void copyfrom(C_LaneFrame src)
	{
		SysTimestamp.copyfrom(src.SysTimestamp);
		ins.copyfrom(src.ins);
		withInsData = src.withInsData;
		for(int i=0;i<20;i++)
		{
			r[i] = src.r[i];
			theta[i] = src.theta[i];
			type[i] = src.type[i];
			beliefe[i] = src.beliefe[i];
			isCurve[i] = src.isCurve[i];
		}
		for(int i=0;i<620;i++)
		{
			points_x[i] = src.points_x[i];
			points_y[i] = src.points_y[i];
		}
	}
	void RTto(C_LaneFrame &dst,C_InsFrame dst_InsFrame)
	{
		dst.SysTimestamp.copyfrom(dst_InsFrame.SysTimestamp);
		dst.ins.copyfrom(dst_InsFrame);
		dst.withInsData = withInsData;
		for(int i=0;i<20;i++)
		{
			dst.r[i] = r[i];
			dst.theta[i] = theta[i];
			dst.type[i] = type[i];
			dst.beliefe[i] = beliefe[i];
			dst.isCurve[i] = isCurve[i];
		}
		double TNE_x;
		double TNE_y;
		for(int i=0;i<620;i++)
		{
			TNE_x = dst_InsFrame.position[0] - ins.position[0];
			TNE_y = dst_InsFrame.position[1] - ins.position[1];
			doRT(points_x[i],points_y[i],dst.points_x[i],dst.points_y[i],TNE_x,TNE_y,ins.attitude[2],dst_InsFrame.attitude[2]);
		}
	}
	void Draw(IplImage *image,int c_R=-1,int c_G=-1,int c_B=-1)
	{
		bool flag_no_color_input = false;
		if(c_R==-1 || c_G==-1 || c_B ==-1)
		{
			 flag_no_color_input = true;
		}
		for (int i=0;i<20;i++)
		{
			if (type[i]!=LANETYPE_NONE)
			{
				if(flag_no_color_input)
				{
					double color_theta = 2*PI - ((double)beliefe[i])/15*2*PI;
					//c_R = 255.0*(1-sin(color_theta))*0.5;
					//c_G = 255.0*(1-cos(color_theta))*0.5;
					//c_B = 255.0*(1+sin(color_theta))*0.5;
					c_R = 255.0*(16 - beliefe[i])/16.0;
					c_G = 255.0*(16 - beliefe[i])/16.0;
					c_B = 255.0;
				}
				for(int j=0;j<31;j++)
				{
					
					CvPoint xy;
					xy.x = points_x[i*31+j];
					xy.y = 3000 - points_y[i*31+j];
					cvCircle(image,xy,6,CV_RGB(c_R,c_G,c_B),-1);
				}
			}
		}
	}
	void PrintTimeStamp()
	{
		fprintf(stderr,"LaneFrame:\tTS:%ld\tINS TS:\t%ld\n",SysTimestamp.ms_since,ins.SysTimestamp.ms_since);
	}
	void PrintXY()
	{
		for(int i=0;i<620;i++)
		{
			fprintf(stderr,"%d %d,",points_x[i],points_y[i]);
		}
		fprintf(stderr,"\n");
	}
};

class C_ScanFrame
{
public:
	//Time Stamp	
	C_TimeStamp SysTimestamp;
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
		long ms = _ms;
		setSysTimestamp(ms);
		getInsFrame(insFrameHis);
	}
	C_ScanFrame(long _ms,int *_x,int *_y,C_InsFrameHis insFrameHis)
	{
		setInfo(_x,_y);
		setSysTimestamp(_ms);
		getInsFrame(insFrameHis);
	}
	void setInfo(int *_x,int *_y)
	{
		for (int i=0;i<720;i++)
		{
			x[i] = _x[i];
			y[i] = _y[i];
		}
	}
	void setSysTimestamp(long _ms)
	{
		SysTimestamp.setMs(_ms);
	}
	void getInsFrame(C_InsFrameHis insFrameHis)
	{
		withInsData = insFrameHis.getMatchInsFrame(SysTimestamp,ins);
	}
	void copyfrom(C_ScanFrame src)
	{
		SysTimestamp.copyfrom(src.SysTimestamp);
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
		dst.SysTimestamp.copyfrom(dst_InsFrame.SysTimestamp);
		dst.ins.copyfrom(dst_InsFrame);
		dst.withInsData = withInsData;
		double TNE_x;
		double TNE_y;
		for(int i=0;i<720;i++)
		{
			TNE_x = dst_InsFrame.position[0] - ins.position[0];
			TNE_y = dst_InsFrame.position[1] - ins.position[1];
			doRT(x[i],y[i],dst.x[i],dst.y[i],TNE_x,TNE_y,ins.attitude[2],dst_InsFrame.attitude[2]);
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
	void Draw(IplImage *image,int c_R,int c_G,int c_B)
	{
		for (int i=0;i<720;i++)
		{
			CvPoint xy;
			xy.x = x[i];
			xy.y = 3000 - y[i];
			cvCircle(image,xy,6,CV_RGB(c_R,c_G,c_B),-1);
		}
	}
	void PrintTimeStamp()
	{
		fprintf(stderr,"ScanFrame:\tTS:%ld\tINS TS:\t%ld\n",SysTimestamp.ms_since,ins.SysTimestamp.ms_since);
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
	C_ScanFrameHis()
	{
		Scan_his_len=0;
	}
	void addSample(const int32_t _ms, const boost::array<int, 720u> _x, const boost::array<int, 720u> _y, C_InsFrameHis insFrameHis)
	{
		C_ScanFrame newFrame(_ms,_x,_y,insFrameHis);
		addScanFrameHis(newFrame);
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
			if (scanframe[0].SysTimestamp.islargerthan(scanframe[1].SysTimestamp) == false)
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
				temp_ScanFrame.Draw(Image,0,180,0);
				//scanframe[i].PrintXY();
				//temp_ScanFrame.PrintXY();
			}
		}
	}
};

class C_FrameHistory
{
public:
	C_InsFrameHis InsFrameHis;
	C_LaneFrame laneframe[MAX_LANE_HIS_LEN];
	int lane_his_len;
	C_ScanFrame scanframe[MAX_SCAN_HIS_LEN];
	int Scan_his_len;
	C_FrameHistory()
	{
		lane_his_len=0;
		Scan_his_len=0;
	}
	bool addLaneFrameHis(C_LaneFrame newframe)
	// Add successful return true, Error return false;
	{
		for(int i=MAX_LANE_HIS_LEN-1;i>0;i--)
		{
			laneframe[i].copyfrom(laneframe[i-1]);
		}
		laneframe[0].copyfrom(newframe);
		lane_his_len++;
		if (lane_his_len > MAX_LANE_HIS_LEN)	
			lane_his_len = MAX_LANE_HIS_LEN;
		if (lane_his_len>=2)
		{
			if (laneframe[0].SysTimestamp.islargerthan(laneframe[1].SysTimestamp) == false)
			{
				return false;
			}
		}
		return true;
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
			if (scanframe[0].SysTimestamp.islargerthan(scanframe[1].SysTimestamp) == false)
			{
				return false;
			}
		}
		return true;
	}
	void Show(C_InsFrame dst_InsFrame,int WindowLen)
	{
		fprintf(stderr,"####################################\n");
		CvSize size;
		size.height =3000;
		size.width = 3000;
		IplImage *Image = cvCreateImage(size,8,4);
		cvSet(Image,cvScalar(255,255,255,0));
		
		int looptimes;
		double color_theta;

		C_LaneFrame temp_LaneFrame;
		//fprintf(stderr,"lane_his_len:%d\n",lane_his_len);
		looptimes = ((WindowLen<lane_his_len)?WindowLen:lane_his_len);
		for (int i=0;i<looptimes;i++)
		{
			//laneframe[0].Draw(Image,180,0,0);
			color_theta = 2*PI - ((double)i)/((double)looptimes)*2*PI;
			if (laneframe[i].withInsData == true)
			{
				laneframe[i].RTto(temp_LaneFrame,dst_InsFrame);
				temp_LaneFrame.Draw(Image);
				//temp_LaneFrame.Draw(Image,(int)(255.0*(1-sin(color_theta))*0.5),(int)(255.0*(1-cos(color_theta))*0.5),(int)(255.0*(1+sin(color_theta))*0.5));
				//laneframe[i].Draw(Image,0,0,0);
				//laneframe[i].PrintXY();
				//temp_LaneFrame.PrintXY();
			}
		}
		
		C_ScanFrame temp_ScanFrame;
		//fprintf(stderr,"Scan_his_len:%d\n",Scan_his_len);
		looptimes = ((WindowLen<Scan_his_len)?WindowLen:Scan_his_len);
		for (int i=0;i<looptimes;i++)
		{
			//scanframe[0].Draw(Image,0,180,0);
			if (scanframe[i].withInsData == true)
			{
				scanframe[i].RTto(temp_ScanFrame,dst_InsFrame);
				temp_ScanFrame.Draw(Image,0,180,0);
				//scanframe[i].PrintXY();
				//temp_ScanFrame.PrintXY();
			}
		}
		cvRectangle(Image,cvPoint(1535,1910),cvPoint(1460,2060),CV_RGB(0,0,0),2);
		cvNamedWindow("C_LocalMap",0);
		cvShowImage("C_LocalMap",Image);
		cvWaitKey(20);
		//cv::imshow("C_LocalMap",Image);
		cvReleaseImage(&Image);
	}
	bool GetLastInsFrame(C_InsFrame &lastInsframe)
	{
		if (InsFrameHis.current_len>0)
		{
			lastInsframe.copyfrom(InsFrameHis.InsFrameList[0]);
			return true;
		}
		else
			return false;
	}
};


