//#include "stdafx.h"
//#include "afx.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include "math.h"

#include <in2_msgs/RoadEdge.h>
#include <in2_msgs/LaneInfo.h>
#include <in2_msgs/UdpGeneral.h>
#include <in2_msgs/LaneInfoV2.h>


#define pi 3.1415926535


#define DebugDisplay 1
#define DebugRecordAVI	0

#define LANETYP_NONE 0
#define LANETYP_SSW 1
#define LANETYP_SSY 2
#define LANETYP_SDW 3
#define LANETYP_SDY 4
#define LANETYP_STOP 5

using namespace std;
using namespace cv;


//#define range 30
//#define gridsize 0.08


#define range 18
#define gridsize 0.20


#define L 300
#define LINE_HORIZONTAL 0
#define LINE_VERTICAL 1


/////////////Setup Parameters//////////////////////
#define IsLikelyToFront 1
#define LeftDefaultDis 10.0/gridsize
#define RightDefaultDis 10.0/gridsize
#define LeftEdgeThresh 0.3
#define RightEdgeThresh 0.3
#define CrossAngleRange 15.0
#define CrossDetThresh 0.6
////////////////////////////////////////////////////



static const std::string OPENCV_WINDOW = "Heightmap";

ros::Publisher RoadEdge_pub;
in2_msgs::RoadEdge roadedge;

ros::Publisher LaneInfo_pub;
in2_msgs::LaneInfo laneinfo;

ros::Publisher GIS_pub;
in2_msgs::UdpGeneral gis;

ros::Publisher LaneInfoV2_pub;
in2_msgs::LaneInfoV2 laneinfov2;

Mat fxh;
Mat fyh;
Mat fxv;
Mat fyv;


void mcvFilterLinesINIT()
{
	fyh = cv::getGaussianKernel(30,1);
	fxh = cv::getGaussianKernel(14,1) - cv::getGaussianKernel(14,4);
	fyh = fyh.t();

	fxv = cv::getGaussianKernel(30,1);
	fyv = cv::getGaussianKernel(14,0.5) - cv::getGaussianKernel(14,3);
	fyv = fyv.t();
}

void mcvFilterLines(cv::InputArray inImage, cv::OutputArray outImage, int lineType)
{
	Mat InImage = inImage.getMat();
	Mat OutImage;
	switch (lineType)
	{
	case LINE_HORIZONTAL:
		filter2D(InImage, OutImage,InImage.depth(), fxh);
		filter2D(OutImage,OutImage,OutImage.depth(), fyh);
		break;
	case LINE_VERTICAL:
		filter2D(InImage, OutImage,InImage.depth(), fxv);
		filter2D(OutImage,OutImage,OutImage.depth(), fyv);
		break;
	}
	int max = 0;
	for(int i = 0; i < OutImage.cols;i++)
		for(int j = 0; j< OutImage.rows; j++)
			if(OutImage.at<uchar>(i,j)>max)
				max = OutImage.at<uchar>(i,j);
	OutImage = OutImage*(255.0/max);
	OutImage.copyTo(outImage);
}


class CMidFilter
{
private:
	double v0;
	double v1;
	double v2;


public:
	bool inited;

	CMidFilter(double initval)
	{
		v0 = initval;
		v1 = initval;;
		v2 = initval;;
		inited = true;
	}

	CMidFilter()
	{
		v0 = 0;
		v1 = 0;
		v2 = 0;
		inited = false;
	}

	double cal(double valnow)
	{
		v2 = v1;
		v1 = v0;
		v0 = valnow;
		if(v0<=v1 && v0>=v2 || v0<=v2 && v0>=v1) return v0;
		if(v1<=v0 && v1>=v2 || v1<=v2 && v1>=v0) return v1;
		if(v2<=v1 && v2>=v0 || v2<=v0 && v2>=v1) return v2;
		return valnow;
	}

};

class CLaneDetection
{
public:
	//define the accumulator array: rows correspond to r and columns to theta
	float rMin;
	float rMax;
	//float rStep = (rMax-rMin)/500.0;
	float rStep;
	float thetaMin;
	float thetaMax;
	float thetaStep;

	float lane_width;
	float lane_width_range;

	float lane_width_max;
	float lane_width_min;
	int lane_width_counter;
	CvPoint upper_extend[101];
	CvPoint lower_extend[101];
	vector<CvPoint> detect_lanes;
	
	double thetaold;
	double upold;
	double downold;
	double upbelieve;
	double downbelieve;
	CMidFilter filterUP;
	CMidFilter filterDOWN;

	
	float HoughSin[3600];
	float HoughCos[3600];

	void HoughSpaceInit(void)
	{
		int thetaBins = int((thetaMax-thetaMin)/thetaStep);
		for (int k=0; k<thetaBins; k++)
		{
			float theta = (float)((k*thetaStep+thetaMin)*pi/180.0);
			HoughCos[k] = cos(theta);
			HoughSin[k] = sin(theta);
		}
	}

	void GetHoughSpace(cv::InputArray InImg,cv::OutputArray OutImg, float ThetaMin, float ThetaMax, int RMin, int RMax)
	{
		Mat inimg = InImg.getMat();
		int r0 = int((RMin-rMin)/rStep);
		int theta0 = int((ThetaMin-thetaMin)/thetaStep);

		int rBins = int((RMax-rMin)/rStep);
		int thetaBins = int((ThetaMax-thetaMin)/thetaStep);
		int k0 = 0;

		//Mat HoughImg = Mat::zeros(rBins,thetaBins,CV_32FC1);
		Mat HoughImg = OutImg.getMat();
		if(HoughImg.empty())
		{
			HoughImg = Mat::zeros((int)((rMax-rMin)/rStep),(int)((thetaMax-thetaMin)/thetaStep),CV_32FC1);
		}
		float rval;
		int r;

		//get non-zero points in the image
		int nzPoints_counter = 0;
		uchar valtemp;

		for (int i=1; i<inimg.cols-1; i++)
		{
			for (int j=1; j<inimg.rows-1; j++)
			{
				if(inimg.at<uchar>(i,j))
				{
					valtemp = inimg.at<uchar>(i,j);
					for (int k=theta0; k<thetaBins; k++)
					{
						k0 = k;
						if(k0>=HoughImg.cols)
						{
							k0-=HoughImg.cols;
						}
						else if(k0<0)
						{
							k0+=HoughImg.cols;
						}
						rval = j * HoughCos[k0] + i * HoughSin[k0];
						r = (int)(rval - rMin);
						//accumulate in the hough space if a valid value
						if (r>=r0 && r<rBins)
						{
							HoughImg.at<float>(r, k0) += valtemp;
							//HoughImg.at<float>(r, k) = HoughImg.at<float>(r, k)+1;
						}
						//compute the r value for that point and that theta
						//theta = thetas[k];
						//theta = (k*thetaStep+thetaMin)*pi/180.0;

					}
				}
			}
		}

		HoughImg.copyTo(OutImg);
	}



	void GetHoughSpace(cv::InputArray InImg,cv::OutputArray OutImg)
	{
		Mat inimg = InImg.getMat();
		int rBins = int((rMax-rMin)/rStep);
		int thetaBins = int((thetaMax-thetaMin)/thetaStep);

		Mat houghSpace = Mat::zeros(rBins,thetaBins,CV_32FC1);


		//get non-zero points in the image
		int nzPoints_counter = 0;


		for (int i=0; i<inimg.cols; i++)
		{
			for (int j=0; j<inimg.rows; j++)
			{
				if(inimg.at<uchar>(i,j))
				{
					for (int k=0; k<thetaBins; k++)
					{
						//compute the r value for that point and that theta
						//theta = thetas[k];
						float theta = (k*thetaStep+thetaMin)*pi/180.0;
						float rval = j * cos(theta) + i * sin(theta);
						int r = (int)((rval - rMin) / rStep);
						//accumulate in the hough space if a valid value
						if (r>=0 && r<rBins)
						{
							houghSpace.at<float>(r, k) += inimg.at<uchar>(i,j);
							//houghSpace.at<float>(r, k) = houghSpace.at<float>(r, k)+1;
						}
					}
				}
			}
		}

		houghSpace.copyTo(OutImg);
	}

	void GetBGRHoughSpace(cv::InputArray InImg,cv::OutputArray OutImg)
	{
		Mat inimg = InImg.getMat();
		int rBins = int((rMax-rMin)/rStep);
		int thetaBins = int((thetaMax-thetaMin)/thetaStep);

		Mat houghSpace = Mat::zeros(rBins,thetaBins,CV_32FC3);


		//get non-zero points in the image
		int nzPoints_counter = 0;


		for (int i=0; i<inimg.cols; i++)
		{
			for (int j=0; j<inimg.rows; j++)
			{
				if(inimg.at<Vec3b>(i,j)[0]||inimg.at<Vec3b>(i,j)[1]||inimg.at<Vec3b>(i,j)[2])
				{
					for (int k=0; k<thetaBins; k++)
					{
						//compute the r value for that point and that theta
						//theta = thetas[k];
						float theta = (k*thetaStep+thetaMin)*pi/180.0;
						float rval = j * cos(theta) + i * sin(theta);
						int r = (int)((rval - rMin) / rStep);
						//accumulate in the hough space if a valid value
						if (r>=0 && r<rBins)
						{
							houghSpace.at<Vec3f>(r, k)[0] += (float)(inimg.at<Vec3b>(i,j)[0]);
							houghSpace.at<Vec3f>(r, k)[1] += (float)(inimg.at<Vec3b>(i,j)[1]);
							houghSpace.at<Vec3f>(r, k)[2] += (float)(inimg.at<Vec3b>(i,j)[2]);
							//houghSpace.at<float>(r, k) = houghSpace.at<float>(r, k)+1;
						}
					}
				}
			}
		}

		houghSpace.copyTo(OutImg);


		////for view/////////////////
		
		float maxvalue = 0;
		for (int i=0; i<houghSpace.cols; i++)
		{
			for (int j=0; j<houghSpace.rows; j++)
			{
				if(houghSpace.at<Vec3f>(i, j)[0]>maxvalue)
					maxvalue = houghSpace.at<Vec3f>(i, j)[0];
				if(houghSpace.at<Vec3f>(i, j)[1]>maxvalue)
					maxvalue = houghSpace.at<Vec3f>(i, j)[1];
				if(houghSpace.at<Vec3f>(i, j)[2]>maxvalue)
					maxvalue = houghSpace.at<Vec3f>(i, j)[2];
			}
		}

		houghSpace = houghSpace/maxvalue*100;
		Mat houghSpace8uc3;//,houghSpace8uc3hsv;
		houghSpace.convertTo(houghSpace8uc3,CV_8UC3);
		//imshow("houghSpace8uc3",houghSpace8uc3);
		imwrite("houghSpace8uc3.bmp",houghSpace8uc3);
		////////////////////////////////
	}


	
	CvPoint FindMax(cv::InputArray InImg, int xmin, int xmax, int ymin, int ymax, float min_val)
	{
		CvPoint max_pt;
		max_pt.x = 0;
		max_pt.y = 0;
		Mat inimg = InImg.getMat();
		float max_hough_pt = 0;
		for(int i=xmin; i<=xmax; i++)
		{
			for(int j=ymin; j<=ymax; j++)
			{
				if(i>0 && i<inimg.rows)
				{
					int j0 = j;
					if(j0 >= inimg.cols) j0 -= inimg.cols;
					else if(j0 < 0) j0 += inimg.cols;
					if(j0>0 && j0<inimg.cols)
					{
						if (inimg.at<float>(i,j0) > max_hough_pt && inimg.at<float>(i,j0) >= min_val)
						{
							max_hough_pt = inimg.at<float>(i,j0);
							max_pt.x = i;
							max_pt.y = j0;
						}
					}
				}
			}
		}
		if(max_pt.x==xmin||max_pt.x==xmax)
		{
			max_pt.x = 0;
			max_pt.y = 0;
		}
		return max_pt;
	}



	//std::vector<Vec2f> lines;
	
	CvPoint max_value_pt;
	float max_value;
	int maxymin;
	int maxymax;
	vector<CvPoint2D64f> max_value_pt_listf;
	bool isCurve;
	bool StopLaneFound;

	Mat houghspace_show;
	void LaneDetect(cv::InputArray _HoughImg, cv::InputArray LidarImg, cv::Vec4f* lanes, cv::OutputArray IPM_lane, cv::OutputArray IPM)
	{
		//Mat houghSpacebgr = HoughImg.getMat();


		//Mat houghSpacespl[3];
		//split(houghSpacebgr,houghSpacespl);
		//Mat houghSpace = Mat(L,L,CV_32FC1);
		//houghSpace = houghSpacespl[0]*0.05+houghSpacespl[1]*0.15+houghSpacespl[2]*0.8;

		//Mat houghSpace = HoughImg.getMat();

		Mat VeloImage = LidarImg.getMat();

		
		Mat ipm_img = IPM.getMat();
		Mat ipm_lane_img = IPM_lane.getMat();
		Mat HoughImg = _HoughImg.getMat();
		int rBins = HoughImg.rows;
		int thetaBins = HoughImg.cols;
		/////////////////////////////////////////////Find MAX/////////////////////////////////////////
	
	//TEMP = HoughImg/2000.0;
	//cout<<"rows = "<<HoughImg.rows-1<<", cols = "<<HoughImg.cols-1<<endl;
	max_value_pt = FindMax(HoughImg,1,HoughImg.rows-1,1,HoughImg.cols-1,15000.0);
	max_value = HoughImg.at<float>(max_value_pt.x,max_value_pt.y);
	
#if DebugDisplay
	fprintf(stderr,"Max Hough Space Value: %f\n",max_value);
	houghspace_show = HoughImg / max_value*255;	//for display
	cv::cvtColor(HoughImg/max_value, houghspace_show, CV_GRAY2RGB);	//for display
	imwrite("HoughImgraw.png",houghspace_show*256);
#endif

	////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////Updata LaneWidth////////////////////////////////////////

	CvPoint upper_max = FindMax
		(HoughImg,
		max_value_pt.x-(int)(lane_width+lane_width_range),
		max_value_pt.x-(int)(lane_width-lane_width_range),
		(int)(max_value_pt.y-0.5/thetaStep),
		(int)(max_value_pt.y+0.5/thetaStep),
		(float)(0.15*max_value));
	CvPoint lower_max = FindMax
		(HoughImg,
		max_value_pt.x+(int)(lane_width-lane_width_range),
		max_value_pt.x+(int)(lane_width+lane_width_range),
		max_value_pt.y-(int)(0.5/thetaStep),
		max_value_pt.y+(int)(0.5/thetaStep),
		(float)(0.15*max_value));

	int lane_width_temp = 0;
	if(max_value_pt.x != 0 || max_value_pt.y != 0)
	{
		if ((upper_max.x != 0 || upper_max.y != 0) && (lower_max.x != 0 || lower_max.y != 0))
		{
			//calc lane width (3 lanes detected)
			lane_width_counter = 0;
			if(HoughImg.at<float>(upper_max.x,upper_max.y)>=HoughImg.at<float>( lower_max.x, lower_max.y) )
				lane_width_temp = max_value_pt.x - upper_max.x;
			else
				lane_width_temp = lower_max.x - max_value_pt.x;
		}
		else if (upper_max.x != 0 || upper_max.y != 0)
		{
			// calc lane width (upper lane and max lane detected)
			lane_width_counter = 0;
			lane_width_temp = max_value_pt.x - upper_max.x;
		}
		else if (lower_max.x != 0 || lower_max.y != 0)
		{
			// calc lane width (lower lane and max lane detected)
			lane_width_counter = 0;
			lane_width_temp = lower_max.x - max_value_pt.x;
		}
		else
		{
			lane_width_range = (float)(0.5/gridsize/rStep);
			lane_width_counter++;
			if(lane_width_counter>5)
				lane_width_range = (float)(1.5/gridsize/rStep);
		}
	}
	else
	{
		lane_width_range = 1.5/gridsize/rStep;
	}
	if((lane_width_temp >= lane_width_min) && (lane_width_temp <= lane_width_max)) lane_width = (float)lane_width_temp;
	
#if DebugDisplay
	fprintf(stderr,"lanewidth = %f\n",lane_width*gridsize);
#endif
	////////////////////////////////////////////////////////////////////////////////////

	if(max_value<15000.0) return;

	/////////////////////////////Find Curve///////////////////////////////////////////
	Mat HoughImgbak;
	HoughImg.copyTo(HoughImgbak);
	CvPoint max_value_pts[1000];
	int x0 = max_value_pt.x, y0 = max_value_pt.y;
	int nn = 0;
	vector<CvPoint> max_value_pt_list;
	max_value_pt_list.resize(0);
	while(HoughImg.at<float>(x0,y0)>max_value*0.35&&nn<1000)
	{
		HoughImg.at<float>(x0,y0) = -1;
		max_value_pts[nn].x=x0;
		max_value_pts[nn].y=y0;
		nn++;

#if DebugDisplay
		cout<<"l: x0 = "<<x0<<", y0 = "<<y0<<", ("<<houghspace_show.cols<<","<<houghspace_show.rows<<")"<<endl;
		if(!houghspace_show.empty())
			//if(x0>0 && x0<houghspace_show.cols&&y0>0&&y0<houghspace_show.rows)
				houghspace_show.at<Vec3f>(x0,y0) = Vec3f(255,0,0);	//for display
				//houghspace_show.at<Vec3f>(x0,y0) = Vec3f((float)((nn*20%255)/255.0),0,0);	//for display
#endif	


		float max_temp = 1;
		int x0_temp = x0, y0_temp = y0;
		if(x0+1>0&&x0+1<HoughImg.rows&&y0>0&&y0<HoughImg.cols&&HoughImg.at<float>(x0+1,y0)>=1.1*max_temp)
		{
			max_temp = HoughImg.at<float>(x0+1,y0);
			x0_temp = x0+1, y0_temp = y0;
		}
		if(x0+1>0&&x0+1<HoughImg.rows&&y0-1>0&&y0-1<HoughImg.cols&&HoughImg.at<float>(x0+1,y0-1)>=max_temp)
		{
			max_temp = HoughImg.at<float>(x0+1,y0-1);
			x0_temp = x0+1, y0_temp = y0-1;
		}
		if(x0>0&&x0<HoughImg.rows&&y0-1>0&&y0-1<HoughImg.cols&&HoughImg.at<float>(x0,y0-1)>=max_temp)
		{
			max_temp = HoughImg.at<float>(x0,y0-1);
			x0_temp = x0, y0_temp = y0-1;
		}
		if(x0-1>0&&x0-1<HoughImg.rows&&y0-1>0&&y0-1<HoughImg.cols&&HoughImg.at<float>(x0-1,y0-1)>=max_temp)//error 0905
		{
			max_temp = HoughImg.at<float>(x0-1,y0-1);
			x0_temp = x0-1, y0_temp = y0-1;
		}
		if(x0-1>0&&x0-1<HoughImg.rows&&y0>0&&y0<HoughImg.cols&&HoughImg.at<float>(x0-1,y0)>=1.1*max_temp)
		{
			max_temp = HoughImg.at<float>(x0-1,y0);
			x0_temp = x0-1, y0_temp = y0;
		}
		x0 = x0_temp; y0 = y0_temp;
	}

	for(int i = nn;i >= 0; i--)
	{
		max_value_pt_list.push_back(max_value_pts[i]);
	}
	//max_value_pt_list.push_back(max_value_pt);
	//HoughImgbak.copyTo(HoughImg);
	x0 = max_value_pt.x, y0 = max_value_pt.y;
	HoughImg.at<float>(x0,y0) = max_value;
	nn = 0;
	while(HoughImg.at<float>(x0,y0)>max_value*0.35&&nn<1000)
	{
		HoughImg.at<float>(x0,y0) = -1;
		max_value_pts[nn].x=x0;
		max_value_pts[nn].y=y0;
		nn++;

		
#if DebugDisplay

		cout<<"r: x0 = "<<x0<<", y0 = "<<y0<<", ("<<houghspace_show.cols<<","<<houghspace_show.rows<<")"<<endl;
		if(!houghspace_show.empty())
			//if(x0>0 && x0<houghspace_show.cols&&y0>0&&y0<houghspace_show.rows)
				//houghspace_show.at<Vec3f>(x0,y0) = Vec3f(0,(float)((nn*20%255)/255.0),0);	//for display
				houghspace_show.at<Vec3f>(x0,y0) = Vec3f(255,0,0);	//for display
#endif

		float max_temp = 1;
		int x0_temp = x0, y0_temp = y0;
		if(x0+1>0 && x0+1<HoughImg.rows && y0>0 && y0<HoughImg.cols && HoughImg.at<float>(x0+1,y0)>=1.1*max_temp)
		{
			max_temp = HoughImg.at<float>(x0+1,y0);
			x0_temp = x0+1, y0_temp = y0;
		}
		if(x0+1>0 && x0+1<HoughImg.rows && y0+1>0 && y0+1<HoughImg.cols && HoughImg.at<float>(x0+1,y0+1)>=max_temp)
		{
			max_temp = HoughImg.at<float>(x0+1,y0+1);
			x0_temp = x0+1, y0_temp = y0+1;
		}
		if(x0>0 && x0<HoughImg.rows && y0+1>0 && y0+1<HoughImg.cols && HoughImg.at<float>(x0,y0+1)>=max_temp)
		{
			max_temp = HoughImg.at<float>(x0,y0+1);
			x0_temp = x0, y0_temp = y0+1;
		}
		if(x0-1>0 && x0-1<HoughImg.rows && y0+1>0 && y0+1<HoughImg.cols && HoughImg.at<float>(x0-1,y0+1)>=max_temp)
		{
			max_temp = HoughImg.at<float>(x0-1,y0+1);
			x0_temp = x0-1, y0_temp = y0+1;
		}
		if(x0-1>0 && x0-1<HoughImg.rows && y0>0 && y0<HoughImg.cols && HoughImg.at<float>(x0-1,y0)>=1.1*max_temp)
		{
			max_temp = HoughImg.at<float>(x0-1,y0);
			x0_temp = x0-1, y0_temp = y0;
		}
		x0 = x0_temp; y0 = y0_temp;
	}
		
	for(int i = 0;i < nn; i++)
	{
		max_value_pt_list.push_back(max_value_pts[i]);
	}
		
	HoughImgbak.copyTo(HoughImg);
	maxymin = max_value_pt_list[1].y;
	maxymax = max_value_pt_list[max_value_pt_list.size()-2].y;
#if DebugDisplay
	cout<<"ymin = "<<maxymin<<", ymax = "<<maxymax<<endl;
#endif

	max_value_pt_listf.resize(0);

	for(int i = maxymin; i <= maxymax; i++)
	{
		double xtemp = 0;
		double xnum = 0;
		for(int j = 1; j < max_value_pt_list.size(); j++)
		{
			if(max_value_pt_list[j].y==i)
			{
				xtemp += HoughImg.at<float>(max_value_pt_list[j].x,max_value_pt_list[j].y) * max_value_pt_list[j].x;
				xnum += HoughImg.at<float>(max_value_pt_list[j].x,max_value_pt_list[j].y);
			}
		}
		if(xnum!=0)
		{
			xtemp = xtemp/xnum;
			max_value_pt_listf.push_back(cvPoint2D64f(xtemp,i));
		}
	}

	isCurve = (max_value_pt_listf.size()>20);
	//////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////Find Lane////////////////////////////////////////
	for(int i = 0; i < 101; i++)
	{
		upper_extend[i] = cvPoint(0,0);
		lower_extend[i] = cvPoint(0,0);
	}
	
	upper_extend[0].x = max_value_pt.x;
	upper_extend[0].y = max_value_pt.y;
	lower_extend[0].x = max_value_pt.x;
	lower_extend[0].y = max_value_pt.y;

	//////Search upper////////////
	int xmin = (int)(max_value_pt.x - 0.6/gridsize/rStep);
	int xmax = (int)(max_value_pt.x + 0.6/gridsize/rStep);
	int ymin, ymax;
	if(isCurve)
	{
		ymin = max_value_pt.y;
		ymax = max_value_pt.y;
	}
	else
	{
		ymin = (int)(max_value_pt.y - 0.5/thetaStep);
		ymax = (int)(max_value_pt.y + 0.5/thetaStep);
	}
	int n = 1;
	float max_hough_pt = 0;
	while(xmax>(-0.5*L-rMin)&&n<101)
	{
		if(upper_extend[n-1].x||upper_extend[n-1].y)
		{
			xmin = (int)(upper_extend[n-1].x - 0.6/gridsize/rStep - lane_width);
			xmax = (int)(upper_extend[n-1].x + 0.6/gridsize/rStep - lane_width);
		}
		else
		{
			xmin -= (int)lane_width;
			xmax -= (int)lane_width;
		}
#if DebugDisplay
		rectangle	//for display
		(
		houghspace_show,
		cvPoint(ymin,xmin),
		cvPoint(ymax,xmax),
		CV_RGB(0,0.5,0.5),1,8,0
		);
#endif
		upper_extend[n] = FindMax(HoughImg, xmin, xmax, ymin, ymax, (float)(0.2*max_value));
		n++;
	}
	/////////////////////////////

	//////Search lower////////////
	xmin = (int)(max_value_pt.x - 0.6/gridsize/rStep);
	xmax = (int)(max_value_pt.x + 0.6/gridsize/rStep);
	if(isCurve)
	{
		ymin = max_value_pt.y;
		ymax = max_value_pt.y;
	}
	else
	{
		ymin = (int)(max_value_pt.y - 0.5/thetaStep);
		ymax = (int)(max_value_pt.y + 0.5/thetaStep);
	}
	n = 1;
	while(xmin<(1.2*L-rMin)&&n<101)
	{
		if(lower_extend[n-1].x||lower_extend[n-1].y)
		{
			xmin = (int)(lower_extend[n-1].x - 0.6/gridsize/rStep + lane_width);
			xmax = (int)(lower_extend[n-1].x + 0.6/gridsize/rStep + lane_width);
		}
		else
		{
			xmin += (int)lane_width;
			xmax += (int)lane_width;
		}
#if DebugDisplay
		rectangle	//for display
		(
		houghspace_show,
		cvPoint(ymin,xmin),
		cvPoint(ymax,xmax),
		CV_RGB(0,0.5,0.5),1,8,0
		);
#endif
		lower_extend[n] = FindMax(HoughImg, xmin, xmax, ymin, ymax, (float)0.2*max_value);
		n++;
	}
	/////////////////////////////

	///////////////Add lost lane//////////////////
	int upper_max_index = -1;
	int lower_max_index = -1;
	for(int i = 0; i < 101; i++)
	{
		if(upper_extend[i].x!=0 || upper_extend[i].y!=0) upper_max_index = i;
		if(lower_extend[i].x!=0 || lower_extend[i].y!=0) lower_max_index = i;
	}

	for(int i = 1; i < 101; i++)
	{
		if(upper_extend[i].x==0 && upper_extend[i].y==0 && i<upper_max_index)
		{
			int jm;
			for(jm = i; jm < 101; jm++)
				if(upper_extend[jm].x!=0 || upper_extend[jm].y!=0) break;
			for(int j = i; j < jm; j++)
			{
				float stepx = (float)((upper_extend[jm].x - upper_extend[i-1].x)/(jm - i + 1.0));
				float stepy = (float)((upper_extend[jm].y - upper_extend[i-1].y)/(jm - i + 1.0));
				upper_extend[j].x = (int)(upper_extend[i-1].x + (j - i + 1) * stepx);
				upper_extend[j].y = (int)(upper_extend[i-1].y + (j - i + 1) * stepy);
			}
		}
		if(lower_extend[i].x==0 && lower_extend[i].y==0 && i<lower_max_index)
		{
			int jm;
			for(jm = i; jm < 101; jm++)
				if(lower_extend[jm].x!=0 || lower_extend[jm].y!=0) break;
			for(int j = i; j < jm; j++)
			{
				float stepx = (float)((lower_extend[jm].x - lower_extend[i-1].x)/(jm - i + 1.0));
				float stepy = (float)((lower_extend[jm].y - lower_extend[i-1].y)/(jm - i + 1.0));
				lower_extend[j].x = (int)(lower_extend[i-1].x + (j - i + 1) * stepx);
				lower_extend[j].y = (int)(lower_extend[i-1].y + (j - i + 1) * stepy);
			}
		}
	}

	
	CvPoint upper_losed_lane = cvPoint(upper_extend[upper_max_index].x-(int)lane_width,upper_extend[upper_max_index].y);
	CvPoint upper_check_lane = cvPoint(upper_extend[upper_max_index].x-(int)(0.5*lane_width),upper_extend[upper_max_index].y);

	int upper_check = 0;
	for (int y=1;y<VeloImage.cols-1;y++)
	{
		float r1 = (float)(rMin + rStep*upper_check_lane.x);
		float theta = (float)(pi*(thetaMin + thetaStep*max_value_pt.y)/180.0);
		int x1 = (int)((r1 - ((float)y)*sin(theta))/cos(theta));
		if(x1 >= 1 && x1 < VeloImage.rows - 1)
			if(VeloImage.at<uchar>(y,x1))
				upper_check ++;
	}
	if(upper_check > 0.7*L)
	{
		upper_extend[upper_max_index+1].x = upper_losed_lane.x;
		upper_extend[upper_max_index+1].y = upper_losed_lane.y;
	}

	CvPoint lower_losed_lane = cvPoint(lower_extend[lower_max_index].x+(int)lane_width,lower_extend[lower_max_index].y);
	CvPoint lower_check_lane = cvPoint(lower_extend[lower_max_index].x+(int)(0.5*lane_width),lower_extend[lower_max_index].y);
	int lower_check = 0;
	for (int y=1;y<VeloImage.cols-1;y++)
	{
		float r2 = (float)(rMin + rStep*lower_check_lane.x);
		float theta = (float)(pi*( thetaMin + thetaStep*max_value_pt.y)/180.0);
		int x2 = (int)((r2 - ((float)y)*sin(theta))/cos(theta));
		if(x2 >= 1 && x2 < VeloImage.rows-1)
			if(VeloImage.at<uchar>(y,x2))
				lower_check ++;
	}
	if(lower_check > 0.7*L)
	{
		lower_extend[lower_max_index+1].x = lower_losed_lane.x;
		lower_extend[lower_max_index+1].y = lower_losed_lane.y;
	}
	

	if (max_value_pt.x != 0 || max_value_pt.y != 0)
	{
		for(int i = 100; i >= 1; i--)
		{
			if(upper_extend[i].x!=0 || upper_extend[i].y!=0)
			{
				detect_lanes.push_back(upper_extend[i]);
				//cout<<upper_extend[i].x<<",";
			}
		}

		detect_lanes.push_back(max_value_pt);
		//cout<<max_value_pt.x<<";";

		for(int i = 1; i < 101; i++)
		{
			if(lower_extend[i].x!=0 || lower_extend[i].y!=0)
			{
				detect_lanes.push_back(lower_extend[i]);
				//cout<<lower_extend[i].x<<",";
			}
		}
		//cout<<endl;
	}


	///////////////Find Double Lane///////////////
	for(int n = 0; n < 101; n++)
	{
		if(n < detect_lanes.size())
		{
			
#if DebugDisplay
			rectangle	//for display
			(
			houghspace_show,
			cvPoint(detect_lanes[n].y - 2,detect_lanes[n].x - (int)(0.45/gridsize/rStep)),
			cvPoint(detect_lanes[n].y + 2,detect_lanes[n].x - (int)(0.15/gridsize/rStep)),
			CV_RGB(0.1,0.4,0.5),1,8,0
			);
#endif
			CvPoint ptemp1 = FindMax
				(
				HoughImg,
				detect_lanes[n].x - (int)(0.45/gridsize/rStep),
				detect_lanes[n].x - (int)(0.15/gridsize/rStep),
				detect_lanes[n].y - 1,//0.1/thetaStep;
				detect_lanes[n].y + 1,//0.1/thetaStep;
				(float)(0.1*max_value)
				);
			
#if DebugDisplay
			rectangle	//for display
			(
			houghspace_show,
			cvPoint(detect_lanes[n].y - 2,detect_lanes[n].x + (int)(0.15/gridsize/rStep)),
			cvPoint(detect_lanes[n].y + 2,detect_lanes[n].x + (int)(0.45/gridsize/rStep)),
			CV_RGB(0.1,0.4,0.5),1,8,0
			);
#endif
			CvPoint ptemp2 = FindMax
				(
				HoughImg,
				detect_lanes[n].x + (int)(0.15/gridsize/rStep),
				detect_lanes[n].x + (int)(0.45/gridsize/rStep),
				detect_lanes[n].y - 1,//0.1/thetaStep
				detect_lanes[n].y + 1,//0.1/thetaStep
				(float)(0.1*max_value)
				);
				

			if((ptemp1.x||ptemp1.y)&&(ptemp2.x||ptemp2.y))
			{
				if(HoughImg.at<float>(ptemp1.x,ptemp1.y)>HoughImg.at<float>(ptemp2.x,ptemp2.y))
				{
					detect_lanes.insert(detect_lanes.begin()+n,cvPoint(ptemp1.x,ptemp1.y));
				}
				else
				{
					detect_lanes.insert(detect_lanes.begin()+n+1,cvPoint(ptemp2.x,ptemp2.y));
				}
				n++;
			}
			else if(ptemp1.x||ptemp1.y)
			{
				detect_lanes.insert(detect_lanes.begin()+n,cvPoint(ptemp1.x,ptemp1.y));
				n++;
			}
			else if(ptemp2.x||ptemp2.y)
			{
				detect_lanes.insert(detect_lanes.begin()+n+1,cvPoint(ptemp2.x,ptemp2.y));
				n++;
			}
		}
	}
	//////////////////////////////////////////////////////

	StopLaneFound = false;
	
#if DebugDisplay
	///**************************for display*************************
	fprintf(stderr, "Detected Lanes = %d\n",detect_lanes.size());

	//display Hough space
	//houghspace_show = HoughImg / max_value*255;
	//cv::cvtColor(HoughImg/max_value, houghspace_show, CV_GRAY2RGB);
	//max value point
	circle(houghspace_show,cvPoint(max_value_pt.y,max_value_pt.x),4,CV_RGB(1,0,0),-1,CV_AA,0);
	//upper search box
	rectangle
	(
	houghspace_show,
	cvPoint(max_value_pt.y-(int)(0.5/thetaStep),max_value_pt.x-(int)(lane_width-lane_width_range)),
	cvPoint(max_value_pt.y+(int)(0.5/thetaStep),max_value_pt.x-(int)(lane_width+lane_width_range)),
	CV_RGB(0.5,0.5,0),1,8,0
	);
	circle(houghspace_show,cvPoint(upper_max.y,upper_max.x),4,CV_RGB(1,1,0),-1,CV_AA,0);
	//lower search box
	rectangle
	(
	houghspace_show,
	cvPoint(max_value_pt.y-(int)(0.5/thetaStep),max_value_pt.x+(int)(lane_width-lane_width_range)),
	cvPoint(max_value_pt.y+(int)(0.5/thetaStep),max_value_pt.x+(int)(lane_width+lane_width_range)),
	CV_RGB(0.5,0.5,0),1,8,0
	);
	circle(houghspace_show,cvPoint(lower_max.y,lower_max.x),4,CV_RGB(1,1,0),-1,CV_AA,0);
	for(int i=0; i<101; i++)
	{
		if(upper_extend[i].x||upper_extend[i].y)
		{
			//upper extended search box
			circle(houghspace_show,cvPoint(upper_extend[i].y,upper_extend[i].x),2,CV_RGB(0,1,1),-1,CV_AA,0);
		}		
		if(lower_extend[i].x||lower_extend[i].y)
		{
			//lower extended search box
			circle(houghspace_show,cvPoint(lower_extend[i].y,lower_extend[i].x),2,CV_RGB(0,1,1),-1,CV_AA,0);
		}
	}
	//circle(HoughImg,cvPoint(stoplaney,stoplanex),2,CV_RGB(128,128,128),-1,CV_AA,0);
	imwrite("HoughImg.png",houghspace_show*256);
	//imshow("HoughImg",houghspace_show);
	//imwrite("HoughImgbgrtemp.bmp",HoughImgbgr);
#endif
	}



	
	typedef struct SortPoint
	{
		int x;
		int y;
		bool operator < (SortPoint & b)
		{
			return y < b.y;
		}
	};
	struct Lane
	{
		float r;
		float theta;
		int type;
		float believe;
		bool isCurve;
		CvPoint points[31];
	};
	int LanesNum;
	Lane Lanes[101];
	void LaneRecognition(cv::InputArray _HoughImg, cv::InputArray IPM_lane)
	{
		Mat HoughImg = _HoughImg.getMat();
		Mat ipm_lane_img = IPM_lane.getMat();
		CvPoint LanePoint[1000];
		//publish laneinfo
		LanesNum = detect_lanes.size();
		for(int i = 0; i < 101; i++)
		{		
			Lanes[i].r = 0;
			Lanes[i].theta = 0;
			Lanes[i].believe = 0;
			Lanes[i].type = LANETYP_NONE;
		}
		
		Mat ipm_lane_img1 = Mat::zeros(L,L,CV_8UC3);
		for(int i = 0; i < detect_lanes.size(); i++)
		{
			float r = (float)(rMin+rStep*detect_lanes[i].x);
			float theta = (float)(pi*(thetaMin+thetaStep*detect_lanes[i].y)/180.0);
		
			if(!isCurve)
			{
				for(int yy = 0; yy <= L; yy++)
				{
					LanePoint[yy].x = (int)(((float)r-(float)(yy)*sin(theta))/cos(theta));
					LanePoint[yy].y = yy;
				}
				Lanes[i].isCurve = false;
			}
			else
			{
				int step = (int)(((int)(max_value_pt_listf.size()) - 10) / 30);
				if(step < 1)step = 1;
				list<SortPoint> points;
				for(int j = 1;j < max_value_pt_listf.size()-10; j+=step)
				{
					double r1 = rMin+rStep*max_value_pt_listf[j].x + detect_lanes[i].x - max_value_pt.x;
					double theta1 = pi*(thetaMin+thetaStep*max_value_pt_listf[j].y)/180.0;
					double r2 = rMin+rStep*max_value_pt_listf[j+10].x + detect_lanes[i].x - max_value_pt.x;
					double theta2 = pi*(thetaMin+thetaStep*max_value_pt_listf[j+10].y)/180.0;

					double l2 = (r1-r2*cos(theta2-theta1))/sin(theta2-theta1);
					SortPoint temp;
					cv::line(ipm_lane_img,cv::Point2d((int)(((float)r1-(float)(0)*sin(theta1))/cos(theta1)),0),cv::Point2d((int)(((float)r1-(float)(L)*sin(theta1))/cos(theta1)),L),CV_RGB(255.0*j/max_value_pt_listf.size(),0,255.0-255.0*j/max_value_pt_listf.size()));
					temp.x = (int)(cos(theta2)*r2 + sin(theta2)*l2);
					temp.y = (int)(sin(theta2)*r2 - cos(theta2)*l2);
					points.push_back(temp);
				}
				points.sort();
				CvPoint px, pxold = cvPoint(-1,-1);
				int yy = 0;
				for(list<SortPoint>::iterator iter = points.begin();iter != points.end(); ++iter)
				{
					px.x = static_cast<SortPoint>(*iter).x;
					px.y = static_cast<SortPoint>(*iter).y;
					if(pxold.x!=-1)
					{
						int dy = px.y - pxold.y;
						int dx = px.x - pxold.x;
						while(yy>=pxold.y && yy < px.y && yy>=0 && yy <= L)
						{
							LanePoint[yy].x = (int)((yy - pxold.y)*dx/dy+pxold.x);
							LanePoint[yy].y = yy;
							yy++;
						}
					}
					else
					{
						double r1 = rMin+rStep*max_value_pt_listf[max_value_pt_listf.size()-1].x + detect_lanes[i].x - max_value_pt.x;
						double theta1 = pi*(thetaMin+thetaStep*max_value_pt_listf[max_value_pt_listf.size()-1].y)/180.0;
						while(yy < px.y && yy>=0 && yy <= L)
						{
							LanePoint[yy].x = (int)(((float)r1-(float)(yy)*sin(theta1))/cos(theta1));
							LanePoint[yy].y = yy;
							yy++;
						}
					}
					pxold.x = px.x;
					pxold.y = px.y;
				}
				double r1 = rMin+rStep*max_value_pt_listf[0].x + detect_lanes[i].x - max_value_pt.x;
				double theta1 = pi*(thetaMin+thetaStep*max_value_pt_listf[0].y)/180.0;
				for(; yy <= L; yy++)
				{
					LanePoint[yy].x = (int)(((float)r1-(float)(yy)*sin(theta1))/cos(theta1));
					LanePoint[yy].y = yy;
				}
				Lanes[i].isCurve = true;
			}
			
			for(int j = 0, n = 0; n < 31; j += L/30.0, n++)
			{
				Lanes[i].points[n].x = LanePoint[j].x;
				Lanes[i].points[n].y = LanePoint[j].y;
			}

	#if DebugDisplay
			cout<<"Lane "<<i<<": r = "<<r*gridsize<<", theta = "<<theta/pi*180<<endl;
	#endif
				/////////////////////\C5ж\CF\D0\E9\CF\DF/////////////////////////////////

			int white_l[1000];
			int black_l[1000];
			int l_temp = 0;
			int white_n = 0, black_n = 0;
			bool iswhite = false;
			Mat IPMLaneHSV;
			ipm_lane_img.convertTo(IPMLaneHSV,CV_RGB2HSV);
			float H = 0, S = 0, V = 0, N = 0;

			for(int j = 0; j < L; j++)
			{
				if(LanePoint[j].y<ipm_lane_img.cols && LanePoint[j].y>=0 && LanePoint[j].x<ipm_lane_img.rows && LanePoint[j].x>=0)
				{
					float avgH = 0, avgS = 0, avgV = 0, avgN = 0;
					for(int dx = -1; dx <= 1; dx++)
					{
						if(IPMLaneHSV.at<Vec3b>(LanePoint[j].y,LanePoint[j].x+dx)[2]>0)
						{
							avgH += IPMLaneHSV.at<Vec3b>(LanePoint[j].y,LanePoint[j].x+dx)[0]*2;
							avgS += IPMLaneHSV.at<Vec3b>(LanePoint[j].y,LanePoint[j].x+dx)[1];
							avgV += IPMLaneHSV.at<Vec3b>(LanePoint[j].y,LanePoint[j].x+dx)[2];
							avgN++;
						}
					}
					avgH = avgN==0?0:avgH/avgN;
					avgS = avgN==0?0:avgS/avgN;
					avgV = avgN==0?0:avgV/avgN;

					if(avgV>0)
					{
						H += avgH;
						S += avgS;
						V += avgV;
						N++;
	#if DebugDisplay
						//ipm_lane_img1.at<Vec3b>(LanePoint[j].y,LanePoint[j].x)=Vec3b(j*255/L,255,0);
						ipm_lane_img1.at<Vec3b>(LanePoint[j].y,LanePoint[j].x)=Vec3b(255,255,255);
	#endif
						if(iswhite)
						{
							l_temp++;
						}
						else
						{
							iswhite = true;
							if(l_temp>=5)
							{
								black_l[black_n] = l_temp;
								black_n++;
								l_temp = 1;
							}
							else
							{
								l_temp += white_l[white_n-1];
								white_n--;
							}
						}
					}
					else
					{
						if(iswhite)
						{
							iswhite = false;
							if(l_temp>=5)
							{
								white_l[white_n] = l_temp;
								white_n++;
								l_temp = 1;
							}
							else
							{
								l_temp += black_l[black_n-1];
								black_n--;
							}
						}
						else
						{
							l_temp++;
						}
					}
				}
			}

			float avgl = 0;
			int avgn = 0;
			for(int nn = 0;nn<white_n;nn++)
			{
				if(white_l[nn]>10)
				{
					avgl+=white_l[nn];
					avgn++;
				}
			}
			avgl = (avgn==0?0:avgl/avgn);
			float MSD = 0;
			for(int nn = 0;nn<white_n;nn++)
			{
				if(white_l[nn]>10)
				{
					MSD+= (avgl - white_l[nn])*(avgl - white_l[nn]);
				}
			}
			MSD = (avgn<=1?1000:sqrtf(MSD/avgn));
			H = N==0?0:H/N;
			S = N==0?0:S/N;
			V = N==0?0:V/N;
	#if DebugDisplay
			cout<<"AVGw = "<<avgl*gridsize<<" MSDw = "<<MSD*gridsize<<endl;
			cout<<"H = "<<H<<" S = "<<S<<" V = "<<V<<endl;
	#endif
			/////////////////////////////////////////////////////////////////

			if(detect_lanes[i].x>=0 && detect_lanes[i].x<HoughImg.rows && detect_lanes[i].y>=0&&detect_lanes[i].y<HoughImg.cols)
			{
				if(H>0 && H<80 && S>90 && V<120)
				{
					if(avgl>1.2/gridsize&&avgl<4/gridsize&&MSD<1.5/gridsize)
					{
						Lanes[i].r = r;
						Lanes[i].theta = theta;
						Lanes[i].believe = HoughImg.at<float>(detect_lanes[i].x,detect_lanes[i].y);
						Lanes[i].type = LANETYP_SDY;
					}
					else
					{
						Lanes[i].r = r;
						Lanes[i].theta = theta;
						Lanes[i].believe = HoughImg.at<float>(detect_lanes[i].x,detect_lanes[i].y);
						Lanes[i].type = LANETYP_SSY;
					}
				}
				else
				{
					if(avgl>1.2/gridsize&&avgl<4/gridsize&&MSD<1.5/gridsize)
					{
						Lanes[i].r = r;
						Lanes[i].theta = theta;
						Lanes[i].believe = HoughImg.at<float>(detect_lanes[i].x,detect_lanes[i].y);
						Lanes[i].type = LANETYP_SDW;
					}
					else
					{
						Lanes[i].r = r;
						Lanes[i].theta = theta;
						Lanes[i].believe = HoughImg.at<float>(detect_lanes[i].x,detect_lanes[i].y);
						Lanes[i].type = LANETYP_SSW;
					}
				}

			}
			else
			{
				Lanes[i].r = r;
				Lanes[i].theta = theta;
				Lanes[i].believe = 0;
				Lanes[i].type = LANETYP_SSW;

			}
		}
		
		imwrite("ipm_lane_img1.png",ipm_lane_img1);
		imwrite("ipm_lane_img2.png",ipm_lane_img);
		//cout<<"LaneRecognition complete"<<endl;
	}

	
	void FindStopLane(cv::InputArray _HoughImg, cv::InputArray _SobelImggrayMasked)
	{
		Mat SobelImggrayMasked = _SobelImggrayMasked.getMat();
		Mat HoughImg = _HoughImg.getMat();
		//cout<<LanesNum<<endl;
		if(LanesNum == 0)
		{
			StopLaneFound = false;
			return;
		}

		Mat StopLaneMask =Mat::zeros(L,L,CV_8UC1);
		if(LanesNum == 1)
		{
			StopLaneMask = 255;
		}
		else
		{
			Point pt[1][63];
			int arr[1]; 
			arr[0] = 63;

			for(int i = 0; i < 31; i++)
			{
				pt[0][i]=Point(Lanes[0].points[i].x,Lanes[0].points[i].y);
			}

			for(int i = 30; i >= 0; i--)
			{
				pt[0][61 - i]=Point(Lanes[LanesNum-1].points[i].x,Lanes[LanesNum-1].points[i].y);
			}

			const Point* ppt[1] = {pt[0]};
			fillPoly(StopLaneMask,ppt,arr,1,CV_RGB(255,255,255));
		}

		imwrite("StopLaneMask.png",StopLaneMask);

		/////////////////////////Find Stop Lane///////////////////
		double thetastopline = 0;
	
		for(int i = 0;i<LanesNum;i++)
		{
			thetastopline += Lanes[i].theta;
		}
		//thetastopline = thetastopline/detect_lanes.size()+90.0/thetaStep;
		thetastopline = (thetastopline/LanesNum*180.0/pi - thetaMin + 90.0)/thetaStep;
		CvPoint stoplanep;
		Mat SobelImggrayMaskedStop;
		SobelImggrayMasked.copyTo(SobelImggrayMaskedStop,StopLaneMask);

		imshow("SobelImggrayMaskedStop",SobelImggrayMaskedStop);
		if(isCurve)
		{
			GetHoughSpace(SobelImggrayMaskedStop, HoughImg, (float)(maxymin*thetaStep+thetaMin-3.0+90.0), (float)(maxymax*thetaStep+thetaMin+3.0+90.0), (int)(-L), (int)(1.2*L));
			//houghspace_show.
			rectangle	//for display
			(
			houghspace_show,
			cvPoint((int)(maxymin+90.0/thetaStep-3.0/thetaStep),1),
			cvPoint((int)(maxymax+90.0/thetaStep+3.0/thetaStep),HoughImg.rows),
			CV_RGB(0.1,0.4,0.5),1,8,0
			);
			stoplanep = FindMax(HoughImg,
				1, 
				HoughImg.rows,
				(int)(maxymin+90.0/thetaStep-3.0/thetaStep), 
				(int)(maxymax+90.0/thetaStep+3.0/thetaStep), 
				(float)(0.01*max_value));
		}
		else
		{
			GetHoughSpace(SobelImggrayMaskedStop, HoughImg, (float)(thetastopline*thetaStep+thetaMin-3.0), (float)(thetastopline*thetaStep+thetaMin+3.0), (int)(-L), (int)(1.2*L));
			//imwrite("HoughImgWithStopLine.png",HoughImg/100);
			rectangle	//for display
			(
			houghspace_show,
			cvPoint((int)(thetastopline-3.0/thetaStep),1),
			cvPoint((int)(thetastopline+3.0/thetaStep),HoughImg.rows),
			CV_RGB(0.1,0.4,0.5),1,8,0
			);
			stoplanep = FindMax(HoughImg,
				1,
				HoughImg.rows,
				(int)(thetastopline-3.0/thetaStep), 
				(int)(thetastopline+3.0/thetaStep), 
				(float)(0.01*max_value));
		}
		if(stoplanep.x!=0||stoplanep.y!=0)
		{
			StopLaneFound = true;
		
			Lanes[LanesNum].r = (float)(rMin+rStep*stoplanep.x);
			Lanes[LanesNum].theta = (float)(pi*(thetaMin+thetaStep*stoplanep.y)/180.0);
			Lanes[LanesNum].type = LANETYP_STOP;
			Lanes[LanesNum].isCurve = false;
			Lanes[LanesNum].believe = HoughImg.at<float>(stoplanep.x,stoplanep.y);
			int xx = 0;
			for(int i = 0; i <= 30; i++)
			{
				Lanes[LanesNum].points[i].x = xx;
				Lanes[LanesNum].points[i].y = (int)(((float)Lanes[LanesNum].r-(float)(xx)*cos(Lanes[LanesNum].theta))/sin(Lanes[LanesNum].theta));
				xx+= L/30;
			}
			LanesNum++;
		}
		else
		{
			StopLaneFound = false;
		}
	#if DebugDisplay
			cout<<"thetastopline = "<<thetastopline<<endl;
			cout<<"LanesNum = "<<LanesNum<<endl;
			cout<<"Stop Lane: ("<< Lanes[LanesNum].r<<", "<<Lanes[LanesNum].theta<<")"<<endl;
			//cout<<"Stop Lane: ("<< stoplanep.x<<", "<<stoplanep.y<<")"<<endl;
			circle(houghspace_show,cvPoint(stoplanep.y,stoplanep.x),2,CV_RGB(0,1,1),-1,CV_AA,0);
		
	#endif
		//////////////////////////////////////////////////////////
	}

	void DrawLane(cv::InputOutputArray _IpmImg)
	{
		Mat IpmImg = _IpmImg.getMat();
		//publish laneinfo
		//cv::resize(IpmImg,IpmImg,cv::Size(750,750);
		for(int i = 0; i < 101; i++)
		{
			if(Lanes[i].type == LANETYP_NONE)
				return;
			CvPoint px,pxold;
			pxold.x=Lanes[i].points[0].x;
			pxold.y=Lanes[i].points[0].y;
			for(int j = 1; j < 31; j++)
			{
				//if(Lanes[i].believe<=3000)continue;
				px.x = Lanes[i].points[j].x;
				px.y = Lanes[i].points[j].y;
				switch(Lanes[i].type)
				{
				case LANETYP_SSW:
					line(IpmImg,pxold,px,CV_RGB(255,0,0),2,CV_AA);
					break;
				case LANETYP_SSY:
					line(IpmImg,pxold,px,CV_RGB(255,180,0),2,CV_AA);
					break;
				case LANETYP_SDW:
					line(IpmImg,pxold,px,CV_RGB(255,0,0),1,CV_AA);
					break;
				case LANETYP_SDY:
					line(IpmImg,pxold,px,CV_RGB(255,180,0),1,CV_AA);
					break;
				case LANETYP_STOP:
					line(IpmImg,pxold,px,CV_RGB(0,255,255),4,CV_AA);
					break;
				}
				pxold.x = px.x;
				pxold.y = px.y;
				//line(ipm_img,cvPoint((int)r1/cos(theta1),0),cvPoint((int)((r1-L*sin(theta1))/cos(theta1)),L),CV_RGB(0,i*255/max_value_pt_list.size(),0),1,CV_AA);
			}
		}
		IpmImg.copyTo(_IpmImg);
	}


	double GetMatADD(cv::InputArray InImg,cv::OutputArray OutImg)
	{
		Mat inimg = InImg.getMat();
		float valtemp;
		double sumtemp = 0;
		float valadd[900];
		float valr[720];
		for (int i=0; i<inimg.cols; i++)
		{   sumtemp = 0;
			for (int j=0; j<inimg.rows; j++)
			{
				valtemp = inimg.at<float>(j,i);
				sumtemp+=valtemp*valtemp/10000.0;
			}
			if(IsLikelyToFront == 1)
				valadd[i] = (1.0 - abs(i-inimg.cols/2.0)/inimg.cols) * sumtemp/10000.0;
			else
				valadd[i] = sumtemp/10000.0;
		}

		float maxx = 0;
		int maxxindex = 0;
		for(int i = 0; i <inimg.cols; i++)
		{
			if(valadd[i]>maxx)
			{
				maxx = valadd[i];
				maxxindex = i;
			}
		}

		for(int i = 0; i <inimg.cols; i++)
			valadd[i] = valadd[i]/maxx;


		for(int j = 0; j <inimg.rows; j++) 
			valr[j] = inimg.at<float>(j,maxxindex);



		double theta = maxxindex*thetaStep+thetaMin;
		

		int xnow = L/2.0*(sin(theta*3.1415926/180.0)+cos(theta*3.1415926/180.0))/rStep - rMin;
		
		//for(int j = xnow - 1; j > 0; j--)
		//	valr[j] = (valr[j+1]*3.0+valr[j])/4.0;
		//for(int j = xnow + 1; j < 720; j++)
		//	valr[j] = (valr[j-1]*3.0+valr[j])/4.0;

		float maxr = 0;
		int maxrindex = 0;
		for(int j = 0; j < inimg.rows; j++)
		{
			if(valr[j]>maxr)
			{
				maxr = valr[j];
				maxrindex = j;
			}
		}
		for(int j = 0; j <inimg.rows; j++)
			valr[j] = valr[j]/255.0/150.0;

		int up = LeftDefaultDis;
		int down = RightDefaultDis;
		double maxup = 0;
		double maxdown = 0;

		for(int j = xnow - 1; j > 0; j--)
			if(valr[j]>maxup)
				maxup = valr[j];
		for(int j = xnow + 1; j < 720; j++)
			if(valr[j]>maxdown)
				maxdown = valr[j];
		upbelieve = maxr*maxup/30000.0;
		downbelieve = maxr*maxdown/30000.0;
		cout<<"\r\nupbelieve = "<<maxr*maxup/30000.0;
		cout<<"\r\ndownbelieve = "<<maxr*maxdown/30000.0;

		for(int j = xnow - 1; j > 0; j--)
		{
			if(valr[j]>LeftEdgeThresh)
			{
				up = xnow - j;
				break;
			}
		}
		for(int j = xnow + 1; j < 720; j++)
		{
			if(valr[j]>RightEdgeThresh)
			{
				down = j - xnow;
				break;
			}
		}		

		if(upold == -1) upold = up;
		if(downold == -1) downold = down;
		if(abs(thetaold-theta)>150.0)
		{
			double temp = upold;
			upold=downold;
			downold = temp;
		}

		//upold = filterUP.cal(up);
		//downold = filterDOWN.cal(down);
		upold = (upold * 9.0 + filterUP.cal(up)) / 10.0;
		downold = (downold * 9.0 + filterDOWN.cal(down)) / 10.0;




		///////////////////////////for display////////////////////////////////////////
		/*
		for(int j = 0; j <inimg.rows; j++)
		{
			int tempval = valr[j]*100.0;
			for(int i = 0; i < tempval; i ++) inimg.at<float>(j,i) = 100000;
			inimg.at<float>(j,maxxindex) = 100000;
		}

		for(int i = 0; i <inimg.cols-1; i++)
		{
			int tempval =  valadd[i]*100.0;
			for(int j = 0; j < tempval; j ++) inimg.at<float>(j,i) = valadd[i]*50000.0;
			inimg.at<float>(xnow,i) = 100000;
			inimg.at<float>(xnow - upold,i) = 100000;
			inimg.at<float>(xnow + downold,i) = 100000;
		}
		*/
		/////////////////////////////////////////////////////////////////////////////

		inimg.copyTo(OutImg);
		
		thetaold = (thetaold*0.0+theta)/1.0;
		return theta;
	}


	double GetMatADDSide(cv::InputArray InImg,cv::OutputArray OutImg, double theta, double* _pos)
	{
		Mat inimg = InImg.getMat();
		double valtemp;
		double sumtemp = 0;
		double valmax[900];
		double valr1[720];

		double theta1 = theta + 90.0;
		if(theta1>90.0) theta1-=180.0;
		else if(theta1<-90.0) theta1+=180.0;
		int theta1index = (theta1-thetaMin)/thetaStep;

		for (int i=0; i<inimg.cols; i++)
		{
			sumtemp = 0;
			valmax[i] = 0;
			for (int j=0; j<inimg.rows; j++)
			{
				valtemp = inimg.at<float>(j,i);
				if(valmax[i] < valtemp) valmax[i] = valtemp;
				sumtemp+=valtemp*valtemp/10000.0;
			}
		}

		for (int i=1; i<inimg.cols; i++) valmax[i] = (valmax[i-1]*2.0+ valmax[i])/3.0;


		int maxxindex1 = 0;
		double maxval1 = 0;
		double sumvalue = 0;
		for(int i = theta1index-CrossAngleRange/thetaStep; i<theta1index+CrossAngleRange/thetaStep; i++)
		{
			int itemp = i;
			if(itemp < 0) itemp = 180.0/thetaStep + itemp;
			else if(itemp > 180.0/thetaStep) itemp = itemp - 180.0/thetaStep;
			if(valmax[itemp] > maxval1)
			{
				maxval1 = valmax[itemp];
				maxxindex1 = itemp;
			}
			sumvalue += valmax[itemp];
		}
		double avrval = sumvalue/(30.0/thetaStep);


		for(int j = 0; j <inimg.rows; j++) 
			valr1[j] = inimg.at<float>(j,maxxindex1);
		double max_valr1 = 0;
		double index_valr1 = 0;
		for(int j = 0; j <inimg.rows; j++) 
		{
			if(max_valr1 < valr1[j])
			{
					max_valr1 = valr1[j];
					index_valr1 = j;
			}
		}
				
		if(avrval/maxval1 > CrossDetThresh) maxxindex1 = -1;

		double Thetatemp;
		if(maxxindex1!=-1)
			Thetatemp = maxxindex1*thetaStep+thetaMin;
		else 
			Thetatemp = -65536;

		int xnow1 = L/2.0*(sin(Thetatemp*3.1415926/180.0)+cos(Thetatemp*3.1415926/180.0))/rStep - rMin;

		*_pos = xnow1 - index_valr1;

		cout<<"avrval/maxval1 = "<<avrval/maxval1<<"\n";
		


		///////////////////////////for display////////////////////////////////////////
		/*		
		for(int j = 0; j <inimg.rows; j++)
			valr1[j] = valr1[j]/255.0/150.0;

		if(maxxindex1!=-1)
		{
			for(int j = 0; j <inimg.rows; j++)
			{
				int tempval = valr1[j]*100.0;
				for(int i = 0; i < tempval; i ++) inimg.at<float>(j,inimg.cols-1-i) = 100000;
				inimg.at<float>(j,maxxindex1) = 20000;
			}		
		}
		for(int i = 0; i <inimg.cols-1; i++)
		{
			int tempval =  valmax[i]/1000.0;
			for(int j = 0; j < tempval; j ++) inimg.at<float>(j,i) = 100000;
			inimg.at<float>(xnow1,i) = 20000;
			inimg.at<float>(index_valr1,i) = 100000;
		}
		*/
		/////////////////////////////////////////////////////////////////////////////

		inimg.copyTo(OutImg);


		return Thetatemp;
	}

	CLaneDetection()
	{
		rMin = -1.2*L;
		rMax = 1.2*L;
		//float rStep = (rMax-rMin)/500.0;
		rStep = 1;
		thetaMin = -90;
		thetaMax = 90;
		thetaStep = 0.2;
		lane_width = 3.0/gridsize/rStep;
		lane_width_range = lane_width*0.333;
		lane_width_max = 6/gridsize/rStep;
		lane_width_min = 2/gridsize/rStep;
		int lane_width_counter = 0;
		thetaold = 0;
		upold = -1;
		downold = -1;
	}
};


class CIPM
{
	Mat a;
	Mat b;
	Mat c;
	Mat d;
	Mat N;
	Mat M;

	int m;
	int n;

public:
	void CreatIPM(cv::InputArray InImg,cv::OutputArray OutImg)
	{
		Mat inimg = InImg.getMat();
		Mat outimg = OutImg.getMat();
		for(int j=0;j<L;j++)
		{
			for(int i=0;i<L;i++)
			{
				if(M.at<int>(j,i)>=0 && M.at<int>(j,i)<m-1)
				{
					if(N.at<int>(j,i)>=0 && N.at<int>(j,i)<n-1)
					{ 

						outimg.at<Vec3b>(L-j-1,L-i-1)=
							inimg.at<Vec3b>(M.at<int>(j,i), N.at<int>(j,i)) * a.at<float>(j,i) +
							inimg.at<Vec3b>(M.at<int>(j,i), N.at<int>(j,i)+1) * b.at<float>(j,i) +
							inimg.at<Vec3b>(M.at<int>(j,i)+1, N.at<int>(j,i)) * c.at<float>(j,i) +
							inimg.at<Vec3b>(M.at<int>(j,i)+1, N.at<int>(j,i)+1) * d.at<float>(j,i);
						//IPM.at<Vec3b>(L-j,L-i)= img.at<Vec3b>(M, N);
					}
					else if(N.at<int>(j,i)==n-1)
					{
						outimg.at<Vec3b>(L-j-1,L-i-1)=
							inimg.at<Vec3b>(M.at<int>(j,i), N.at<int>(j,i)) * a.at<float>(j,i) +
							inimg.at<Vec3b>(M.at<int>(j,i), 0) * b.at<float>(j,i) +
							inimg.at<Vec3b>(M.at<int>(j,i)+1, N.at<int>(j,i)) * c.at<float>(j,i) +
							inimg.at<Vec3b>(M.at<int>(j,i)+1, 0) * d.at<float>(j,i);
						cout<<N.at<int>(j,i)<<endl;
					}
					else
					{
						cout<<N.at<int>(j,i)<<endl;
					}
				}


			}
		}
	}

	CIPM(float h, float yaw, float pitch, float roll, int rows, int cols)
	{
		m = rows;
		n = cols;
		Mat R_yaw = Mat(3,3,CV_32FC1);
		R_yaw = (Mat_<float>(3,3)<<
			cos((-90+yaw)*pi/180),sin((-90+yaw)*pi/180),0,
			-sin((-90+yaw)*pi/180),cos((-90+yaw)*pi/180),0,
			0,0,1);
		Mat R_pitch = Mat(3,3,CV_32FC1);
		R_pitch = (Mat_<float>(3,3)<<
			1,0,0,
			0,cos(pitch*pi/180),sin(pitch*pi/180),
			0,-sin(pitch*pi/180),cos(pitch*pi/180));
		Mat R_roll = Mat(3,3,CV_32FC1);
		R_roll = (Mat_<float>(3,3)<<
			cos(roll*pi/180),0,-sin(roll*pi/180),
			0,1,0,
			sin(roll*pi/180),0,cos(roll*pi/180));
		Mat R = R_roll*R_pitch*R_yaw.t();
		Mat T1 = (Mat_<float>(3,1)<<0,0,-h);
		Mat T = T1*Mat::ones(1,L*L,CV_32FC1);
		//T = T*T1;

		Mat Groundpt = Mat(3,L*L,CV_32FC1);
		for(int j=0;j<L;j++)
		{
			for(int i=0;i<L;i++)
			{
				Groundpt.at<float>(0,j*L+i) = -range + gridsize * i;
				Groundpt.at<float>(1,j*L+i) = -range*2/3 + gridsize * j;
				Groundpt.at<float>(2,j*L+i) = 0;
			}
		}

		Mat LadybugPt =R*(Groundpt+T);


		Mat NMf = Mat(2,L*L,CV_32FC1);

		for(int j=0;j<L*L;j++)
		{
			float _x=LadybugPt.at<float>(0,j);
			float _y=LadybugPt.at<float>(1,j);
			float _z=LadybugPt.at<float>(2,j);
			float r=sqrt(_x*_x + _y*_y + _z*_z);
			float theta = atan(_y/_x);
			if(_x<0 && _y>=0)
			{
				theta = pi+theta;
			}
			else if(_x<0 && _y<0)
			{
				theta = -pi+theta;
			}
			float phi = acos(_z/r);
			if(phi>pi/2)phi=-phi+pi/2;
			NMf.at<float>(0,j)=(n/2+theta/pi*n/2);
			NMf.at<float>(1,j)=(m/2-phi/(pi/2)*m/2);
		}

		a = Mat(L,L,CV_32FC1);
		b = Mat(L,L,CV_32FC1);
		c = Mat(L,L,CV_32FC1);
		d = Mat(L,L,CV_32FC1);
		N = Mat(L,L,CV_32SC1);
		M = Mat(L,L,CV_32SC1);

		for(int j=0;j<L;j++)
		{
			for(int i=0;i<L;i++)
			{
				N.at<int>(j,i) = (int)NMf.at<float>(0,j*L+i);
				M.at<int>(j,i) = (int)NMf.at<float>(1,j*L+i);

				if(M.at<int>(j,i)>=0 && M.at<int>(j,i)<m-1)
				{
					if(N.at<int>(j,i)>=0 && N.at<int>(j,i)<n)
					{ 
						float dx = NMf.at<float>(0,j*L+i) - N.at<int>(j,i), dy = NMf.at<float>(1,j*L+i) - M.at<int>(j,i);

						a.at<float>(j,i)=(1-dx) * (1-dy);
						b.at<float>(j,i)=dx * (1-dy);
						c.at<float>(j,i)=(1-dx) * dy;
						d.at<float>(j,i)=dx * dy;
					}
				}
			}
		}	
	}
};


CLaneDetection ldetection = CLaneDetection();

int edgemain(cv::InputArray InImg, double* Angle, double* leftdis, double* rightdis, double* leftb, double* rightb, double* AngleLeft, double* AngleRight, double* left_fwd_dist, double* right_fwd_dist, double* left_fwd_b, double* right_fwd_b)
{
	//ROS_ERROR("cv_bridge exception");
	//fprintf(stdout,"A\n");
	long frame_counter = 0;
	double theta = 0;
	CMidFilter thetafilter;
	int frameN = 0;

	frame_counter++;
			
	fprintf(stdout,"\r\nFrame: %ld\t", frame_counter);

	Mat img = InImg.getMat();
	if(img.empty())
	{
		fprintf(stderr, "Can not load image\n");
		return -1;
	}

	Mat lidarbgr[3];
	split(img,lidarbgr);
	cv::threshold(lidarbgr[0],lidarbgr[0],200,255,0);
	//imshow("lidarbgr[0]",lidarbgr[0]);

	Mat HoughImg;
	Mat MaskL, MaskR, HoughImgL, HoughImgR;
	//Mat HoughImgBGR;
	HoughImg = Mat::zeros((int)((ldetection.rMax-ldetection.rMin)/ldetection.rStep),(int)((ldetection.thetaMax-ldetection.thetaMin)/ldetection.thetaStep),CV_32FC1);
	ldetection.HoughSpaceInit();
	ldetection.GetHoughSpace(lidarbgr[0],HoughImg,-90,90,(int)(-1.2*L),(int)(1.2*L));

	double thetanew = ldetection.GetMatADD(HoughImg,HoughImg);

	if(theta < -5 && thetanew > 5)
		thetanew -= 180;
	else if(thetanew < -5 && theta > 5)
		thetanew += 180;
	theta = thetafilter.cal(thetanew);
	//theta = (theta*1.0 + thetanew)/2.0;
	//theta = 
	if(theta<-90)theta+=180;
	else if(theta>90)theta-=180;
	theta = thetanew;
	fprintf(stdout, "THETA=%f\n", theta);
	fprintf(stdout, "THETAnew=%f\n", thetanew);			
//imwrite("HoughImg1.bmp",HoughImg/50.0);


			//////////////////////////////////////////////////////
/////////////////////////////chalu//////////////////////////////////////////

				
	Point ptl[1][4];
	int arrl[1];
	arrl[0] = 4;
	ptl[0][0]=Point(150-250*sin(theta*3.1415926/180.0)-ldetection.upold*sin((theta+90.0)*3.1415926/180.0),
			150+250*cos(theta*3.1415926/180.0)+ldetection.upold*cos((theta+90.0)*3.1415926/180.0));
	ptl[0][1]=Point(150+250*sin(theta*3.1415926/180.0)-ldetection.upold*sin((theta+90.0)*3.1415926/180.0),
			150-250*cos(theta*3.1415926/180.0)+ldetection.upold*cos((theta+90.0)*3.1415926/180.0));
	ptl[0][2]=Point(150+250*sin(theta*3.1415926/180.0)+220*sin((theta+90.0)*3.1415926/180.0),
			150-250*cos(theta*3.1415926/180.0)-220*cos((theta+90.0)*3.1415926/180.0));
	ptl[0][3]=Point(150-250*sin(theta*3.1415926/180.0)+220*sin((theta+90.0)*3.1415926/180.0),
			150+250*cos(theta*3.1415926/180.0)-220*cos((theta+90.0)*3.1415926/180.0));

	const Point* pptl[1] = {ptl[0]};
	Mat lidarbgr0l;
	lidarbgr[0].copyTo(lidarbgr0l);
	fillPoly(lidarbgr0l,pptl,arrl,1,CV_RGB(0,0,0));
	//imshow("lidarbgr0l",lidarbgr0l);
	ldetection.GetHoughSpace(lidarbgr0l,HoughImgL,theta+90.0-15.0,theta+90.0+15.0,(int)(-1.2*L),(int)(1.2*L));
		
	Point ptr[1][4];
	int arrr[1];
	arrr[0] = 4;
	ptr[0][0]=Point(150-250*sin(theta*3.1415926/180.0)+ldetection.downold*sin((theta+90.0)*3.1415926/180.0),
			150+250*cos(theta*3.1415926/180.0)-ldetection.downold*cos((theta+90.0)*3.1415926/180.0));
	ptr[0][1]=Point(150+250*sin(theta*3.1415926/180.0)+ldetection.downold*sin((theta+90.0)*3.1415926/180.0),
			150-250*cos(theta*3.1415926/180.0)-ldetection.downold*cos((theta+90.0)*3.1415926/180.0));
	ptr[0][2]=Point(150+250*sin(theta*3.1415926/180.0)-220*sin((theta+90.0)*3.1415926/180.0),
			150-250*cos(theta*3.1415926/180.0)+220*cos((theta+90.0)*3.1415926/180.0));
	ptr[0][3]=Point(150-250*sin(theta*3.1415926/180.0)-220*sin((theta+90.0)*3.1415926/180.0),
			150+250*cos(theta*3.1415926/180.0)+220*cos((theta+90.0)*3.1415926/180.0));

	const Point* pptr[1] = {ptr[0]};
	Mat lidarbgr0r;
	lidarbgr[0].copyTo(lidarbgr0r);
	fillPoly(lidarbgr0r,pptr,arrr,1,CV_RGB(0,0,0));
	//imshow("lidarbgr0r",lidarbgr0r);
	ldetection.GetHoughSpace(lidarbgr0r,HoughImgR,theta+90.0-15.0,theta+90.0+15.0,(int)(-1.2*L),(int)(1.2*L));

	double posL, posR;
	double theta1 = ldetection.GetMatADDSide(HoughImgL,HoughImgL,theta, &posL);
	double theta2 = ldetection.GetMatADDSide(HoughImgR,HoughImgR,theta, &posR);
			
///////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


	//imshow("HoughImg",HoughImg/50000.0);
	//imshow("HoughImgL",HoughImgL/50000.0);
	//imshow("HoughImgR",HoughImgR/50000.0);
			

	cv::line(img,cv::Point
		(
		150-150*sin(theta*3.1415926/180.0),
		150+150*cos(theta*3.1415926/180.0)
		),
		cv::Point
		(
		150+150*sin(theta*3.1415926/180.0),
		150-150*cos(theta*3.1415926/180.0)
		),
		CV_RGB(255,0,0),1,CV_AA);			
			
	if(theta1!=-65536)
	{
		cv::line(img,cv::Point(150,150),
			cv::Point(
			150-posL*sin((theta1+90.0)*3.1415926/180.0),
			150+posL*cos((theta1+90.0)*3.1415926/180.0)
			),
			CV_RGB(255,0,255),1,CV_AA);
		if(theta1>0)
			cv::line(img,cv::Point
				(
				150-posL*sin((theta1+90.0)*3.1415926/180.0)-150*sin(theta1*3.1415926/180.0),
				150+posL*cos((theta1+90.0)*3.1415926/180.0)+150*cos(theta1*3.1415926/180.0)
					),
				cv::Point(
				150-posL*sin((theta1+90.0)*3.1415926/180.0),
				150+posL*cos((theta1+90.0)*3.1415926/180.0)
				),
				CV_RGB(255,0,255),2,CV_AA);
		else
			cv::line(img,cv::Point
				(
				150-posL*sin((theta1+90.0)*3.1415926/180.0)+150*sin(theta1*3.1415926/180.0),
				150+posL*cos((theta1+90.0)*3.1415926/180.0)-150*cos(theta1*3.1415926/180.0)
				),
				cv::Point(
				150-posL*sin((theta1+90.0)*3.1415926/180.0),
				150+posL*cos((theta1+90.0)*3.1415926/180.0)
				),
				CV_RGB(255,0,255),2,CV_AA);
	}
	if(theta2!=-65536)
	{
		cv::line(img,cv::Point(150,150),
				cv::Point(
				150-posR*sin((theta2+90.0)*3.1415926/180.0),
				150+posR*cos((theta2+90.0)*3.1415926/180.0)
				),
				CV_RGB(255,255,0),1,CV_AA);
		if(theta2>0)
			cv::line(img,cv::Point
				(
				150-posR*sin((theta2+90.0)*3.1415926/180.0)+150*sin(theta2*3.1415926/180.0),
				150+posR*cos((theta2+90.0)*3.1415926/180.0)-150*cos(theta2*3.1415926/180.0)
				),
				cv::Point(
				150-posR*sin((theta2+90.0)*3.1415926/180.0),
				150+posR*cos((theta2+90.0)*3.1415926/180.0)
				),
				CV_RGB(255,255,0),2,CV_AA);
		else
			cv::line(img,cv::Point
				(
				150-posR*sin((theta2+90.0)*3.1415926/180.0)-150*sin(theta2*3.1415926/180.0),
				150+posR*cos((theta2+90.0)*3.1415926/180.0)+150*cos(theta2*3.1415926/180.0)
				),
				cv::Point(
				150-posR*sin((theta2+90.0)*3.1415926/180.0),
				150+posR*cos((theta2+90.0)*3.1415926/180.0)
				),
				CV_RGB(255,255,0),2,CV_AA);
	}

	cv::line(img,cv::Point
		(
		150+ldetection.downold*sin((theta+90.0)*3.1415926/180.0),
		150-ldetection.downold*cos((theta+90.0)*3.1415926/180.0)
		),
		cv::Point(
		150-ldetection.upold*sin((theta+90.0)*3.1415926/180.0),
		150+ldetection.upold*cos((theta+90.0)*3.1415926/180.0)
		),
		CV_RGB(255,0,0),1,CV_AA);
	
	
	cv::line(img,cv::Point
		(
		150-250*sin(theta*3.1415926/180.0)+ldetection.downold*sin((theta+90.0)*3.1415926/180.0),
		150+250*cos(theta*3.1415926/180.0)-ldetection.downold*cos((theta+90.0)*3.1415926/180.0)
		),
		cv::Point(
		150+250*sin(theta*3.1415926/180.0)+ldetection.downold*sin((theta+90.0)*3.1415926/180.0),
		150-250*cos(theta*3.1415926/180.0)-ldetection.downold*cos((theta+90.0)*3.1415926/180.0)
		),
		CV_RGB(255,0,0),2,CV_AA);
	cv::line(img,cv::Point
		(
		150-250*sin(theta*3.1415926/180.0)-ldetection.upold*sin((theta+90.0)*3.1415926/180.0),
		150+250*cos(theta*3.1415926/180.0)+ldetection.upold*cos((theta+90.0)*3.1415926/180.0)
		),
		cv::Point(
		150+250*sin(theta*3.1415926/180.0)-ldetection.upold*sin((theta+90.0)*3.1415926/180.0),
		150-250*cos(theta*3.1415926/180.0)+ldetection.upold*cos((theta+90.0)*3.1415926/180.0)
		),
		CV_RGB(255,0,0),2,CV_AA);
//		imshow("lidarbgr[0]",img);
	//sent messages
	*Angle = theta;
	*leftdis = ldetection.upold;
	*rightdis = ldetection.downold;
	*leftb = ldetection.upbelieve;
	*rightb = ldetection.downbelieve;

	*AngleLeft = theta1;
	*AngleRight = theta2;
	*left_fwd_dist = posL;
	*right_fwd_dist = posR;
	*left_fwd_b = 15;
	*right_fwd_b = 15;
	

	return 0;
}



void imageConvert(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

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

	edgemain(cv_ptr->image, &Angle, &leftdis, &rightdis, &leftb, &rightb, &AngleLeft, &AngleRight, &left_fwd_dist, &right_fwd_dist, &left_fwd_b, &right_fwd_b);
	
	//publish RoadEdge messages
	roadedge.Angle = Angle;
	roadedge.leftdis = leftdis*gridsize;
	roadedge.rightdis = rightdis*gridsize;
	roadedge.leftb = leftb;
	roadedge.rightb = rightb;
	roadedge.AngleLeft = AngleLeft;
	roadedge.AngleRight = AngleRight;
	roadedge.left_fwd_dist = left_fwd_dist*gridsize;
	roadedge.right_fwd_dist = right_fwd_dist*gridsize;
	roadedge.left_fwd_b = left_fwd_b;
	roadedge.right_fwd_b = right_fwd_b;
	RoadEdge_pub.publish(roadedge);

	//publish laneinfo messages
	laneinfo.lane_num = 2;
	laneinfo.r[0] = (36.055*cos(atan(2.0/3.0)+Angle*pi/180.0)-leftdis*gridsize)*50.0;
	laneinfo.r[1] = (36.055*cos(atan(2.0/3.0)+Angle*pi/180.0)+rightdis*gridsize)*50.0;
	//laneinfo.r[0] = (50.0*cos(atan(4.0/3.0)-Angle*pi/180.0)-leftdis*0.2)*50.0;
	//laneinfo.r[1] = (50.0*cos(atan(4.0/3.0)-Angle*pi/180.0)+rightdis*0.2)*50.0;
	laneinfo.theta[0] = Angle*pi/180.0;
	laneinfo.theta[1] = Angle*pi/180.0;
	LaneInfo_pub.publish(laneinfo);

	//publish gis messages
	float midlane_r = laneinfo.r[0] + 0.5*(laneinfo.r[1]-laneinfo.r[0]);
	float midlane_theta = laneinfo.theta[0];
	float y1 = 0;
	float x1 = midlane_r/cos(midlane_theta);
	float y2 = 3000.0;
	float x2 = (midlane_r+y2*sin(midlane_theta))/cos(midlane_theta);
	float length = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	CvPoint2D32f direction = cvPoint2D32f((x2-x1)/length*10.0,(y2-y1)/length*10.0);
	for(int j=0;j<301;j++)
	{
		gis.x[j] = (int)(x1 + direction.x*j);
		gis.y[j] = (int)(y1 + direction.y*j);
	}
	GIS_pub.publish(gis);

	//publish LaneInfoV2 messages
	midlane_r = laneinfo.r[0];
	midlane_theta = laneinfo.theta[0];
	y1 = 0;
	x1 = midlane_r/cos(midlane_theta);
	y2 = 3000.0;
	x2 = (midlane_r+y2*sin(midlane_theta))/cos(midlane_theta);
	length = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	direction.x = (x2-x1)/length*10.0;
	direction.y = (y2-y1)/length*10.0;
	for(int j=0;j<31;j++)
	{
		laneinfov2.points_x[j] = (int)(x1 + direction.x*j*10.0);
		laneinfov2.points_y[j] = (int)(y1 + direction.y*j*10.0);
	}
	midlane_r = laneinfo.r[1];
	midlane_theta = laneinfo.theta[1];
	y1 = 0;
	x1 = midlane_r/cos(midlane_theta);
	y2 = 3000.0;
	x2 = (midlane_r+y2*sin(midlane_theta))/cos(midlane_theta);
	length = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	direction.x = (x2-x1)/length*10.0;
	direction.y = (y2-y1)/length*10.0;
	for(int j=0;j<31;j++)
	{
		laneinfov2.points_x[j+31] = (int)(x1 + direction.x*j*10.0);
		laneinfov2.points_y[j+31] = (int)(y1 + direction.y*j*10.0);
	}
	laneinfov2.type[0] = 1;
	laneinfov2.type[1] = 1;
	laneinfov2.beliefe[0] = 15;
	laneinfov2.beliefe[1] = 15;
	LaneInfoV2_pub.publish(laneinfov2);
	

	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(5);  
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "roadedge_node");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	image_transport::Subscriber image_sub_;
	image_sub_ = it_.subscribe("heightmap", 1, &imageConvert); 
	RoadEdge_pub = nh_.advertise<in2_msgs::RoadEdge>("RoadEdge",1);
	LaneInfo_pub = nh_.advertise<in2_msgs::LaneInfo>("LaneInfoV3",1);
	GIS_pub = nh_.advertise<in2_msgs::UdpGeneral>("gisV2",1);
	LaneInfoV2_pub = nh_.advertise<in2_msgs::LaneInfoV2>("LaneInfoV2_edge",1);
	cv::namedWindow(OPENCV_WINDOW,0);
	ros::spin();
	return 0;
}