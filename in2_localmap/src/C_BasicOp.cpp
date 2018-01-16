/*
 * C_BasicOp.cpp
 *
 *  Created on: 2013-10-12
 *      Author: greensky
 */

#include "in2_localmap/C_BasicOp.h"
#include <in2_localmap/C_Param.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>


#define MAX_NEG_LIST_LEN 250000
#define MAX_NEG_LIST_LEN_2 50
using namespace cv;

bool regionGrowing(double list[],int len,int i,int &edge_1,int &edge_2,double reg_maxdist = 0.2)
//list 0.0~1.0, len 0~MAX_NEG_LIST_LEN_2, i 0~len
{
	int input_i = i;
	if( i > len )
	{
		fprintf(stderr,"ERROR: regionGrowing Input error!!\n");
		return false;
	}
	if( len > MAX_NEG_LIST_LEN_2)
	{
		fprintf(stderr,"ERROR: regionGrowing Input error!!\n");
		return false;
	}

	double J[MAX_NEG_LIST_LEN_2] = {0};
	double reg_mean = list[i];
	int reg_size = 1;

	int neg_pos = 0;
	int neg_list_i[MAX_NEG_LIST_LEN_2];
	double neg_list_3[MAX_NEG_LIST_LEN_2];

	double pixdist = 0.0;
	int neigb[2]={-1,1};
	while(pixdist<reg_maxdist&&reg_size<=len)
	{
		for(int j=0;j<2;j++)
		{
			int in = i+neigb[j];

			bool ins=(in>=0)&&(in<len);
			if(ins&&(J[in]==0))
			{
                //neg_list(Range(neg_pos,neg_pos+1),Range(0,3))...
				neg_list_i[neg_pos] = in;
				neg_list_3[neg_pos] = list[in];
                neg_pos = neg_pos+1;
                J[in] = 1;
                //neg_list(neg_pos,:) = [xn yn I(xn,yn)];
                //J(xn,yn)=1;
                if(neg_pos >= MAX_NEG_LIST_LEN_2)
                {
                	fprintf(stderr,"ERROR: in regionGrowing, not enough space");
                }
			}
		}
		double min_value;
		double min_index;
		bool first_time_flag = true;
		for(int j=0;j<neg_pos;j++)
		{
			if(first_time_flag == true)
			{
				first_time_flag = false;
				min_value = fabs(neg_list_3[j] - reg_mean);
				min_index = j;
			}
			else
			{
				if(min_value > fabs(neg_list_3[j] - reg_mean))
				{
					min_value = fabs(neg_list_3[j] - reg_mean);
					min_index = j;
				}
			}
		}
		pixdist = min_value;
		int index = min_index;
		J[i] = 2.0;
		reg_size ++;
		reg_mean= (reg_mean*((double)reg_size-1) + neg_list_3[index])/((double)reg_size);
		i = neg_list_i[index];
		neg_list_i[index] = neg_list_i[neg_pos-1];
		neg_list_3[index] = neg_list_3[neg_pos-1];
		neg_pos=neg_pos-1;
	}
	for(int j=input_i;j>=0;j--)
	{
		if(J[j] > 1)
		{
			edge_1 = j;
		}
		else
		{
			break;
		}
	}
	for(int j=input_i;j<len;j++)
	{
		if(J[j] > 1)
		{
			edge_2 = j;
		}
		else
		{
			break;
		}
	}
	return true;
}

Mat regionGrowing(Mat I,int x,int y,double reg_maxdist)
/*
	Mat I;
	I = imread("medtest.png");
	Mat J = regionGrowing(I,198,359,0.2);
	namedWindow( "medtest", 0 );
	imshow("medtest",I);
	waitKey(20);
	namedWindow( "medtest 2", 0 );
	imshow("medtest 2",J);
	waitKey(20);
*/
{
	Mat J = Mat::zeros(I.rows,I.cols,CV_8UC3);
	double reg_mean = ((double)I.at<Vec3b>(x,y)[0])/255.0;
	int reg_size = 1;

	//int neg_free = MAX_NEG_LIST_LEN;
	int neg_pos = 0;
	int neg_list_x[MAX_NEG_LIST_LEN];
	int neg_list_y[MAX_NEG_LIST_LEN];
	double neg_list_3[MAX_NEG_LIST_LEN];
	//Mat neg_list = Mat::zeros(neg_free,3,CV_64FC1);
	double pixdist = 0.0;
	int neigb[4][2]={{-1,0},{1,0},{0,-1},{0,1}};
	while(pixdist<reg_maxdist&&reg_size<I.rows*I.cols)
	{
		for(int j=0;j<4;j++)
		{
			int xn = x+neigb[j][0];
			int yn = y+neigb[j][1];

			bool ins=(xn>=0)&&(yn>=0)&&(xn<I.rows)&&(yn<I.cols);
			if(ins&&(J.at<Vec3b>(xn,yn)[0]==0))
			{
                //neg_list(Range(neg_pos,neg_pos+1),Range(0,3))...
				neg_list_x[neg_pos] = xn;
				neg_list_y[neg_pos] = yn;
				neg_list_3[neg_pos] = ((double)I.at<Vec3b>(xn,yn)[0])/255.0;
                neg_pos = neg_pos+1;
                J.at<Vec3b>(xn,yn)[0] = 1;
                //neg_list(neg_pos,:) = [xn yn I(xn,yn)];
                //J(xn,yn)=1;
                if(neg_pos >= MAX_NEG_LIST_LEN)
                {
                	fprintf(stderr,"ERROR: in regionGrowing, not enough space");
                }
			}
		}
		double min_value;
		double min_index;
		bool first_time_flag = true;
		for(int i=0;i<neg_pos;i++)
		{
			if(first_time_flag == true)
			{
				first_time_flag = false;
				min_value = fabs(neg_list_3[i] - reg_mean);
				min_index = i;
			}
			else
			{
				if(min_value > fabs(neg_list_3[i] - reg_mean))
				{

					min_value = fabs(neg_list_3[i] - reg_mean);
					min_index = i;
				}
			}
		}
		pixdist = min_value;
		int index = min_index;
		J.at<Vec3b>(x,y)[0] = 2.0;
		reg_size ++;
		reg_mean= (reg_mean*((double)reg_size-1) + neg_list_3[index])/((double)reg_size);
		x = neg_list_x[index];
		y = neg_list_y[index];
		neg_list_x[index] = neg_list_x[neg_pos-1];
		neg_list_y[index] = neg_list_y[neg_pos-1];
		neg_list_3[index] = neg_list_3[neg_pos-1];
		neg_pos=neg_pos-1;
	}
	for(int i=0;i<J.rows;i++){
		for(int j=0;j<J.cols;j++){
			if(J.at<Vec3b>(i,j)[0] > 1.0)
				J.at<Vec3b>(i,j)[0] = 255;
		}
	}
	return J;
}


double normof(double x,double y)
{
	return sqrt(x*x + y*y);
}

double vectorangle3K(double x,double y)
// return -180 ~ 0 ~ 180
{
	double angle3K =  atan2(x,y)*180.0/PI;
	return angle3K;
	return 0;
}
double vectorangle3K(int x,int y)
// return -180 ~ 0 ~ 180
{
	double angle3K =  atan2((double)x,(double)y)*180.0/PI;
	return angle3K;
}

double angle3KDiff(double angle3K_1,double angle3K_2)
//input -180~180, return 0 ~ 180
{
	if( -180.5< angle3K_1 && angle3K_1 < -180.0)
	{
		angle3K_1 = -180.0;
	}
	if( 180.0< angle3K_1 && angle3K_1 < 180.5)
	{
		angle3K_1 = 180.0;
	}
	if( -180.5< angle3K_2 && angle3K_2 < -180.0)
	{
		angle3K_2 = -180.0;
	}
	if( 180.0< angle3K_2 && angle3K_2 < 180.5)
	{
		angle3K_2 = 180.0;
	}
	if( -180.0 <= angle3K_1 && angle3K_1<=180.0 && -180.0 <= angle3K_2 && angle3K_2<=180.0)
	{
		double temp = fabs(angle3K_1 - angle3K_2);
		if(temp > 180.0)
		{
			temp = 360.0 - temp;
		}
		if(temp >=0.0)
			return temp;
		else
		{
			fprintf(stderr,"ERROR: double azimuthDiff(double azimuth_1,double azimuth_2)::%lf\n",temp);
			return -1;
		}

	}
	else
	{
		fprintf(stderr,"ERROR: angle3KDiff(double angle3K,double angle3K)::%lf,%lf \n",angle3K_1,angle3K_2);
		return -1;
	}
}

double byangle3KDiff(double byangle3K_1,double byangle3K_2)
//input -90~90 or -180~180, return 0 ~ 90
{
	if( -180.5< byangle3K_1 && byangle3K_1 < -180.0)
	{
		byangle3K_1 = -180.0;
	}
	if( 180.0< byangle3K_1 && byangle3K_1 < 180.5)
	{
		byangle3K_1 = 180.0;
	}
	if( -180.5< byangle3K_2 && byangle3K_2 < -180.0)
	{
		byangle3K_2 = -180.0;
	}
	if( 180.0< byangle3K_2 && byangle3K_2 < 180.5)
	{
		byangle3K_2 = 180.0;
	}
	if( -180.0 <= byangle3K_1 && byangle3K_1<=180.0 && -180.0 <= byangle3K_2 && byangle3K_2<=180.0)
	{
		double temp = fabs(byangle3K_1 - byangle3K_2);
		if(temp > 180.0)
		{
			temp = 360.0 - temp;
		}
		if(temp > 90.0)
		{
			temp = 180.0 - temp;
		}
		if(temp >=0.0)
			return temp;
		else
		{
			fprintf(stderr,"ERROR: double azimuthDiff(double azimuth_1,double azimuth_2)\n");
			return -1;
		}
	}
	else
	{
		fprintf(stderr,"ERROR: angle3KDiff(double angle3K,double angle3K)\n");
		return -1;
	}
}

double azimuthDiff(double azimuth_1,double azimuth_2)
//return 0 ~ 180;
{
	if( 0.0 <= azimuth_1 && azimuth_1<=360.0 && 0.0 <= azimuth_2 && azimuth_2<=360.0)
	{
		double temp = fabs(azimuth_1 - azimuth_2);
		if(temp > 180.0)
		{
			temp = 360.0 - temp;
		}
		if(temp >=0.0)
			return temp;
		else
		{
			fprintf(stderr,"ERROR: double azimuthDiff(double azimuth_1,double azimuth_2)\n");
			return -1;
		}
	}
	else
	{
		fprintf(stderr,"ERROR: double azimuthDiff(double azimuth_1,double azimuth_2)\n");
		return -1;
	}
}

double lineAngleDiff(double azimuth_1,double azimuth_2)
// return 0 ~ 90;
{
	if( 0.0 <= azimuth_1 && azimuth_1<=360.0 && 0.0 <= azimuth_2 && azimuth_2<=360.0)
	{
		double temp = fabs(azimuth_1 - azimuth_2);
		if(temp > 180.0)
		{
			temp = 360.0 - temp;
		}
		if(temp > 90.0)
		{
			temp = 180.0 - temp;
		}
		if(temp >=0.0)
		{

			//fprintf(stdout,"azimuth_1: %lf, azimuth_2: %lf, result = %lf",azimuth_1,azimuth_2,temp);
			return temp;
		}
		else
		{
			fprintf(stderr,"ERROR: double azimuthDiff(double azimuth_1,double azimuth_2)\n");
			return -1;
		}
	}
	else
	{
		fprintf(stderr,"ERROR: double azimuthDiff(double azimuth_1,double azimuth_2)\n");
		return -1;
	}
}


double interpolation(double t1,double t2,double y1,double y2,double t)
{
	if(t1 == t2)
		return 0.5*(y1+y2);
	else
		return (y2-y1)*(t-t1)/(t2-t1) + y1;
};

double interpolation_for_periodic(double t1,double t2,double y1,double y2,double t)
{
	if(y1<45.0 && y2> 315)
	{
		y1 = y1+360.0;
		return interpolation(t1,t2,y1,y2,t) - 360.0;
	}
	if(y2<45.0 && y1>315)
	{
		y2 = y2+360.0;
		return interpolation(t1,t2,y1,y2,t) - 360.0;
	}
	return interpolation(t1,t2,y1,y2,t);
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

void doRT2(double src_x,double src_y,double &dst_x,double &dst_y,double T_NEx_src,double T_NEy_src,double T_NEx_dst,double T_NEy_dst,double azimuth_src,double azimuth_dst)
{
	azimuth_src = azimuth_src*PI/180;
	azimuth_dst = azimuth_dst*PI/180;
	double T_NEx = T_NEx_dst - T_NEx_src;
	double T_NEy = T_NEy_dst - T_NEy_src;
	double Tx = 1500;
	double Ty = 1000;
	double UVx = src_x-Tx;
	double UVy = src_y-Ty;
	double NEx = cos(azimuth_src)*UVx + sin(azimuth_src)*UVy;
	double NEy = -sin(azimuth_src)*UVx + cos(azimuth_src)*UVy;
	double NEx_ = NEx - T_NEx*50;
	double NEy_ = NEy - T_NEy*50;
	double UVx_ = cos(azimuth_dst)*NEx_ - sin(azimuth_dst)*NEy_;
	double UVy_ = +sin(azimuth_dst)*NEx_ + cos(azimuth_dst)*NEy_;
	dst_x = (UVx_ + Tx);
	dst_y = (UVy_ + Ty);
};

void getPerpendicularFoot(double a,double b,double c,double src_x,double src_y,double &foot_x,double &foot_y)
{
	foot_x = + b*b*src_x - a*b*src_y - a*c;
	foot_x /= a*a+b*b;
	foot_y = - a*b*src_x + a*a*src_y - b*c;
	foot_y /= a*a+b*b;
}
void getPerpendicularFoot(double a,double b,double c,int src_x,int src_y,int &foot_x,int &foot_y)
{
	double x = (double)src_x;
	double y = (double)src_y;
	double tfoot_x = + b*b*x - a*b*y - a*c;
	tfoot_x /= a*a+b*b;
	double tfoot_y = - a*b*src_x + a*a*src_y - b*c;
	tfoot_y /= a*a+b*b;
	foot_x = (int) tfoot_x;
	foot_y = (int) tfoot_y;
}

