#include "Filter.h"

#define   PI     3.1415926

int obsCostMapFilter(Mat CurFrm, Mat LastFrm, Mat &out, Point3d CurIns, Point3d LastIns)
{
	int inertia = 40; // 0~100，数值越大，滤波的惯性越大

	Size CurSz = CurFrm.size();
	int high = CurSz.height;
	int wide = CurSz.width;
	
	Size LastSz = LastFrm.size();
	
	for(int y=0; y<high; y++)
	{
		for(int x=0; x<wide; x++)
		{
			int CurCost = CurFrm.at<uchar>(y,x);
			Point Cur3000 = convMapTo3000(Point(x,y), high);
			
			CvPoint2D64f ena = conv3000toENU(Cur3000.x, Cur3000.y, CurIns.x, CurIns.y, CurIns.z);
			CvPoint Last3000tmp = convENUto3000(ena.x, ena.y, LastIns.x, LastIns.y, LastIns.z);
			Point Last3000;
			Last3000.x = Last3000tmp.x;
			Last3000.y = Last3000tmp.y;
			/*
			Point Last3000;
			doRT(Cur3000.x, Cur3000.y, Last3000.x, Last3000.y,
				(LastIns.x-CurIns.x), (LastIns.y-CurIns.y),
				CurIns.z, LastIns.z);
			*/
			
			Point LastMap = conv3000ToMap(Last3000, high);
			int LastCost = 0;
			bool a = (LastMap.x>0) && (LastMap.x<LastSz.width);
			bool b = (LastMap.y>0) && (LastMap.y<LastSz.height);
			if(a && b)
			{
				LastCost = LastFrm.at<uchar>(LastMap.y, LastMap.x);
			}
			
			int NewCost = (CurCost*(100-inertia) + LastCost*inertia) / 100;
			out.at<uchar>(y,x) = NewCost;
		}
	}

	return 0;
}

void doRT(int src_x, int src_y, int &dst_x, int &dst_y, double T_NEx, double T_NEy, double azimuth_src, double azimuth_dst)
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
}

Point convMapTo3000(Point src_pt, int MapSize)
{
	int x = (1.0f * src_pt.x) / MapSize * 3000;
	int y = 3000 - (1.0f * src_pt.y) / MapSize * 3000;
	
	return Point(x, y);
}

Point conv3000ToMap(Point src_pt, int MapSize)
{
	int x = (1.0f * src_pt.x) / 3000 * MapSize;
	int y = MapSize - (1.0f * src_pt.y) / 3000 * MapSize;
	
	return Point(x, y);
}

CvPoint convENUto3000(double x, double y, double x0, double y0, double azimuth)
{
	double dx = x - x0;
	double dy = y - y0;
	double cosa = cos(azimuth * PI / 180.0);
	double sina = sin(azimuth * PI / 180.0);
	double resultX = dx * cosa - dy * sina;
	double resultY = dx * sina + dy * cosa;
	int finalX = (int)(resultX * 50.0 + 1500.0);
	int finalY = (int)(resultY * 50.0 + 1000.0);
	/*
	if(finalX < 0) finalX = 0;
	else if(finalX > 3000) finalX = 3000;
	if(finalY < 0) finalY = 0;
	else if(finalY > 3000) finalY = 3000;
	*/
	CvPoint pt;
	pt.x = finalX;
	pt.y = finalY;
	return pt;
}

CvPoint2D64f conv3000toENU(int x, int y, double x0, double y0, double azimuth)
{
	double resultX = (double(x) - 1500.0)/50.0;
	double resultY = (double(y) - 1000.0)/50.0;
	double cosa = cos(azimuth * PI / 180.0);
	double sina = sin(azimuth * PI / 180.0);
	double dx = resultX * cosa + resultY * sina;
	double dy = -resultX * sina + resultY * cosa;
	double finalX = dx + x0;
	double finalY = dy + y0;
	CvPoint2D64f pt;
	pt.x = finalX;
	pt.y = finalY;
	return pt;
}

