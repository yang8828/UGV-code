#include "Coster.h"
#include "Filter.h"
#include "const.h"

void Coster::GenerateObsCostMap()
{
	Prepare();
	Process();
	//obsCostMapFilter(LidarMaskBlur, LastLidarMaskBlur, LidarMaskFilt, CurIns, LastIns);
	LidarMaskBlur.copyTo(LidarMaskFilt);
	//LidarMaskFilt.copyTo(LastLidarMaskBlur);
	LastIns = CurIns;
	Debug();
}

void FindEdgePoint(double Angle,double dis,double *p1x,double *p1y,double *p2x,double *p2y)
{
	double angletTemp = (90.0 - Angle) * 3.1415926 / 180.0;
	double LeftOffset;
	LeftOffset = dis * 50.0;
	double cosangle = cos(angletTemp);
	double sinangle = sin(angletTemp);
	double cosangle2 = cos( Angle*3.1415926 / 180.0);
	CvPoint *points = new CvPoint[4];
	int PointsNum = 0;
	double xx,yy;
	if( (Angle>1.0&&Angle<89.0)||(Angle<-1.0&&Angle>-89.0))
	{
		xx=0.0;
		yy = sinangle / cosangle * (xx - 1500.0 + LeftOffset/cosangle2) + 1000.0;
		if(yy>=0.0&&yy<=2999.0)
		{
			points[PointsNum].x = 0.0;
			points[PointsNum++].y = yy; 
		}
		xx=2999.0;
		yy = sinangle / cosangle * (xx - 1500.0 + LeftOffset/cosangle2) + 1000.0;
		if(yy>=0.0&&yy<=2999.0)
		{
			points[PointsNum].x = 2999.0;
			points[PointsNum++].y = yy; 
		}
		yy = 0.0;
		xx = cosangle / sinangle * ( yy - 1000.0) + 1500.0 -  LeftOffset/cosangle2;
		if(xx>0.0&&xx<2999.0)
		{
			points[PointsNum].x = xx;
			points[PointsNum++].y = 0.0; 
		}
		yy = 2999.0;
		xx = cosangle / sinangle * ( yy - 1000.0) + 1500.0 -  LeftOffset/cosangle2;
		if(xx>0.0&&xx<2999.0)
		{
			points[PointsNum].x = xx;
			points[PointsNum++].y = 2999.0; 
		}
	}
	else if(Angle<=1.0 && Angle>=-1.0)
	{
		points[0].y = 0.0;
		points[0].x = 1500.0 - LeftOffset;
		points[1].y = 2999.0;
		points[1].x = 1500.0 - LeftOffset;
	}
	else
	{
		points[0].x = 0.0;
		points[0].y = 1000.0 + LeftOffset;
		points[1].x = 2999.0;
		points[1].y = 1000.0 + LeftOffset;
	}
	if(points[0].y>=points[1].y)
	{
		*p1x = points[1].x;
		*p1y = points[1].y;
		*p2x = points[0].x;
		*p2y = points[0].y;
	}
	else
	{
		*p1x = points[0].x;
		*p1y = points[0].y;
		*p2x = points[1].x;
		*p2y = points[1].y;
	}
	
	delete[] points;
}

#if 0

void Coster::GenerateLaneCostMap() {

	// 1.
	if(LaneMaskFilt.empty())
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
	LaneMaskFilt = 255;
	Mat lanePoints = Mat(2,62,CV_32FC1);
	
	// 2.
	int left_id = -1;
	if(oLaneMarks.lanemarks[0].type != oLaneMarks.lanemarks[0].TYPE_NONE &&
		oLaneMarks.lanemarks[0].type != oLaneMarks.lanemarks[0].TYPE_STOP)
		left_id = 0;
	else {
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
		return;
	}
	
	// 3.
	int right_id = -1;
	//cout <<"oLaneMarks.lanemarks.size(): " << oLaneMarks.lanemarks.size() << endl;
	for(int i = oLaneMarks.lanemarks.size() - 1; i >= 0; i--)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			right_id = i;
			break;
		}
	}
	
	// 4.
	if(left_id == right_id)
	{
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
		return;
	}
	
	// 5.
	for(int j = 0; j < oLaneMarks.lanemarks[left_id].points.size(); j++)
	{
		int x = oLaneMarks.lanemarks[left_id].points[j].x;
		int y = oLaneMarks.lanemarks[left_id].points[j].y;
		if (x < 0)                     x = 0;
		if (x > LOCAL_COOR_WIDTH - 1)  x = LOCAL_COOR_WIDTH - 1;
		if (y < 0)                     y = 0;
		if (y > LOCAL_COOR_LENGTH - 1) y = LOCAL_COOR_LENGTH - 1;
		
		lanePoints.at<float>(0,j) = (float(y) - 1000)*0.02;
		lanePoints.at<float>(1,j) = (1500 - float(x))*0.02;
	}
	
	// 6.
	for(int j = 0; j < oLaneMarks.lanemarks[right_id].points.size(); j++)
	{
		int x = oLaneMarks.lanemarks[right_id].points[oLaneMarks.lanemarks[right_id].points.size()-1-j].x;
		int y = oLaneMarks.lanemarks[right_id].points[oLaneMarks.lanemarks[right_id].points.size()-1-j].y;
		if (x < 0)                     x = 0;
		if (x > LOCAL_COOR_WIDTH - 1)  x = LOCAL_COOR_WIDTH - 1;
		if (y < 0)                     y = 0;
		if (y > LOCAL_COOR_LENGTH - 1) y = LOCAL_COOR_LENGTH - 1;
		
		lanePoints.at<float>(0,j+31) = (float(y) - 1000)*0.02;
		lanePoints.at<float>(1,j+31) = (1500 - float(x))*0.02;
	
		/*if(oLaneMarks.lanemarks[right_id].points[j].y>=0 &&
		   oLaneMarks.lanemarks[right_id].points[j].y<=2999 &&
		   oLaneMarks.lanemarks[right_id].points[j].x>=0 &&
		   oLaneMarks.lanemarks[right_id].points[j].x<=2999)
		{
			lanePoints.at<float>(0,j) = (float(oLaneMarks.lanemarks[right_id].points[j].y) - 1000)*0.02;
			lanePoints.at<float>(1,j) = (1500 - float(oLaneMarks.lanemarks[right_id].points[j].x))*0.02;
		}
		else
		{
			lanePoints.at<float>(0,j)=0;
			lanePoints.at<float>(1,j)=0;
		}*/
	}
	
	// 7.
	int x,y;
	Point pt[1][62];
	int arr[1];
	arr[0] = 62;
	for(int i=0;i<62;i++)
	{
		x = (int)(-lanePoints.at<float>(0,i) / gridsize + L / 3.0*2.0);
		y = (int)(-lanePoints.at<float>(1,i) / gridsize + L / 2.0);
		
		if (x < 0)     x = 0;
		if (x > L - 1) x = L - 1;
		if (y < 0)     y = 0;
		if (y > L - 1) y = L - 1;
		
		pt[0][i]=Point(y,x);
	}
	const Point* ppt[1] = {pt[0]};
	fillPoly(LaneMaskFilt, ppt, arr, 1, CV_RGB(0,0,0));
	
	// 8.
	int x1,y1,x2,y2;
	for(int i = 0; i < oLaneMarks.lanemarks.size(); i++)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			for(int j = 0; j < oLaneMarks.lanemarks[i].points.size()-1; j++)
			{
				double laneinfo_x1 = (double(oLaneMarks.lanemarks[i].points[j].y) - 1000)*0.02;
				double laneinfo_y1 = (1500 - double(oLaneMarks.lanemarks[i].points[j].x))*0.02;
				double laneinfo_x2 = (double(oLaneMarks.lanemarks[i].points[j+1].y) - 1000)*0.02;
				double laneinfo_y2 = (1500 - double(oLaneMarks.lanemarks[i].points[j+1].x))*0.02;
				x1 = (int)(-laneinfo_x1/ gridsize + L / 3.0*2.0);
				y1 = (int)(-laneinfo_y1/ gridsize + L / 2.0);
				x2 = (int)(-laneinfo_x2/ gridsize + L / 3.0*2.0);
				y2 = (int)(-laneinfo_y2/ gridsize + L / 2.0);
				cv::line(LaneMaskFilt,cvPoint(y1,x1),cvPoint(y2,x2),CV_RGB(255,255,255),int(3.0/4.0/gridsize));
			}
		}
	}
	cv::GaussianBlur(LaneMaskFilt,LaneMaskFilt,cv::Size((int)(1.5/gridsize)*2+1,(int)(1.5/gridsize)*2+1),(int)(1.0/gridsize),(int)(1.0/gridsize),0);
}

#endif

#if 0

void Coster::GenerateLaneCostMapPtExt() {

	// 1.
	if(LaneMaskFilt.empty())
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
	LaneMaskFilt = 255;
	Mat lanePoints = Mat(2,62,CV_32FC1);
	
	// 2.
	int left_id = -1;
	if(oLaneMarks.lanemarks[0].type != oLaneMarks.lanemarks[0].TYPE_NONE &&
		oLaneMarks.lanemarks[0].type != oLaneMarks.lanemarks[0].TYPE_STOP)
		left_id = 0;
	else {
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
		return;
	}
	
	// 3.
	int right_id = -1;
	//cout <<"oLaneMarks.lanemarks.size(): " << oLaneMarks.lanemarks.size() << endl;
	for(int i = oLaneMarks.lanemarks.size() - 1; i >= 0; i--)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			right_id = i;
			break;
		}
	}
	
	// 4.
	if(left_id == right_id)
	{
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
		return;
	}
	
	// tmp variable
	double x1, x2, x3, x4, y1, y2, y3, y4, sz;
	
	// 5.
	sz = oLaneMarks.lanemarks[left_id].points.size();
	if (oLaneMarks.lanemarks[left_id].points[0].y > oLaneMarks.lanemarks[left_id].points[sz-1].y) {
		x1 = oLaneMarks.lanemarks[left_id].points[0].x;
		y1 = oLaneMarks.lanemarks[left_id].points[0].y;
		x2 = oLaneMarks.lanemarks[left_id].points[sz-1].x;
		y2 = oLaneMarks.lanemarks[left_id].points[sz-1].y;
	} else {
		x1 = oLaneMarks.lanemarks[left_id].points[sz-1].x;
		y1 = oLaneMarks.lanemarks[left_id].points[sz-1].y;
		x2 = oLaneMarks.lanemarks[left_id].points[0].x;
		y2 = oLaneMarks.lanemarks[left_id].points[0].y;
	}
	if (x1 < 0)                     x1 = 0;
	if (x1 > LOCAL_COOR_WIDTH - 1)  x1 = LOCAL_COOR_WIDTH - 1;
	if (y1 < 0)                     y1 = 0;
	if (y1 > LOCAL_COOR_LENGTH - 1) y1 = LOCAL_COOR_LENGTH - 1;
	if (x2 < 0)                     x2 = 0;
	if (x2 > LOCAL_COOR_WIDTH - 1)  x2 = LOCAL_COOR_WIDTH - 1;
	if (y2 < 0)                     y2 = 0;
	if (y2 > LOCAL_COOR_LENGTH - 1) y2 = LOCAL_COOR_LENGTH - 1;
	y3 = LOCAL_COOR_LENGTH - 1;
	x3 = x2 + (x1 - x2) * (y3 - y2) / (y1 - y2);
	y4 = 0;
	x4 = x2 + (x1 - x2) * (y4 - y2) / (y1 - y2);
	lanePoints.at<float>(0,0) = (float(y3) - 1000)*0.02;
	lanePoints.at<float>(1,0) = (1500 - float(x3))*0.02;
	lanePoints.at<float>(0,1) = (float(y4) - 1000)*0.02;
	lanePoints.at<float>(1,1) = (1500 - float(x4))*0.02;
	/*for(int j = 0; j < oLaneMarks.lanemarks[left_id].points.size(); j++)
	{
		int x = oLaneMarks.lanemarks[left_id].points[j].x;
		int y = oLaneMarks.lanemarks[left_id].points[j].y;
		if (x < 0)                     x = 0;
		if (x > LOCAL_COOR_WIDTH - 1)  x = LOCAL_COOR_WIDTH - 1;
		if (y < 0)                     y = 0;
		if (y > LOCAL_COOR_LENGTH - 1) y = LOCAL_COOR_LENGTH - 1;
		
		lanePoints.at<float>(0,j) = (float(y) - 1000)*0.02;
		lanePoints.at<float>(1,j) = (1500 - float(x))*0.02;
	}*/
	
	// 6.
	sz = oLaneMarks.lanemarks[right_id].points.size();
	if (oLaneMarks.lanemarks[right_id].points[0].y > oLaneMarks.lanemarks[right_id].points[sz-1].y) {
		x1 = oLaneMarks.lanemarks[right_id].points[0].x;
		y1 = oLaneMarks.lanemarks[right_id].points[0].y;
		x2 = oLaneMarks.lanemarks[right_id].points[sz-1].x;
		y2 = oLaneMarks.lanemarks[right_id].points[sz-1].y;
	} else {
		x1 = oLaneMarks.lanemarks[right_id].points[sz-1].x;
		y1 = oLaneMarks.lanemarks[right_id].points[sz-1].y;
		x2 = oLaneMarks.lanemarks[right_id].points[0].x;
		y2 = oLaneMarks.lanemarks[right_id].points[0].y;
	}
	y3 = LOCAL_COOR_LENGTH - 1;
	x3 = x2 + (x1 - x2) * (y3 - y2) / (y1 - y2);
	y4 = 0;
	x4 = x2 + (x1 - x2) * (y4 - y2) / (y1 - y2);
	lanePoints.at<float>(0,2) = (float(y4) - 1000)*0.02;
	lanePoints.at<float>(1,2) = (1500 - float(x4))*0.02;
	lanePoints.at<float>(0,3) = (float(y3) - 1000)*0.02;
	lanePoints.at<float>(1,3) = (1500 - float(x3))*0.02;
	/*for(int j = 0; j < oLaneMarks.lanemarks[right_id].points.size(); j++)
	{
		int x = oLaneMarks.lanemarks[right_id].points[oLaneMarks.lanemarks[right_id].points.size()-1-j].x;
		int y = oLaneMarks.lanemarks[right_id].points[oLaneMarks.lanemarks[right_id].points.size()-1-j].y;
		if (x < 0)                     x = 0;
		if (x > LOCAL_COOR_WIDTH - 1)  x = LOCAL_COOR_WIDTH - 1;
		if (y < 0)                     y = 0;
		if (y > LOCAL_COOR_LENGTH - 1) y = LOCAL_COOR_LENGTH - 1;
		
		lanePoints.at<float>(0,j+31) = (float(y) - 1000)*0.02;
		lanePoints.at<float>(1,j+31) = (1500 - float(x))*0.02;
	}*/
	
	// 7.
	int x,y;
	Point pt[1][4];
	int arr[1];
	arr[0] = 4;
	for(int i=0;i<4;i++)
	{
		x = (int)(-lanePoints.at<float>(0,i) / gridsize + L / 3.0*2.0);
		y = (int)(-lanePoints.at<float>(1,i) / gridsize + L / 2.0);
		
		if (x < 0)     x = 0;
		if (x > L - 1) x = L - 1;
		if (y < 0)     y = 0;
		if (y > L - 1) y = L - 1;
		
		pt[0][i]=Point(y,x);
	}
	const Point* ppt[1] = {pt[0]};
	fillPoly(LaneMaskFilt, ppt, arr, 1, CV_RGB(0,0,0));
	/*int x,y;
	Point pt[1][62];
	int arr[1];
	arr[0] = 62;
	for(int i=0;i<62;i++)
	{
		x = (int)(-lanePoints.at<float>(0,i) / gridsize + L / 3.0*2.0);
		y = (int)(-lanePoints.at<float>(1,i) / gridsize + L / 2.0);
		
		if (x < 0)     x = 0;
		if (x > L - 1) x = L - 1;
		if (y < 0)     y = 0;
		if (y > L - 1) y = L - 1;
		
		pt[0][i]=Point(y,x);
	}
	const Point* ppt[1] = {pt[0]};
	fillPoly(LaneMaskFilt, ppt, arr, 1, CV_RGB(0,0,0));*/
	
	// 8.
	for(int i = 0; i < oLaneMarks.lanemarks.size(); i++)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			sz = oLaneMarks.lanemarks[i].points.size();
			if (oLaneMarks.lanemarks[i].points[0].y > oLaneMarks.lanemarks[i].points[sz-1].y) {
				x1 = oLaneMarks.lanemarks[i].points[0].x;
				y1 = oLaneMarks.lanemarks[i].points[0].y;
				x2 = oLaneMarks.lanemarks[i].points[sz-1].x;
				y2 = oLaneMarks.lanemarks[i].points[sz-1].y;
			} else {
				x1 = oLaneMarks.lanemarks[i].points[sz-1].x;
				y1 = oLaneMarks.lanemarks[i].points[sz-1].y;
				x2 = oLaneMarks.lanemarks[i].points[0].x;
				y2 = oLaneMarks.lanemarks[i].points[0].y;
			}
			y3 = LOCAL_COOR_LENGTH - 1;
			x3 = x2 + (x1 - x2) * (y3 - y2) / (y1 - y2);
			y4 = 0;
			x4 = x2 + (x1 - x2) * (y4 - y2) / (y1 - y2);
			double laneinfo_x1 = (double(y3) - 1000)*0.02;
			double laneinfo_y1 = (1500 - double(x3))*0.02;
			double laneinfo_x2 = (double(y4) - 1000)*0.02;
			double laneinfo_y2 = (1500 - double(x4))*0.02;
			x1 = (int)(-laneinfo_x1/ gridsize + L / 3.0*2.0);
			y1 = (int)(-laneinfo_y1/ gridsize + L / 2.0);
			x2 = (int)(-laneinfo_x2/ gridsize + L / 3.0*2.0);
			y2 = (int)(-laneinfo_y2/ gridsize + L / 2.0);
			cv::line(LaneMaskFilt,cvPoint(y1,x1),cvPoint(y2,x2),CV_RGB(255,255,255),int(3.0/4.0/gridsize));
		}
	}
	/*int x1,y1,x2,y2;
	for(int i = 0; i < oLaneMarks.lanemarks.size(); i++)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			for(int j = 0; j < oLaneMarks.lanemarks[i].points.size()-1; j++)
			{
				double laneinfo_x1 = (double(oLaneMarks.lanemarks[i].points[j].y) - 1000)*0.02;
				double laneinfo_y1 = (1500 - double(oLaneMarks.lanemarks[i].points[j].x))*0.02;
				double laneinfo_x2 = (double(oLaneMarks.lanemarks[i].points[j+1].y) - 1000)*0.02;
				double laneinfo_y2 = (1500 - double(oLaneMarks.lanemarks[i].points[j+1].x))*0.02;
				x1 = (int)(-laneinfo_x1/ gridsize + L / 3.0*2.0);
				y1 = (int)(-laneinfo_y1/ gridsize + L / 2.0);
				x2 = (int)(-laneinfo_x2/ gridsize + L / 3.0*2.0);
				y2 = (int)(-laneinfo_y2/ gridsize + L / 2.0);
				cv::line(LaneMaskFilt,cvPoint(y1,x1),cvPoint(y2,x2),CV_RGB(255,255,255),int(3.0/4.0/gridsize));
			}
		}
	}*/
	
	// 9.
	cv::GaussianBlur(LaneMaskFilt,LaneMaskFilt,cv::Size((int)(1.5/gridsize)*2+1,(int)(1.5/gridsize)*2+1),(int)(1.0/gridsize),(int)(1.0/gridsize),0);
}

#endif

#if 1

void Coster::GenerateLaneCostMap() {

	// 1.
	if(LaneMaskFilt.empty())
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
	LaneMaskFilt = 255;
	Mat lanePoints = Mat(2,62,CV_32FC1);
	
	// 2.
	int left_id = -1;
	if(oLaneMarks.lanemarks[0].type != oLaneMarks.lanemarks[0].TYPE_NONE &&
		oLaneMarks.lanemarks[0].type != oLaneMarks.lanemarks[0].TYPE_STOP)
		left_id = 0;
	else {
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
		return;
	}
	
	// 3.
	int right_id = -1;
	//cout <<"oLaneMarks.lanemarks.size(): " << oLaneMarks.lanemarks.size() << endl;
	for(int i = oLaneMarks.lanemarks.size() - 1; i >= 0; i--)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			right_id = i;
			break;
		}
	}
	
	// 4.
	if(left_id == right_id)
	{
		LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
		return;
	}
	
	// tmp variable
	double x1, x2, x3, x4, y1, y2, y3, y4, sz;
	double angle, r_dis;
	
	// 5.
	angle = oLaneMarks.lanemarks[left_id].theta;
	r_dis = oLaneMarks.lanemarks[left_id].r;
	FindEdgePoint(angle, r_dis, &x3, &y3, &x4, &y4);
	lanePoints.at<float>(0,0) = (float(y3) - 1000)*0.02;
	lanePoints.at<float>(1,0) = (1500 - float(x3))*0.02;
	lanePoints.at<float>(0,1) = (float(y4) - 1000)*0.02;
	lanePoints.at<float>(1,1) = (1500 - float(x4))*0.02;
	
	// 6.
	angle = oLaneMarks.lanemarks[right_id].theta;
	r_dis = oLaneMarks.lanemarks[right_id].r;
	FindEdgePoint(angle, r_dis, &x3, &y3, &x4, &y4);
	lanePoints.at<float>(0,2) = (float(y4) - 1000)*0.02;
	lanePoints.at<float>(1,2) = (1500 - float(x4))*0.02;
	lanePoints.at<float>(0,3) = (float(y3) - 1000)*0.02;
	lanePoints.at<float>(1,3) = (1500 - float(x3))*0.02;
	
	// 7.
	int x,y;
	Point pt[1][4];
	int arr[1];
	arr[0] = 4;
	for(int i=0;i<4;i++)
	{
		x = (int)(-lanePoints.at<float>(0,i) / gridsize + L / 3.0*2.0);
		y = (int)(-lanePoints.at<float>(1,i) / gridsize + L / 2.0);
		
		if (x < 0)     x = 0;
		if (x > L - 1) x = L - 1;
		if (y < 0)     y = 0;
		if (y > L - 1) y = L - 1;
		
		pt[0][i]=Point(y,x);
	}
	const Point* ppt[1] = {pt[0]};
	fillPoly(LaneMaskFilt, ppt, arr, 1, CV_RGB(0,0,0));
	
	// 8.
	for(int i = 0; i < oLaneMarks.lanemarks.size(); i++)
	{
		if(oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_NONE &&
			oLaneMarks.lanemarks[i].type != oLaneMarks.lanemarks[i].TYPE_STOP)
		{
			angle = oLaneMarks.lanemarks[i].theta;
			r_dis = oLaneMarks.lanemarks[i].r;
			FindEdgePoint(angle, r_dis, &x3, &y3, &x4, &y4);
			double laneinfo_x1 = (double(y3) - 1000)*0.02;
			double laneinfo_y1 = (1500 - double(x3))*0.02;
			double laneinfo_x2 = (double(y4) - 1000)*0.02;
			double laneinfo_y2 = (1500 - double(x4))*0.02;
			x1 = (int)(-laneinfo_x1/ gridsize + L / 3.0*2.0);
			y1 = (int)(-laneinfo_y1/ gridsize + L / 2.0);
			x2 = (int)(-laneinfo_x2/ gridsize + L / 3.0*2.0);
			y2 = (int)(-laneinfo_y2/ gridsize + L / 2.0);
			cv::line(LaneMaskFilt,cvPoint(y1,x1),cvPoint(y2,x2),CV_RGB(255,255,255),int(3.0/4.0/gridsize));
		}
	}

	// 9.
	cv::GaussianBlur(LaneMaskFilt,LaneMaskFilt,cv::Size((int)(1.5/gridsize)*2+1,(int)(1.5/gridsize)*2+1),(int)(1.0/gridsize),(int)(1.0/gridsize),0);
}

#endif

void Coster::Prepare()
{
	/*** 初始化网格与数据点 ***/
	LidarMask = Mat::zeros(L,L,CV_8UC1);
	LidarMask = 255;
	LidarMaskFilt = Mat::zeros(L,L,CV_8UC1);

	/*** 转换成图像坐标 ***/
	DataPts3000.clear();
	for(int i=0; i<720; i++)
	{
		Point pt = conv3000ToMap(Point(oScanInfo.ScanInfoX[i],oScanInfo.ScanInfoY[i]), L);
		
		DataPts3000.push_back(pt);
	}
}

void Coster::Process()
{
	Fill();
	//Connect();
	Dilate();
	Gaussian();
}

void Coster::Debug()
{
	//ROS_ERROR("%d", LidarMaskBlur.at<uchar>(180, 350));
}

void Coster::Fill()
{
	Point pt[1][720];
	int arr[1];
	arr[0] = DataPts3000.size();
	for(int i=0; i<DataPts3000.size(); i++)
	{
		pt[0][i] = DataPts3000[i];
	}
	const Point* ppt[1] = {pt[0]};
	/*** fillPoly: Fills the area bounded by one or more polygons. ***/
	fillPoly(LidarMask, ppt, arr, 1, 0);
}

void Coster::Connect()
{
	int thickness = 1;
	
	for(int i=0; i<DataPts3000.size()-1; i++)
	{
		cv::line(LidarMask, DataPts3000[i], DataPts3000[i+1], 255, thickness);
	}
	cv::line(LidarMask, DataPts3000[0], DataPts3000[DataPts3000.size()-1], 255, thickness);
}

void Coster::Dilate()
{
	float r = 1.0;
	float r2 = 0.5;
	Mat kernel = Mat::zeros(((int)(r/gridsize))*2, ((int)(r/gridsize))*2, CV_8U);
	Mat kernel2 = Mat::zeros(((int)(r2/gridsize))*2, ((int)(r2/gridsize))*2, CV_8U);
	cv::circle(kernel, cv::Point((int)(r/gridsize), (int)(r/gridsize)), (int)(r/gridsize), CV_RGB(1,1,1), -1);
	cv::circle(kernel2, cv::Point((int)(r2/gridsize), (int)(r2/gridsize)), (int)(r2/gridsize), CV_RGB(1,1,1), -1);
	cv::dilate(LidarMask, LidarMask1m, kernel, cv::Point(-1,-1), 1);
	cv::dilate(LidarMask1m, LidarMask2m, kernel2, cv::Point(-1,-1), 1);
	//LidarMask.copyTo(LidarMask1m);
	//LidarMask.copyTo(LidarMask2m);
}

void Coster::Gaussian()
{
	//cv::GaussianBlur(LidarMask2m, LidarMaskBlur, cv::Size((int)(2.0/gridsize)*2+1,(int)(2.0/gridsize)*2+1), (int)(0.64/gridsize), (int)(0.64/gridsize), 0);
	cv::GaussianBlur(LidarMask2m, LidarMaskBlur, cv::Size((int)(1.0/gridsize)*2+1,(int)(1.0/gridsize)*2+1), (int)(0.64/gridsize), (int)(0.64/gridsize), 0);
	LidarMaskBlur = LidarMaskBlur/2;
	LidarMask1m.copyTo(LidarMaskBlur,LidarMask1m);
}

Coster::Coster()
{
}

