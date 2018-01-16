#include <fstream>
#include "GetParaLines.h"

//#define PARA_DEBUG
//#define TRAN_DEBUG
#define DIS_THRE   (3.0*50) // 3.0 m
#define GIS_INTERVAL 0.4 // m

extern IplImage * PlanMapShow;
extern double Cur_EV_Speed;
extern double endlinedistance;

int getCorner(CvPoint iline[1000])
{
	double thre = 5; // 0.1 m

	int startID = FindMinDisID(iline, 0, 300, 1500, 1000);
	int CornerID = 0;
	double tmp_max = -1;
	for(int i=startID; i<300; i++)
	{
		if(i > 0)
		{
			double deltaX_1 = iline[i].x - iline[i-1].x;
			double deltaX_2 = iline[i+1].x - iline[i].x;
			double deltaY_1 = iline[i].y - iline[i-1].y;
			double deltaY_2 = iline[i+1].y - iline[i].y;
			if(fabs(deltaX_1)<thre && fabs(deltaY_1)<thre)
			{
				continue;
			}
			if(fabs(deltaX_2)<thre && fabs(deltaY_2)<thre)
			{
				continue;
			}
			double tmp      = fabs(atan2(deltaX_1,deltaY_1) - atan2(deltaX_2,deltaY_2));
			if(tmp > tmp_max)
			{
				tmp_max = tmp;
				CornerID = i;
			}
		}
	}
	return CornerID;
}

/*
void paralane(CvPoint* lanetopara, CvPoint point, CvPoint* outputlane)
{
	double distancemin1 = 10000000000000000.0;
	double distance1;
	int nearcarindextemp;
	int xoffset,yoffset;	
	for(int j=0; j<301;j++)
	{
		distance1 = ((double)lanetopara[j].x - (double)point.x)*((double)lanetopara[j].x - (double)point.x)+((double)lanetopara[j].y - (double)point.y)*((double)lanetopara[j].y - (double)point.y);
		if(distance1<distancemin1)
		{
			distancemin1 = distance1;
			nearcarindextemp = j;
		}
	}
	xoffset = point.x - lanetopara[nearcarindextemp].x;
	yoffset = point.y - lanetopara[nearcarindextemp].y;
	
	for(int i=0;i<301;i++)
	{
		outputlane[i].x = lanetopara[i].x +  xoffset;
		outputlane[i].y = lanetopara[i].y +  yoffset;
		if(outputlane[i].x<=1)
			outputlane[i].x = 1;
		if(outputlane[i].x>2999)
			outputlane[i].x=2999;
		if(outputlane[i].y<=1)
			outputlane[i].y = 1;
		if(outputlane[i].y>2999)
			outputlane[i].y=2999;
	}
}
*/

int removeExcePts(CvPoint iline[1000], CvPoint oline[1000])
{
	double disdis = 0.0; // power 2
	disdis = (iline[0].x-iline[1].x)*(iline[0].x-iline[1].x) + (iline[0].y-iline[1].y)*(iline[0].y-iline[1].y);
	if(disdis > DIS_THRE*DIS_THRE)
	{
		for(int i=0; i<301; i++)
		{
			oline[i] = iline[i];
		}
		return -1;
	}
	else
	{
		int ocnt = 0;
		oline[ocnt++] = iline[0];
		oline[ocnt++] = iline[1];
		for(int i=2; i<301; i++)
		{
			disdis = (oline[ocnt-1].x-iline[i].x)*(oline[ocnt-1].x-iline[i].x) + (oline[ocnt-1].y-iline[i].y)*(oline[ocnt-1].y-iline[i].y);
			if(disdis < DIS_THRE*DIS_THRE)
			{
				oline[ocnt++] = iline[i];
			}
		}
		
		while(ocnt < 301)
		{
			oline[ocnt] = oline[ocnt-1];
			ocnt++;
		}
		return 0;
	}
}

int removeDup(CvPoint iline[1000], CvPoint oline[1000], int MoveThre)
{
	int i = 0;
	oline[i++] = iline[0];
	int j = 0;
	for(; i<301;)
	{
		CvPoint refpt = oline[i-1];
		for(j++; j<301; j++)
		{
			bool f1 = abs(iline[j].x - refpt.x) > MoveThre;
			bool f2 = abs(iline[j].y - refpt.y) > MoveThre;
			if(f1 || f2)
			{
				oline[i++] = iline[j];
				break;
			}
		}
		if(j>=300)
		{
			break;
		}
	}

	for(; i<301; i++)
	{
		oline[i] = oline[i-1];
	}
}

int getGisStEd(CvPoint *GISRectify, bool &GISIndxFlg, int &GISStrtIndx, int &GISEndIndx)
{
	int GIS_SD_THRE_BIG = 2;
	int GIS_SD_THRE_SMALL = 1;
// 2.�ҵ�GIS���ϵ������յ�
	GISIndxFlg = true;
	GISStrtIndx = -1;
	GISEndIndx = -1;
	for(int i = 0; i<300; i++)
	{
		if( (fabs(GISRectify[i].x-GISRectify[i+1].x)>GIS_SD_THRE_BIG) || (fabs(GISRectify[i].y-GISRectify[i+1].y)>GIS_SD_THRE_BIG) )
		{
			GISStrtIndx = i+3; // �ɿ���
			break;
		}
	}
	for(int i = 300; i > 0; i--)
	{
		if( (fabs(GISRectify[i].x-GISRectify[i-1].x)>GIS_SD_THRE_BIG) || (fabs(GISRectify[i].y-GISRectify[i-1].y)>GIS_SD_THRE_BIG) )
		{
			GISEndIndx = i-5; // �ɿ���
			break;
		}
	}
		// 2.1.1�Ϸ��Լ��
	if(GISStrtIndx == -1)
	{
		GISIndxFlg = false;
		for(int i = 0; i<300; i++)
		{
			if( (fabs(GISRectify[i].x-GISRectify[i+1].x)>GIS_SD_THRE_SMALL) || (fabs(GISRectify[i].y-GISRectify[i+1].y)>GIS_SD_THRE_SMALL) )
			{
				GISStrtIndx = i+3;
				break;
			}
		}
		if(GISStrtIndx == -1)
		{
#ifdef PARA_DEBUG
			ROS_ERROR("Can't find start point on the GIS rectify!");
#endif
			GISStrtIndx = 3;
		}
	}
		// 2.1.2�Ϸ��Լ��
	if(GISEndIndx == -1)
	{
		GISIndxFlg = false;
		for(int i = 300; i > 0; i--)
		{
			if( (fabs(GISRectify[i].x-GISRectify[i-1].x)>GIS_SD_THRE_SMALL) || (fabs(GISRectify[i].y-GISRectify[i-1].y)>GIS_SD_THRE_SMALL) )
			{
				GISEndIndx = i-5;
				break;
			}
		}
		if(GISEndIndx == -1)
		{
#ifdef PARA_DEBUG
			ROS_ERROR("Can't find end point on the GIS rectify!");
#endif
			GISEndIndx = GISStrtIndx+75;
			if(GISEndIndx>300)
			{
				GISEndIndx = 300;
			}
		}
	}
		// 2.1.3�Ϸ��Լ��
	if( (GISEndIndx - GISStrtIndx <= 15) || (GISStrtIndx > 150) || (GISEndIndx < 80) )
	{
		GISIndxFlg = false;
#ifdef PARA_DEBUG
		ROS_ERROR("GIS rectify start point and end point exception! GISStrtIndx = %d, GISEndIndx = %d", GISStrtIndx, GISEndIndx);
#endif
	}
}

int FindMinDisID(CvPoint2D64f* pts, int minID, int maxID, double centerX, double centerY)
{
	int mindisID = 0;
	double mindis = 100000000;
	for(int i = minID; i < maxID; i++)
	{
		double dis = sqrt
				(
						(pts[i].x - centerX)*(pts[i].x - centerX)+
						(pts[i].y - centerY)*(pts[i].y - centerY)
				);
		if(dis < mindis)
		{
			mindis = dis;
			mindisID = i;
		}
	}
	return mindisID;
}

int FindMinDisID(CvPoint* pts, int minID, int maxID, int centerX, int centerY)
{
	int mindisID = 0;
	double mindis = 100000000;
	for(int i = minID; i < maxID; i++)
	{
		double dis = sqrt
				(
						(double)
						(
								(pts[i].x - centerX)*(pts[i].x - centerX)+
								(pts[i].y - centerY)*(pts[i].y - centerY)
						)
				);
		if(dis < mindis)
		{
			mindis = dis;
			mindisID = i;
		}
	}
	return mindisID;
}

double calcTurnRadius(CvPoint2D64f P1, CvPoint2D64f P2, CvPoint2D64f P3)
// ����İ뾶�У���ֵ�����ת����ֵ�����ת
{
	// K-ON!
	double P2P1Power2 = (P2.y-P1.y)*(P2.y-P1.y) + (P2.x-P1.x)*(P2.x-P1.x);
	double P2P3Power2 = (P2.y-P3.y)*(P2.y-P3.y) + (P2.x-P3.x)*(P2.x-P3.x);
	double P1P3Power2 = (P1.y-P3.y)*(P1.y-P3.y) + (P1.x-P3.x)*(P1.x-P3.x);
	if(P2P1Power2 + P2P3Power2 > P1P3Power2)
	{
#ifdef PARA_DEBUG
		ROS_ERROR("Turn radius is too small! P2P1Power2 = %lf, P2P3Power2 = %lf, P1P3Power2 = %lf", P2P1Power2, P2P3Power2, P1P3Power2);
#endif
	}

	double R = 0.0;
	double T1_x,T1_y,T2_x,T2_y;
	T1_x = -(P2.y-P1.y)/sqrt((P2.x-P1.x)*(P2.x-P1.x)+(P2.y-P1.y)*(P2.y-P1.y));
	T1_y =  (P2.x-P1.x)/sqrt((P2.x-P1.x)*(P2.x-P1.x)+(P2.y-P1.y)*(P2.y-P1.y));
	T2_x = -(P3.y-P2.y)/sqrt((P3.x-P2.x)*(P3.x-P2.x)+(P3.y-P2.y)*(P3.y-P2.y));
	T2_y =  (P3.x-P2.x)/sqrt((P3.x-P2.x)*(P3.x-P2.x)+(P3.y-P2.y)*(P3.y-P2.y));
	//if(T1_x*T2_y != T2_x*T1_y)
	if(fabs(T1_x*T2_y - T2_x*T1_y)>0.01)
		//R = (T1_y*(P2.x-P1.x) - T1_x*(P2.y-P1.y))/(T2_y*T1_x-T2_x*T1_y+0.00001);
		R = (T1_y*(P2.x-P1.x) - T1_x*(P2.y-P1.y))/(T2_y*T1_x-T2_x*T1_y);
	else
		R = 100000.0;
	return R;
}

cv::Point2d calcTwoMinTurnRadii(CvPoint* RoadPts)
// ����������Сת��뾶������ֵ��x����Ϊ��ת��y����Ϊ��ת
{
	CvPoint2D64f P1,P2,P3;
	double turn_radius = 0.0;
	double left_min_turn_radius = 10000000.0;
	double right_min_turn_radius = -10000000.0;
	int min_gis_pts_id = FindMinDisID(RoadPts,0,300,1500,1000);
	int fws_gis_pts_id = ((min_gis_pts_id+75)>300)?300:(min_gis_pts_id+75);
	int st(0);
	int ed(0);
	bool GISIndxFlg = false;
	int GISStrtIndx = -1;
	int GISEndIndx = -1;
	getGisStEd(RoadPts, GISIndxFlg, GISStrtIndx, GISEndIndx);
	if(GISIndxFlg)
	{
		st = GISStrtIndx + 5;
		ed = GISEndIndx - 5;
	}
	else
	{
		st = min_gis_pts_id + 5;
		ed = fws_gis_pts_id - 5;
	}
	
	/*double pros_dis = (Cur_EV_Speed * 1000.0 / 3600.0) * 3.0;
	if (pros_dis < 10.0) pros_dis = 10.0;
	int pros_id = st + pros_dis / GIS_INTERVAL;
	ed = pros_id;*/
	
	for(int i=st; i<ed; i++)
	{
		P1.x = RoadPts[i-5].x;
		P1.y = RoadPts[i-5].y;
		P2.x = RoadPts[i].x;
		P2.y = RoadPts[i].y;
		P3.x = RoadPts[i+5].x;
		P3.y = RoadPts[i+5].y;
		if((fabs(P1.x -P2.x)<1 && fabs(P1.y-P2.y)<1 && fabs(P2.x- P3.x) <1 && fabs(P2.y- P3.y)<1) || (fabs(P1.x)<1 && fabs(P1.y)<1) || (fabs(P2.x)<1 && fabs(P2.y)<1) || (fabs(P3.x)<1 && fabs(P3.y)<1))
		// ����㿿��̫����������κ�һ�����복��̫��
			turn_radius = 0;
		else
		{
			turn_radius = calcTurnRadius(P1,P2,P3);
			// ���κ�ʱ��ʹ��ת��뾶ʱ����Ҫע�⵽����Ž����������ת��������ⲻ����Ϊ�ظ��丽���κκ���
			// !isnan(turn_radius)����Ϊ����һ����nan��not a number
			if(isnan(turn_radius))
			{
#ifdef PARA_DEBUG
				ROS_ERROR("turn_radius is not a number!");
#endif
				continue;
			}
			
			if(turn_radius > 0)
			// ��ת
			{
				if(turn_radius < left_min_turn_radius)
				{
					left_min_turn_radius = turn_radius;
				}
			}
			else
			// ��ת
			{
				if(fabs(turn_radius) < fabs(right_min_turn_radius))
				{
					right_min_turn_radius = turn_radius;
				}
			}
		} // else
	} // for
	
	return cv::Point2d(left_min_turn_radius, right_min_turn_radius);
}

int getParaLines(CvPoint cline[1000], int leftlen, int rightlen, CvPoint plines[500][1000], int &num, int &id, int &speedlimit, int &stout, int &edout)
{
	// param
	int CurveDist = 20; // 0.4m
	//int CrossThre = 100; // 2m
	//int CrossThre = 50; // 1m
	int CrossThre = 5;
	int MoveThre = 8; // 0.16m
	int HalfMinNum = 1;
	float Wradius = 40; // Ҫ��
	float Wwide = 10; // Ҫ��
	int MinSpeed = 0;
	int MaxSpeed = 40000; // 40km/h

	int leftneednum = (leftlen / CurveDist) + 0.5;
	int rightneednum = (rightlen / CurveDist) + 0.5;
	
	CvPoint clineV2[1000];
	removeDup(cline, clineV2, MoveThre);
	Point2d gisminradii = calcTwoMinTurnRadii(clineV2);
	//printf("a\n");
#ifdef PARA_DEBUG
	ROS_ERROR("left_r = %lf, right_r = %lf", gisminradii.x, gisminradii.y);
#endif

	int i = 0;
	for(; i<leftneednum; i++)
	{
		double rtmp = gisminradii.x - (i+1) * CurveDist;
		if(rtmp < CrossThre)
		{
			break;
		}
	}
	int leftactunum = i;
	int lefttrannum = leftneednum - leftactunum;
	lefttrannum = 0; // no translation forcely

	i = 0;
	for(; i<rightneednum; i++)
	{
		double rtmp = gisminradii.y + (i+1) * CurveDist;
		if(rtmp > -CrossThre)
		{
			break;
		}
	}
	int rightactunum = i;
	int righttrannum = rightneednum - rightactunum;
	righttrannum = 0; // no translation forcely
	
	if(leftactunum==0 && rightactunum==0)
	{
		leftactunum = HalfMinNum;
		rightactunum = HalfMinNum;
#ifdef PARA_DEBUG
		ROS_ERROR("leftactunum==0 && rightactunum==0");
#endif
	}

	num = leftactunum + rightactunum + 1;
	id = 500/2;
	
	stout = id - leftactunum;
	edout = id + rightactunum;
	
	double minr = abs(gisminradii.x);
	if(abs(gisminradii.y) < minr)
	{
		minr = abs(gisminradii.y);
	}
	int totalwide = leftlen + rightlen;
	speedlimit = Wradius * minr + Wwide * totalwide;
	if(speedlimit > MaxSpeed)
	{
		speedlimit = MaxSpeed;
	}
	else if(speedlimit < MinSpeed)
	{
		speedlimit = MinSpeed;
	}

	for(int i=0; i<301; i++)
	{
		plines[id][i] = clineV2[i];
	}

	for(int i=0; i<leftactunum; i++)
	{
		getOnePara(clineV2, (leftactunum-i)*CurveDist, plines[stout+i], MoveThre);
	}

	for(int i=0; i<rightactunum; i++)
	{
		getOnePara(clineV2, -(i+1)*CurveDist, plines[id+i+1], MoveThre);
	}
	
#ifdef PARA_DEBUG
	ROS_ERROR("num = %d", num);
	ROS_ERROR("id = %d", id);
	ROS_ERROR("speedlimit = %d", speedlimit);
#endif
#ifdef TRAN_DEBUG
	ROS_WARN("--------------------------------getParaLines!-----------------------------------");
	ROS_WARN("[need]left = %d, right = %d", leftneednum, rightneednum);
	ROS_WARN("[actu]left = %d, right = %d", leftactunum, rightactunum);
	ROS_ERROR("[tran]left = %d, right = %d", lefttrannum, righttrannum);
#endif

	return 0;
}

int getOnePara(CvPoint *clineV2, double dis, CvPoint *out, int MoveThre) {

	int st = -1;
	int ed = -1;
	Point2d direction;
	for(int i=0; i<300; i++)
	{
		bool f1 = abs(clineV2[i].x - clineV2[i+1].x) > MoveThre;
		bool f2 = abs(clineV2[i].y - clineV2[i+1].y) > MoveThre;
		if(f1 || f2)
		{
			st = i;
			break;
		}
	}
	for(int i=300; i>0; i--)
	{
		bool f1 = abs(clineV2[i].x - clineV2[i-1].x) > MoveThre;
		bool f2 = abs(clineV2[i].y - clineV2[i-1].y) > MoveThre;
		if(f1 || f2)
		{
			ed = i;
			break;
		}
	}
	//printf("c\n");
#ifdef PARA_DEBUG
	ROS_ERROR("st = %d, ed = %d", st, ed);
#endif
	if(st==-1 || st==ed || ed<3)
	{ // error!
#ifdef PARA_DEBUG
		ROS_ERROR("st and ed error!");
#endif
		return -1;
	}
	
	/*direction.x = clineV2[st+2].x - clineV2[st].x;
	direction.y = clineV2[st+2].y - clineV2[st].y;
	for(int j=0; j<=st; j++)
	{
		out[j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
		out[j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
	}

	for(int j=st+1; j<ed-1; j++)
	{
		direction.x = clineV2[j+2].x - clineV2[j].x;
		direction.y = clineV2[j+2].y - clineV2[j].y;
		out[j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
		out[j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
	}

	direction.x = clineV2[ed].x - clineV2[ed-2].x;
	direction.y = clineV2[ed].y - clineV2[ed-2].y;
	for(int j=ed-1; j<301; j++)
	{
		out[j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
		out[j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
	}*/
	
	direction.x = 0;
	direction.y = dis;
	for(int j=0; j<301-2; j++)
	{
		int pdx = clineV2[j+2].x - clineV2[j].x;
		int pdy = clineV2[j+2].y - clineV2[j].y;
		if (sqrt(pdx*pdx+pdy*pdy) > 20) {
			direction.x = pdx;
			direction.y = pdy;
		}
		
		out[j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
		out[j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*dis;
	}
	
	for (int j = 301-2; j < 301; j++) {
		out[j] = out[301-2-1];
	}
	
	
}

int getParaLinesV2(CvPoint cline[1000], int leftlen, int rightlen, CvPoint plines[100][1000], int &num, int &id, int &speedlimit)
{
	// param
	int CurveDist = 20; // 0.4m
	//int CrossThre = 100; // 2m
	int CrossThre = 50; // 1m
	int MoveThre = 8; // 0.16m
	int HalfMinNum = 1;
	float Wradius = 40; // Ҫ��
	float Wwide = 10; // Ҫ��
	int MinSpeed = 0;
	int MaxSpeed = 40000; // 40km/h

	int leftneednum = (leftlen / CurveDist) + 0.5;
	int rightneednum = (rightlen / CurveDist) + 0.5;

	CvPoint clineV2[1000];
	removeDup(cline, clineV2, MoveThre);
	Point2d gisminradii = calcTwoMinTurnRadii(clineV2);
	//printf("a\n");
#ifdef PARA_DEBUG
	ROS_ERROR("left_r = %lf, right_r = %lf", gisminradii.x, gisminradii.y);
#endif

	int i = 0;
	for(; i<leftneednum; i++)
	{
		double rtmp = gisminradii.x - (i+1) * CurveDist;
		if(rtmp < CrossThre)
		{
			break;
		}
	}
	int leftactunum = i;
	int lefttrannum = leftneednum - leftactunum;
	lefttrannum = 0; // no translation forcely

	i = 0;
	for(; i<rightneednum; i++)
	{
		double rtmp = gisminradii.y + (i+1) * CurveDist;
		if(rtmp > -CrossThre)
		{
			break;
		}
	}
	int rightactunum = i;
	int righttrannum = rightneednum - rightactunum;
	righttrannum = 0; // no translation forcely

	if(leftactunum==0 && rightactunum==0)
	{
		leftactunum = HalfMinNum;
		rightactunum = HalfMinNum;
#ifdef PARA_DEBUG
		ROS_ERROR("leftactunum==0 && rightactunum==0");
#endif
	}

	num = leftactunum + rightactunum + 1;
	id = leftactunum;
	double minr = abs(gisminradii.x);
	if(abs(gisminradii.y) < minr)
	{
		minr = abs(gisminradii.y);
	}
	int totalwide = leftlen + rightlen;
	speedlimit = Wradius * minr + Wwide * totalwide;
	if(speedlimit > MaxSpeed)
	{
		speedlimit = MaxSpeed;
	}
	else if(speedlimit < MinSpeed)
	{
		speedlimit = MinSpeed;
	}

	for(int i=0; i<301; i++)
	{
		plines[id][i] = clineV2[i];
	}

	int st = -1;
	int ed = -1;
	Point2d direction;
	for(int i=0; i<300; i++)
	{
		bool f1 = abs(clineV2[i].x - clineV2[i+1].x) > MoveThre;
		bool f2 = abs(clineV2[i].y - clineV2[i+1].y) > MoveThre;
		if(f1 || f2)
		{
			st = i;
			break;
		}
	}
	for(int i=300; i>0; i--)
	{
		bool f1 = abs(clineV2[i].x - clineV2[i-1].x) > MoveThre;
		bool f2 = abs(clineV2[i].y - clineV2[i-1].y) > MoveThre;
		if(f1 || f2)
		{
			ed = i;
			break;
		}
	}
	//printf("c\n");
#ifdef PARA_DEBUG
	ROS_ERROR("st = %d, ed = %d", st, ed);
#endif
	if(st==-1 || st==ed || ed<3)
	{ // error!
#ifdef PARA_DEBUG
		ROS_ERROR("st and ed error!");
#endif
		return -1;
	}

	//for(int i=0; i<leftactunum; i++)
	for(int i=leftactunum-1; i>=0; i++) {
		direction.x = plines[i+1][st+2].x - plines[i+1][st].x;
		direction.y = plines[i+1][st+2].y - plines[i+1][st].y;
		for(int j=0; j<=st; j++)

		{
			plines[i][j].x = plines[i+1][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*CurveDist;
			plines[i][j].y = plines[i+1][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*CurveDist;
		}

		for(int j=st+1; j<ed-1; j++)
		{
			direction.x = plines[i+1][j+2].x - plines[i+1][j].x;
			direction.y = plines[i+1][j+2].y - plines[i+1][j].y;
			plines[i][j].x = plines[i+1][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*CurveDist;
			plines[i][j].y = plines[i+1][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*CurveDist;
		}

		direction.x = plines[i+1][ed].x - plines[i+1][ed-2].x;
		direction.y = plines[i+1][ed].y - plines[i+1][ed-2].y;
		for(int j=ed-1; j<301; j++)
		{
			plines[i][j].x = plines[i+1][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*CurveDist;
			plines[i][j].y = plines[i+1][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*CurveDist;
		}
		
		// smooth
		vector<CvPoint2D64f> tmp_pts;
		CvPoint2D64f P1,P2,P3;
		double turn_radius = 0x7fffffff;
		for (int k = 0 + 5; k < 301 - 5; k++) {
			P1.x = plines[i][k-5].x;
			P1.y = plines[i][k-5].y;
			P2.x = plines[i][k].x;
			P2.y = plines[i][k].y;
			P3.x = plines[i][k+5].x;
			P3.y = plines[i][k+5].y;
			if((fabs(P1.x -P2.x)<1 && fabs(P1.y-P2.y)<1 && fabs(P2.x- P3.x) <1 && fabs(P2.y- P3.y)<1) || (fabs(P1.x)<1 && fabs(P1.y)<1) || (fabs(P2.x)<1 && fabs(P2.y)<1) || (fabs(P3.x)<1 && fabs(P3.y)<1))
				turn_radius = 0;
			else
			{
				turn_radius = calcTurnRadius(P1,P2,P3);
				if(isnan(turn_radius))
				{
	#ifdef PARA_DEBUG
					ROS_ERROR("turn_radius is not a number!");
	#endif
					continue;
				}
			
			} // else
			
			if (fabs(turn_radius) < CrossThre) {
				
			} else {
				tmp_pts.push_back(P1);
			}
		} // smooth
	}

	for(int i=0; i<rightactunum; i++) {
		direction.x = plines[id+i][st+2].x - plines[id+i][st].x;
		direction.y = plines[id+i][st+2].y - plines[id+i][st].y;
		for(int j=0; j<=st; j++)
		{
			plines[i+id+1][j].x = plines[id+i][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(-1)*CurveDist;
			plines[i+id+1][j].y = plines[id+i][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(-1)*CurveDist;
		}

		for(int j=st+1; j<ed-1; j++)
		{
			direction.x = plines[id+i][j+2].x - plines[id+i][j].x;
			direction.y = plines[id+i][j+2].y - plines[id+i][j].y;
			plines[i+id+1][j].x = plines[id+i][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(-1)*CurveDist;
			plines[i+id+1][j].y = plines[id+i][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(-1)*CurveDist;
		}

		direction.x = plines[id+i][ed].x - plines[id+i][ed-2].x;
		direction.y = plines[id+i][ed].y - plines[id+i][ed-2].y;
		for(int j=ed-1; j<301; j++)
		{
			plines[i+id+1][j].x = plines[id+i][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(-1)*CurveDist;
			plines[i+id+1][j].y = plines[id+i][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(-1)*CurveDist;
		}
	}
	
	//----------------------------translation---------------------------------
	if(lefttrannum != 0)
	{
		for(int i=num-1; i>=0; i--)
		{
			for(int j=0; j<301; j++)
			{
				plines[i+lefttrannum][j] = plines[i][j];
			}
		}
	}
	if(lefttrannum != 0)
	{
		//int id = getCorner(plines[lefttrannum]);
		int id = getCorner(clineV2);
#ifdef TRAN_DEBUG
		ROS_WARN("left_corner_id = %d", id);
#endif
		if(id < 0)
			id = 0;
		if(id > 288)
			id = 288;
		direction.x = clineV2[id+2].x - clineV2[id].x;
		direction.y = clineV2[id+2].y - clineV2[id].y;
#ifdef TRAN_DEBUG
		ROS_WARN("[left]dx = %lf, dy = %lf", direction.x, direction.y);
#endif
		for(int i=0; i<lefttrannum; i++)
		{
			for(int j=0; j<301; j++)
			{
				plines[i][j].x = plines[lefttrannum][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(lefttrannum-i)*CurveDist;
				plines[i][j].y = plines[lefttrannum][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(lefttrannum-i)*CurveDist;
			}
		}
	}
	if(righttrannum != 0)
	{
		//int id = getCorner(plines[lefttrannum+num-1]);
		int id = getCorner(clineV2);
#ifdef TRAN_DEBUG
		ROS_WARN("right_corner_id = %d", id);
#endif
		if(id < 0)
			id = 0;
		if(id > 288)
			id = 288;
		direction.x = clineV2[id+2].x - clineV2[id].x;
		direction.y = clineV2[id+2].y - clineV2[id].y;
#ifdef TRAN_DEBUG
		ROS_WARN("[right]dx = %lf, dy = %lf", direction.x, direction.y);
#endif
		for(int i=lefttrannum+num; i<lefttrannum+num+righttrannum; i++)
		{
			for(int j=0; j<301; j++)
			{
				plines[i][j].x = plines[lefttrannum+num-1][j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(lefttrannum+num-1-i)*CurveDist;
				plines[i][j].y = plines[lefttrannum+num-1][j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(lefttrannum+num-1-i)*CurveDist;
			}
		}
	}
	num = lefttrannum+num+righttrannum;
	id  = id + lefttrannum;

	for(int i=0; i<num; i++)
	{
		for(int j=0; j<301; j++)
		{
			CvPoint pt = plines[i][j];
			/*if(pt.x<1)
				plines[i][j].x = 1;
			if(pt.x>2999)
				plines[i][j].x = 2999;
			if(pt.y<1)
				plines[i][j].y = 1;
			if(pt.y>2999)
				plines[i][j].y = 2999;*/
		}
	}

/*

	CvPoint plinesV2[100][1000];

	for(int i=0; i<num; i++)

	{

		removeExcePts(plines[i], plinesV2[i]);

	}

	for(int i=0; i<num; i++)

	{

		for(int j=0; j<301; j++)

		{

			plines[i][j] = plinesV2[i][j];

		}

	}

*/

#ifdef PARA_DEBUG
	ROS_ERROR("num = %d", num);
	ROS_ERROR("id = %d", id);
	ROS_ERROR("speedlimit = %d", speedlimit);
#endif
#ifdef TRAN_DEBUG
	ROS_WARN("--------------------------------getParaLines!-----------------------------------");
	ROS_WARN("[need]left = %d, right = %d", leftneednum, rightneednum);
	ROS_WARN("[actu]left = %d, right = %d", leftactunum, rightactunum);
	ROS_ERROR("[tran]left = %d, right = %d", lefttrannum, righttrannum);
#endif

	return 0;
}

int getParaLine(CvPoint cline[1000], CvPoint pt, CvPoint plines[3][1000])
{
#ifdef PARA_DEBUG
	time_t t = time(0);
	char StartRunTimeString[100];
	strftime(StartRunTimeString, sizeof(StartRunTimeString), "%Y-%m-%d %X %A", localtime(&t));
	static string FileTime = StartRunTimeString;
	static long int SaveDataCnter(0);
	SaveDataCnter++;
	string FileName;
	string FileFullName;
#endif
#ifdef PARA_DEBUG
	FileName = "clineP2PDistances.txt";
	FileFullName = FileTime + " " + FileName;
	ofstream GISP2PDistancesObj(FileFullName.c_str(), ofstream::app);
	if(GISP2PDistancesObj.is_open())
	{
		GISP2PDistancesObj << "----------------------------------------------- " << SaveDataCnter << " -----------------------------------------------" << endl;
		for(int i = 0; i < 300; i++)
		{
			double distmp = sqrt(  (cline[i+1].x-cline[i].x)*(cline[i+1].x-cline[i].x)
			                      +(cline[i+1].y-cline[i].y)*(cline[i+1].y-cline[i].y)
			                    );

			GISP2PDistancesObj << "[" << i << "] " << "dist = " << (distmp/50) << " m" << endl;
		}
		GISP2PDistancesObj.close();
	}
	else
	{
		ROS_ERROR("Couldn't open: GISP2PDistances.txt");
	}
#endif

	int outid = -1;
	int minID = FindMinDisID(cline, 0, 300, pt.x, pt.y);
	int CurveDist = sqrt((cline[minID].x-pt.x)*(cline[minID].x-pt.x)+(cline[minID].y-pt.y)*(cline[minID].y-pt.y));
	int LeftCurveDist = CurveDist;
	int RightCurveDist = CurveDist;
	int leftorright = 0;// 0-left
//--------------------------------------------------------------------------------------------------
	//int CrossThre = 100; // 2m
	int CrossThre = 50; // 1m
	int MoveThre = 8; // 0.16m

#ifdef PARA_DEBUG
	ROS_ERROR("CurveDist = %d", CurveDist);
#endif

	CvPoint clineV2[1000];
	removeDup(cline, clineV2, MoveThre);
	Point2d gisminradii = calcTwoMinTurnRadii(clineV2);
#ifdef PARA_DEBUG
	ROS_ERROR("left_r = %lf, right_r = %lf", gisminradii.x, gisminradii.y);
#endif

	double rtmp = gisminradii.x - LeftCurveDist;
	if(rtmp < CrossThre)
	{
		LeftCurveDist = gisminradii.x - CrossThre;
	}
	if(LeftCurveDist < 0)
	{
		LeftCurveDist = 0;
	}

	rtmp = gisminradii.y + RightCurveDist;
	if(rtmp > -CrossThre)
	{
		RightCurveDist = -gisminradii.y - CrossThre;
	}
	if(RightCurveDist < 0)
	{
		RightCurveDist = 0;
	}

#ifdef PARA_DEBUG
	ROS_ERROR("LeftCurveDist = %d, RightCurveDist = %d", LeftCurveDist, RightCurveDist);
#endif
	int id = 1;
	for(int i=0; i<301; i++)
	{
		plines[id][i] = clineV2[i];
	}

	int st = -1;
	int ed = -1;
	Point2d direction;
	for(int i=0; i<300; i++)
	{
		bool f1 = abs(clineV2[i].x - clineV2[i+1].x) > MoveThre;
		bool f2 = abs(clineV2[i].y - clineV2[i+1].y) > MoveThre;
		if(f1 || f2)
		{
			st = i;
			break;
		}
	}
	for(int i=300; i>0; i--)
	{
		bool f1 = abs(clineV2[i].x - clineV2[i-1].x) > MoveThre;
		bool f2 = abs(clineV2[i].y - clineV2[i-1].y) > MoveThre;
		if(f1 || f2)
		{
			ed = i;
			break;
		}
	}
	//printf("ccc\n");
#ifdef PARA_DEBUG
	ROS_ERROR("st = %d, ed = %d", st, ed);
#endif
	if(st==-1 || st==ed || ed<3)
	{ // error!
#ifdef PARA_DEBUG
		ROS_ERROR("st and ed error!");
#endif
		return outid;
	}

	for(int i=0; i<1; i++)
	{
		direction.x = clineV2[st+2].x - clineV2[st].x;
		direction.y = clineV2[st+2].y - clineV2[st].y;
		for(int j=0; j<=st; j++)
		{
			plines[i][j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(id-i)*LeftCurveDist;
			plines[i][j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(id-i)*LeftCurveDist;
		}

		for(int j=st+1; j<ed-1; j++)
		{
			direction.x = clineV2[j+2].x - clineV2[j].x;
			direction.y = clineV2[j+2].y - clineV2[j].y;
			plines[i][j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(id-i)*LeftCurveDist;
			plines[i][j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(id-i)*LeftCurveDist;
		}

		direction.x = clineV2[ed].x - clineV2[ed-2].x;
		direction.y = clineV2[ed].y - clineV2[ed-2].y;
		for(int j=ed-1; j<301; j++)
		{
			plines[i][j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*(id-i)*LeftCurveDist;
			plines[i][j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*(id-i)*LeftCurveDist;
		}
	}

	for(int i=0; i<1; i++)
	{
		direction.x = clineV2[st+2].x - clineV2[st].x;
		direction.y = clineV2[st+2].y - clineV2[st].y;
		for(int j=0; j<=st; j++)
		{
			plines[i+id+1][j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*-(i+1)*RightCurveDist;
			plines[i+id+1][j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*-(i+1)*RightCurveDist;
		}

		for(int j=st+1; j<ed-1; j++)
		{
			direction.x = clineV2[j+2].x - clineV2[j].x;
			direction.y = clineV2[j+2].y - clineV2[j].y;
			plines[i+id+1][j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*-(i+1)*RightCurveDist;
			plines[i+id+1][j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*-(i+1)*RightCurveDist;
		}

		direction.x = clineV2[ed].x - clineV2[ed-2].x;
		direction.y = clineV2[ed].y - clineV2[ed-2].y;
		for(int j=ed-1; j<301; j++)
		{
			plines[i+id+1][j].x = clineV2[j].x - direction.y/sqrt(direction.x*direction.x+direction.y*direction.y)*-(i+1)*RightCurveDist;
			plines[i+id+1][j].y = clineV2[j].y + direction.x/sqrt(direction.x*direction.x+direction.y*direction.y)*-(i+1)*RightCurveDist;
		}
	}

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<301; j++)
		{
			CvPoint pt = plines[i][j];
			/*if(pt.x<1)
				plines[i][j].x = 1;
			if(pt.x>2999)
				plines[i][j].x = 2999;
			if(pt.y<1)
				plines[i][j].y = 1;
			if(pt.y>2999)
				plines[i][j].y = 2999;*/
		}
	}

	int leid = FindMinDisID(plines[0], 0, 300, pt.x, pt.y);
	int riid = FindMinDisID(plines[2], 0, 300, pt.x, pt.y);
	int ledis = sqrt((plines[0][leid].x-pt.x)*(plines[0][leid].x-pt.x)+(plines[0][leid].y-pt.y)*(plines[0][leid].y-pt.y));
	int ridis = sqrt((plines[2][riid].x-pt.x)*(plines[2][riid].x-pt.x)+(plines[2][riid].y-pt.y)*(plines[2][riid].y-pt.y));
#ifdef PARA_DEBUG
	ROS_ERROR("left_dis = %d, right_dis = %d", ledis, ridis);
#endif
	if(ledis > ridis)
	{
		outid = 2;
	}
	else
	{
		outid = 0;
	}
	/*
	CvPoint plinesV2[3][1000];
	for(int i=0; i<3; i++)
	{
		removeExcePts(plines[i], plinesV2[i]);
	}
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<301; j++)
		{
			plines[i][j] = plinesV2[i][j];
		}
	}
*/

	//printf("ddd\n");
	return outid;
}
