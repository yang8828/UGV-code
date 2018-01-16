/*
 * C_Gis.cpp
 *
 *  Created on: 2013-10-12
 *      Author: greensky
 */


#include <in2_localmap/C_Gis.h>
#include <math.h>


bool C_GisFrame::getAbsolutePoint2(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &angle)
{
	int min_delta;
	for(int i=0;i<301;i++)
	{
		if(i == 0)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(initialAlignedData().x[i],initialAlignedData().y[i]);
		}
		if(ReferenceLine->distanceToModel(initialAlignedData().x[i],initialAlignedData().y[i])<min_delta)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(initialAlignedData().x[i],initialAlignedData().y[i]);
		}
	}
	ReferencePoint.x = initialAlignedData().x[ReferenceIndex];
	ReferencePoint.y = initialAlignedData().y[ReferenceIndex];
	double x1,y1; //start point
	double x2,y2; //end point
	for(int i=0;i<5;i++)
	{
		if(ReferenceIndex + i < 301)
		{
			x2 = (double)initialAlignedData().x[i];
			y2 = (double)initialAlignedData().y[i];
		}
		if(ReferenceIndex - i >= 0)
		{
			x1 = (double)initialAlignedData().x[i];
			y1 = (double)initialAlignedData().y[i];
			}
		}
		angle =  atan2(x2-x1,y2-y1)*180.0/PI;
		return true;
	}

bool C_GisFrame::getAbsolutePoint(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins)
{
	int min_delta;
	for(int i=0;i<301;i++)
	{
		if(i == 0)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(initialAlignedData().x[i],initialAlignedData().y[i]);
		}
		if(ReferenceLine->distanceToModel(initialAlignedData().x[i],initialAlignedData().y[i])<min_delta)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(initialAlignedData().x[i],initialAlignedData().y[i]);
		}
	}
	ReferencePoint.x = initialAlignedData().x[ReferenceIndex];
	ReferencePoint.y = initialAlignedData().y[ReferenceIndex];
	double x1,y1; //start point
	double x2,y2; //end point
	for(int i=0;i<5;i++)
	{
		if(ReferenceIndex + i < 301)
		{
			x2 = (double)initialAlignedData().x[i];
			y2 = (double)initialAlignedData().y[i];
		}
		if(ReferenceIndex - i >= 0)
		{
			x1 = (double)initialAlignedData().x[i];
			y1 = (double)initialAlignedData().y[i];
		}
	}

	direct_angle =  atan2(x2-x1,y2-y1)*180.0/PI;
	direct_angle += dst_Ins.attitude[2];
	direct_angle = fmod(direct_angle+360.0,360.0);
	return true;
}

bool C_GisFrame::getRawPoint(Point CrossPoint,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins)
{
	double min_delta;
	for(int i=0;i<301;i++)
	{
		if(i == 0)
		{
			ReferenceIndex = i;
			double delta_x = (double)(CrossPoint.x-x[i]);
			double delta_y = (double)(CrossPoint.y-y[i]);
			min_delta = delta_x*delta_x + delta_y*delta_y;
		}
		else
		{
			double delta_x = (double)(CrossPoint.x-x[i]);
			double delta_y = (double)(CrossPoint.y-y[i]);
			double temp = delta_x*delta_x + delta_y*delta_y;
			if(temp<min_delta)
			{
				ReferenceIndex = i;
				min_delta = temp;
			}
		}
	}
	ReferencePoint.x = x[ReferenceIndex];
	ReferencePoint.y = y[ReferenceIndex];
	double x1,y1; //start point
	double x2,y2; //end point
	for(int i=0;i<5;i++)
	{
		if(ReferenceIndex + i < 301)
		{
			x2 = (double)x[ReferenceIndex + i ];
			y2 = (double)y[ReferenceIndex + i ];
		}
		if(ReferenceIndex - i >= 0)
		{
			x1 = (double)x[ReferenceIndex - i ];
			y1 = (double)y[ReferenceIndex - i ];
		}
	}
	direct_angle = atan2(x2-x1,y2-y1)*180.0/PI;
	direct_angle += dst_Ins.attitude[2];
	direct_angle = fmod(direct_angle+360.0,360.0);
	return true;
}

bool C_GisFrame::getRawPoint(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins)
{
	int min_delta;
	for(int i=0;i<301;i++)
	{
		if(i == 0)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(x[i],y[i]);
		}
		if(ReferenceLine->distanceToModel(x[i],y[i])<min_delta)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(x[i],y[i]);
		}
	}
	ReferencePoint.x = x[ReferenceIndex];
	ReferencePoint.y = y[ReferenceIndex];
	double x1,y1; //start point
	double x2,y2; //end point
	for(int i=0;i<5;i++)
	{
		if(ReferenceIndex + i < 301)
		{
			x2 = (double)x[ReferenceIndex + i ];
			y2 = (double)y[ReferenceIndex + i ];
		}
		if(ReferenceIndex - i >= 0)
		{
			x1 = (double)x[ReferenceIndex - i ];
			y1 = (double)y[ReferenceIndex - i ];
		}
	}

	direct_angle =  atan2(x2-x1,y2-y1)*180.0/PI;
	direct_angle += dst_Ins.attitude[2];
	direct_angle = fmod(direct_angle+360.0,360.0);
	return true;
}

bool C_GisFrame::getRawPoint(Point &gisRawPoint,double &angle3K)
{
	double min_delta;
	Point CarPos(1500,1000);
	int gisRawPointIndex;
	for(int i=0;i<301;i++)
	{
		if(i == 0)
		{
			gisRawPointIndex = 0;
			double delta_x = (double)(CarPos.x-x[i]);
			double delta_y = (double)(CarPos.y-y[i]);
			min_delta = delta_x*delta_x + delta_y*delta_y;
		}
		else
		{
			double delta_x = (double)(CarPos.x-x[i]);
			double delta_y = (double)(CarPos.y-y[i]);
			double temp = delta_x*delta_x + delta_y*delta_y;
			if(temp<min_delta)
			{
				gisRawPointIndex = i;
				min_delta = temp;
			}
		}
	}
	gisRawPoint.x = x[gisRawPointIndex];
	gisRawPoint.y = y[gisRawPointIndex];
	double x1,y1; //start point
	double x2,y2; //end point
	for(int i=0;i<5;i++)
	{
		if(gisRawPointIndex + i < 301)
		{
			x2 = (double)x[gisRawPointIndex + i ];
			y2 = (double)y[gisRawPointIndex + i ];
		}
		if(gisRawPointIndex - i >= 0)
		{
			x1 = (double)x[gisRawPointIndex - i ];
			y1 = (double)y[gisRawPointIndex - i ];
		}
	}
	angle3K = atan2(x2-x1,y2-y1)*180.0/PI;
	return true;
}

bool C_GisFrame::getReferencePoint(C_StoplineModel *ReferenceLine,int &ReferenceIndex,Point &ReferencePoint,double &direct_angle,C_InsFrame dst_Ins)
{
	int min_delta;
	for(int i=0;i<301;i++)
	{
		if(i == 0)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(AlignedData().x[i],AlignedData().y[i]);
		}
		if(ReferenceLine->distanceToModel(AlignedData().x[i],AlignedData().y[i])<min_delta)
		{
			ReferenceIndex = i;
			min_delta = ReferenceLine->distanceToModel(AlignedData().x[i],AlignedData().y[i]);
		}
	}
	ReferencePoint.x = AlignedData().x[ReferenceIndex];
	ReferencePoint.y = AlignedData().y[ReferenceIndex];
	double x1,y1; //start point
	double x2,y2; //end point
	for(int i=0;i<5;i++)
	{
		if(ReferenceIndex + i < 301)
		{
			x2 = (double)AlignedData().x[i];
			y2 = (double)AlignedData().y[i];
		}
		if(ReferenceIndex - i >= 0)
		{
			x1 = (double)AlignedData().x[i];
			y1 = (double)AlignedData().y[i];
		}
	}

	direct_angle =  atan2(x2-x1,y2-y1)*180.0/PI;
	direct_angle += dst_Ins.attitude[2];
	direct_angle = fmod(direct_angle+360.0,360.0);
	return true;
}
