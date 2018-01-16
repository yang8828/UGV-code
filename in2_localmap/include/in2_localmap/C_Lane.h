/*
 * C_Lane.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_LANE_H_
#define C_LANE_H_

#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>

#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_Ins.h>
#include <in2_localmap/C_Stopline.h>
#include <in2_localmap/C_Linear.h>
#include <in2_localmap/C_Line.h>
#include <in2_localmap/C_BasicOp.h>
#include <in2_localmap/C_CostMap.h>

using namespace cv;

class C_LaneModel
{
public:
	C_LineSample StoplineSample[20];
	int s_len;
	C_LineSample LinearSample[20];
	int l_len;
	C_LineSample CurveSample[20];
	int c_len;
public:
	C_StoplineModelSet StoplineSet;
	C_LinearModelSet LinearSet;
	C_CurveModelSet  CurveSet;
	C_LaneModel()
	{
		s_len=0;
		l_len=0;
		c_len=0;
	}

	void processinput(const int32_t _ms, const boost::array<int, 20u> _r, const boost::array<int, 20u> _theta, const boost::array<int, 20u> _type, const boost::array<int, 20u> _beliefe, const boost::array<int, 20u> _isCurve, const boost::array<int, 620u> _points_x, const boost::array<int, 620u> _points_y, C_InsFrameHis insFrameHis)
	{
		C_InsFrame matchInsFrame;
		bool temp_withIns;
		temp_withIns = insFrameHis.getMatchInsFrame(_ms,matchInsFrame);
		if(temp_withIns == false)
		{
			return;
		}
		s_len = 0;
		l_len = 0;
		c_len = 0;
		for (int s=0;s<20;s++)
		{
			if (_type[s] == 0 )	continue;
			if (_type[s] == 5)
			{
				if( true )
				{
					StoplineSample[s_len].valid = true;
					StoplineSample[s_len].timestamp = _ms;
					StoplineSample[s_len].r = _r[s];
					StoplineSample[s_len].theta = _theta[s];
					StoplineSample[s_len].type = _type[s];
					StoplineSample[s_len].beliefe = _beliefe[s];
					StoplineSample[s_len].isCurve = _isCurve[s];
					for (int i=0;i<31;i++)
					{
						StoplineSample[s_len].points_x[i] = _points_x[s*31+i];
						StoplineSample[s_len].points_y[i] = _points_y[s*31+i];
					}
					StoplineSample[s_len].withIns = temp_withIns;
					StoplineSample[s_len].InsData.copyfrom(matchInsFrame);
					s_len ++;
				}
				continue;
			}
			if ( _isCurve[s] == 0)
			{
				LinearSample[l_len].valid = true;
				LinearSample[l_len].timestamp = _ms;
				LinearSample[l_len].r = _r[s];
				LinearSample[l_len].theta = _theta[s];
				LinearSample[l_len].type = _theta[s];
				LinearSample[l_len].beliefe = _beliefe[s];
				LinearSample[l_len].isCurve = _isCurve[s];
				for (int i=0;i<31;i++)
				{
					LinearSample[l_len].points_x[i] = _points_x[s*31+i];
					LinearSample[l_len].points_y[i] = _points_y[s*31+i];
				}
				LinearSample[l_len].withIns = temp_withIns;
				LinearSample[l_len].InsData.copyfrom(matchInsFrame);
				l_len ++;
			}
			else
			{
				CurveSample[c_len].valid = true;
				CurveSample[c_len].timestamp = _ms;
				CurveSample[c_len].r = _r[s];
				CurveSample[c_len].theta = _theta[s];
				CurveSample[c_len].type = _theta[s];
				CurveSample[c_len].beliefe = _beliefe[s];
				CurveSample[c_len].isCurve = _isCurve[s];
				for (int i=0;i<31;i++)
				{
					CurveSample[c_len].points_x[i] = _points_x[s*31+i];
					CurveSample[c_len].points_y[i] = _points_y[s*31+i];
				}
				CurveSample[c_len].withIns = temp_withIns;
				CurveSample[c_len].InsData.copyfrom(matchInsFrame);
				c_len++;
			}
		}// END:for (int s=0;s<20;s++)
	}

	void processinput(const int32_t _ms, const boost::array<int, 20u> _r, const boost::array<int, 20u> _theta, const boost::array<int, 20u> _type, const boost::array<int, 20u> _beliefe, const boost::array<int, 20u> _isCurve, const boost::array<int, 620u> _points_x, const boost::array<int, 620u> _points_y, C_InsFrameHis insFrameHis,double gis_3K_angle)
	{
		C_InsFrame matchInsFrame;
		bool temp_withIns;
		temp_withIns = insFrameHis.getMatchInsFrame(_ms,matchInsFrame);
		if(temp_withIns == false)
		{
			return;
		}
		s_len = 0;
		l_len = 0;
		c_len = 0;
		for (int s=0;s<20;s++)
		{
			if (_type[s] == 0 )	continue;
			if (_type[s] == 5)
			{
				if( 1 )// the restriction of the beliefe
				{
					double StopLineAngle3K = vectorangle3K(_points_x[s*31+30]-_points_x[s*31+0],_points_y[s*31+30]-_points_y[s*31+0]);
					if(byangle3KDiff(StopLineAngle3K,gis_3K_angle) > 10.0)
					{
						continue;
					}
					StoplineSample[s_len].valid = true;
					StoplineSample[s_len].timestamp = _ms;
					StoplineSample[s_len].r = _r[s];
					StoplineSample[s_len].theta = _theta[s];
					StoplineSample[s_len].type = _type[s];
					StoplineSample[s_len].beliefe = _beliefe[s];
					StoplineSample[s_len].beliefe = 15;
					StoplineSample[s_len].isCurve = _isCurve[s];
					for (int i=0;i<31;i++)
					{
						StoplineSample[s_len].points_x[i] = _points_x[s*31+i];
						StoplineSample[s_len].points_y[i] = _points_y[s*31+i];
					}
					StoplineSample[s_len].withIns = temp_withIns;
					StoplineSample[s_len].InsData.copyfrom(matchInsFrame);
					s_len ++;
				}
				continue;
			}
			if ( _isCurve[s] == 0)
			{
				double laneAngle3K = vectorangle3K(_points_x[s*31+30]-_points_x[s*31+0],_points_y[s*31+30]-_points_y[s*31+0]);
				if(angle3KDiff(laneAngle3K,gis_3K_angle) > 10.0)
				{
					continue;
				}
				LinearSample[l_len].valid = true;
				LinearSample[l_len].timestamp = _ms;
				LinearSample[l_len].r = _r[s];
				LinearSample[l_len].theta = _theta[s];
				LinearSample[l_len].type = _theta[s];
				LinearSample[l_len].beliefe = _beliefe[s];
				LinearSample[l_len].isCurve = _isCurve[s];
				for (int i=0;i<31;i++)
				{
					LinearSample[l_len].points_x[i] = _points_x[s*31+i];
					LinearSample[l_len].points_y[i] = _points_y[s*31+i];
				}
				LinearSample[l_len].withIns = temp_withIns;
				LinearSample[l_len].InsData.copyfrom(matchInsFrame);
				l_len ++;
			}
			else
			{
				CurveSample[c_len].valid = true;
				CurveSample[c_len].timestamp = _ms;
				CurveSample[c_len].r = _r[s];
				CurveSample[c_len].theta = _theta[s];
				CurveSample[c_len].type = _theta[s];
				CurveSample[c_len].beliefe = _beliefe[s];
				CurveSample[c_len].isCurve = _isCurve[s];
				for (int i=0;i<31;i++)
				{
					CurveSample[c_len].points_x[i] = _points_x[s*31+i];
					CurveSample[c_len].points_y[i] = _points_y[s*31+i];
				}
				CurveSample[c_len].withIns = temp_withIns;
				CurveSample[c_len].InsData.copyfrom(matchInsFrame);
				c_len++;
			}
		}// END:for (int s=0;s<20;s++)
	}

	long addSample(const int32_t _ms, const boost::array<int, 20u> _r,
			const boost::array<int, 20u> _theta, const boost::array<int, 20u> _type,
			const boost::array<int, 20u> _beliefe, const boost::array<int, 20u> _isCurve,
			const boost::array<int, 620u> _points_x, const boost::array<int, 620u> _points_y,
			C_InsFrameHis insFrameHis,C_GisFrame gisFrame, C_InsFrame cur_ins)
	{
		//processinput(_ms,_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y,insFrameHis,gisFrame.angle);
		processinput(_ms,_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y,insFrameHis);
		//StopLine:
#if PRINT_DEBUG_LANE
		std::cout<<"\n\n#########################################################"<<std::endl;
#endif
		StoplineSet.addSample(StoplineSample,s_len);
		StoplineSet.addAge();
		StoplineSet.SetFilter_Type2();
		StoplineSet.getLikeliestModelByGis(&gisFrame);
#if PRINT_DEBUG_LANE
		StoplineSet.printModelSetStatus();
		StoplineSet.printLikeliestModel();
#endif
		//Linear:
		LinearSet.addSample(LinearSample,l_len);
		LinearSet.addAge();
		LinearSet.SetFilter_type2();
		LinearSet.GetLikeliestModelByGis(gisFrame,_ms,cur_ins);
#if PRINT_DEBUG_LANE
		LinearSet.printModelSetStatus();
		LinearSet.printLikeliestModel();
#endif
		//Curve:
		//CurveSet.addSample(CurveSample,c_len);
		return _ms;
	}

	long addSample(const int32_t _ms, const boost::array<int, 20u> _r,
			const boost::array<int, 20u> _theta, const boost::array<int, 20u> _type,
			const boost::array<int, 20u> _beliefe,const boost::array<int, 20u> _isCurve,
			const boost::array<int, 620u> _points_x,const boost::array<int, 620u> _points_y,
			C_InsFrameHis insFrameHis,C_GisFrame gisFrame,C_CostMap costMap,C_InsFrame cur_ins)
	{
		//processinput(_ms,_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y,insFrameHis,gisFrame.angle);
		processinput(_ms,_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y,insFrameHis);
		//StopLine:
#if PRINT_DEBUG_LANE
		std::cout<<"\n\n#########################################################"<<std::endl;
#endif
		StoplineSet.addSample(StoplineSample,s_len);
		StoplineSet.addAge();
		StoplineSet.SetFilter_Type2();
		StoplineSet.getLikeliestModelByGis(&gisFrame);
#if PRINT_DEBUG_LANE
		StoplineSet.printModelSetStatus();
		StoplineSet.printLikeliestModel();
#endif
		//Linear:
		LinearSet.addSample(LinearSample,l_len);
		LinearSet.addAge();
		LinearSet.SetFilter_type2();
		//LinearSet.GetLikeliestModel(_ms,cur_ins);
		//LinearSet.GetLikeliestModelByGis(gisFrame,_ms,cur_ins);
		LinearSet.GetLikeliestModelByCostAndGis(gisFrame,costMap,_ms,cur_ins);
#if PRINT_DEBUG_LANE
		LinearSet.printModelSetStatus();
		LinearSet.printLikeliestModel();
#endif
		//Curve:
		//CurveSet.addSample(CurveSample,c_len);
		return _ms;
	}

	long addSample(const int32_t _ms, const boost::array<int, 20u> _r,
			const boost::array<int, 20u> _theta, const boost::array<int, 20u> _type,
			const boost::array<int, 20u> _beliefe,const boost::array<int, 20u> _isCurve,
			const boost::array<int, 620u> _points_x,const boost::array<int, 620u> _points_y,
			C_InsFrameHis insFrameHis,C_GisFrame gisFrame,C_CostMap costMap,C_RoadEdgeFrame road_edge_frame,C_InsFrame cur_ins)
	{
		//processinput(_ms,_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y,insFrameHis,gisFrame.angle);
		processinput(_ms,_r,_theta,_type,_beliefe,_isCurve,_points_x,_points_y,insFrameHis);
		//StopLine:
#if PRINT_DEBUG_LANE
		std::cout<<"\n\n#########################################################"<<std::endl;
#endif
		StoplineSet.addSample(StoplineSample,s_len);
		StoplineSet.addAge();
		StoplineSet.SetFilter_Type2();
		StoplineSet.getLikeliestModelByGis(&gisFrame);
#if PRINT_DEBUG_LANE
		StoplineSet.printModelSetStatus();
		StoplineSet.printLikeliestModel();
#endif
		//Linear:
		LinearSet.addSample(LinearSample,l_len);
		LinearSet.addAge();
		LinearSet.SetFilter_type2();
		//LinearSet.GetLikeliestModel(_ms,cur_ins);
		//LinearSet.GetLikeliestModelByGis(gisFrame,_ms,cur_ins);
		//LinearSet.GetLikeliestModelByCostAndGis(gisFrame,costMap,_ms,cur_ins);
#ifndef GIS_AND_HIS
		LinearSet.GetLikeliestModelByCostAndGisAndRoadEdge(gisFrame,costMap,road_edge_frame,cur_ins,_ms);
#else
		LinearSet.GetLikeliestModelByCostAndGisAndRoadEdgeAndHis(gisFrame,costMap,road_edge_frame,cur_ins,_ms);
#endif

#if PRINT_DEBUG_LANE
		LinearSet.printModelSetStatus();
		LinearSet.printLikeliestModel();
#endif
		//Curve:
		//CurveSet.addSample(CurveSample,c_len);
		return _ms;
	}

	void DrawLiearSample(Mat Image,C_InsFrame dst_InsFrame)
	{
		C_LineSample temp;
		for (int j=0;j<l_len;j++) //loop for all newSample
			for(int i=0;i<31;i++)
			{
				LinearSample[j].RTto(temp,dst_InsFrame);
				int x = temp.points_x[i];
				int y = 3000-temp.points_y[i];
				circle(Image,Point(x,y),6,LinearSampleColor,-1);
			}
	}

	void DrawStoplineSample(Mat Image,C_InsFrame dst_InsFrame)
	{
		C_LineSample temp;
		for (int j=0;j<s_len;j++) //loop for all newSample
			for(int i=0;i<31;i++)
			{
				StoplineSample[j].RTto(temp,dst_InsFrame);
				int x = temp.points_x[i];
				int y = 3000-temp.points_y[i];
				circle(Image,Point(x,y),6,StoplineSampleColor,-1);
			}
	}

};


#endif /* C_LANE_H_ */
