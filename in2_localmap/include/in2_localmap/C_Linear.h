/*
 * C_Linear.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_LINEAR_H_
#define C_LINEAR_H_


#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>


#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_RoadEdge.h>
#include <in2_localmap/C_Ins.h>
#include <in2_localmap/C_Stopline.h>
#include <in2_localmap/C_Gis.h>
#include <in2_localmap/C_BasicOp.h>
#include <in2_localmap/C_CostMap.h>

using namespace cv;

class C_CurveModel
{

};

class C_CurveModelSet
{
public:
	C_CurveModel CurveModelSet[MAX_CURVE_MODEL_NUM];
	void addSample(C_LineSample *newLineSamples)
	{
	}
	void updateTo(C_InsFrame dst_Ins);
};

class C_LinearModel
{
	//x = k*y+b+i*db;
public:
	bool with_data;
	double k,b,db;
	double default_lane_width;
	int age;
	double strength;
	double beliefe;
	C_LineSampleHis line_set[MAX_LINE_NUM];
	double total_cost[20];
	double map_cost[20]; //0~1
	double last_map_cost[20]; //0~1
	double belief_cost[20]; //0~1
	int ObjID;
	int LastObjID;
	bool ObjIDInitial;
	int last_eage_1_index;
	int last_eage_2_index;
	int OddLastDir;
	Point2d LastObjTwoPtsEarth[2];
	CvPoint2D64f HistoryEdge[2][2];
	long HistoryEdge_TS;
	bool haveHisEdge;

	double getFittest(C_LineSample sample,int &match_line_index)
	{
		double fittness_list[MAX_LINE_NUM];
		double min_value;
		int min_index;
		for (int i=0;i<MAX_LINE_NUM;i++)
		{
			fittness_list[i] = 0.0;
			fittness_list[i] += distanceToLine(i,sample.points_x[0],sample.points_y[0]);
			fittness_list[i] += distanceToLine(i,sample.points_x[30],sample.points_y[30]);
			fittness_list[i] /=2.0;
			if (i==0)
			{
				min_value = fittness_list[i];
				min_index = i;
			}
			else
			{
				if (fittness_list[i]<min_value)
				{
					min_value = fittness_list[i];
					min_index = i;
				}
			}
		}
		match_line_index = min_index;
		return min_value;
	}
	double distanceToLine(int index,int x,int y)
	{
		double double_i = (double) index;
		double double_x = (double) x;
		double double_y = (double) y;
		double distance = fabs(double_x-k*double_y-b-double_i*db)/sqrt(1+k*k);
		return distance;
	}
	C_LinearModel()
	{
		with_data = false;
		k = 0.0;
		b = 0.0;
		db = 0.0;
		default_lane_width = DEFAULT_LANE_WIDTH;
		age = 0;
		beliefe = 0.0;
		strength = 0.0;
		ObjID = 0;
		LastObjID = MAX_LINE_NUM/2;
		ObjIDInitial = true;
		OddLastDir = 1;
		haveHisEdge = false;
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			line_set[i].init();
			total_cost[i] = -1;
			map_cost[i] = -1;
			last_map_cost[i] = 0;
			belief_cost[i] = -1;
		}
	}
	C_LinearModel(C_LineSample newSample)
	{
		Mat A = (Mat_<double>(2, 2)<<newSample.points_y[0],1,newSample.points_y[30],1);
		Mat Y = (Mat_<double>(2, 1)<<newSample.points_x[0],newSample.points_x[30]);
		Mat AtA = A.t()*A;
		Mat p = AtA.inv()*A.t()*Y;
		double temp_k = p.at<double>(0,0);
		double temp_b_p_i_t_db = p.at<double>(1,0);
		k = temp_k;
		default_lane_width = DEFAULT_LANE_WIDTH;
		db = default_lane_width*sqrt(1+k*k);
		b = temp_b_p_i_t_db - 10*db;

		age = 0;
		beliefe = 0.0;
		strength = 0.0;
		ObjID = 0;
		LastObjID = MAX_LINE_NUM/2;
		ObjIDInitial = true;
		OddLastDir = 1;
		haveHisEdge = false;
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			line_set[i].init();
			last_map_cost[i] = 0;
		}
	}
	void initWith(C_LineSample newSample)
	{
		Mat A = (Mat_<double>(2, 2)<<newSample.points_y[0],1,newSample.points_y[30],1);
		Mat Y = (Mat_<double>(2, 1)<<newSample.points_x[0],newSample.points_x[30]);
		Mat AtA = A.t()*A;
		Mat p = AtA.inv()*A.t()*Y;
		double temp_k = p.at<double>(0,0);
		double temp_b_p_i_t_db = p.at<double>(1,0);
		k = temp_k;
		default_lane_width = DEFAULT_LANE_WIDTH;
		db = default_lane_width*sqrt(1+k*k);
		b = temp_b_p_i_t_db - 10*db;

		age = 0;
		beliefe = 0.0;
		strength = 0.0;
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			line_set[i].init();
		}
	}
	void init()
	{
		with_data = false;
		k = 0.0;
		b = 0.0;
		db = 0.0;
		default_lane_width = DEFAULT_LANE_WIDTH;
		age = 0;
		beliefe = 0.0;
		strength = 0.0;
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			line_set[i].init();
		}
	}
	void addLineSampe(int line_index,C_LineSample newframe)
	{
		line_set[line_index].addNewFrame(newframe);
		if(line_set[line_index].line_his[MAX_LINE_HIS_LEN-1].type!=0)
		{
			strength --;
			beliefe -= line_set[line_index].line_his[MAX_LINE_HIS_LEN-1].beliefe;
		}

		if(newframe.type !=0)
		{

			strength ++;
			beliefe += newframe.beliefe;
		}
	}
	void copyfrom(C_LinearModel src)
	{
		with_data = src.with_data;
		k	= src.k;
		b	= src.b;
		db	= src.db;
		default_lane_width = src.default_lane_width;
		for (int i=0;i<MAX_LINE_NUM;i++)
		{
			line_set[i].copyfrom(src.line_set[i]);
		}
		age = src.age;
		strength = src.strength;
		beliefe = src.beliefe;
	}
	void addAge()
	{
		age++;
		if(age > MAX_LINE_HIS_LEN) age = MAX_LINE_HIS_LEN;
	}
	void updateTo(C_InsFrame dst_Ins)
	{
		Mat XY_list[MAX_LINE_NUM];
		int index_list[MAX_LINE_NUM];
		int counter = 0;
		for (int i=0;i<MAX_LINE_NUM;i++)
		{
			Mat temp = line_set[i].getXYat(dst_Ins);
			//std::cout<<temp<<std::endl;
			if(temp.rows > 0)
			{
				XY_list[counter] = temp;
				index_list[counter] = i;
				counter++;
			}
		}
		if (counter==0)
		{
			//do nothing
			//k=k;
			//b=b;
			//db = db;
			counter = 0;
		}
		if (counter==1)
		{//A*p=Y
			int A_rows = XY_list[0].rows;
			Mat A = Mat::ones(A_rows,2,CV_64F);
			XY_list[0](Range(0,XY_list[0].rows),Range(1,2)).copyTo(A(Range(0,A.rows),Range(0,1)));
			Mat Y = XY_list[0](Range(0,XY_list[0].rows),Range(0,1));
			Mat AtA = A.t()*A;
			Mat p = AtA.inv()*A.t()*Y;
			double temp_k = p.at<double>(0,0);
			double temp_b_p_i_t_db = p.at<double>(1,0);
			k = temp_k;
			db = default_lane_width*sqrt(1+k*k);
			b = temp_b_p_i_t_db - index_list[0]*db;
		}
		if (counter>1)
		{
			int rows=0;
			for (int i=0;i<counter;i++)
				rows+=XY_list[i].rows;
			Mat A = Mat::ones(rows,3,CV_64F);
			Mat Y = Mat(rows,1,CV_64F);
			int s_index = 0;
			int end_index = 0;
			for (int i=0;i<counter;i++)
			{
				end_index = s_index+XY_list[i].rows;
				Mat temp_ones = Mat::ones(XY_list[i].rows,1,CV_64F)*(double)index_list[i];
				XY_list[i](Range(0,XY_list[i].rows),Range(1,2)).copyTo(A(Range(s_index,end_index),Range(0,1)));
				temp_ones.copyTo(A(Range(s_index,end_index),Range(2,3)));
				XY_list[i](Range(0,XY_list[i].rows),Range(0,1)).copyTo(Y(Range(s_index,end_index),Range(0,1)));
				s_index = end_index;
			}
			Mat AtA = A.t()*A;
			Mat p = AtA.inv()*A.t()*Y;
			double temp_k = p.at<double>(0,0);
			double temp_b = p.at<double>(1,0);
			double temp_db = p.at<double>(2,0);
			k = temp_k;
			db = temp_db;
			b = temp_b;
			default_lane_width = getLaneWidth();
		}
	}
	void printLineSetStatus()
	{
		std::cout<<"LineSetStatus:\n";
		std::cout<<"Idx: ";
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			fprintf(stdout,"%4d  ",i);
		}
		std::cout<<std::endl;
		std::cout<<"Age: ";
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			fprintf(stdout,"%4d  ",line_set[i].age);
		}
		std::cout<<std::endl;
		std::cout<<"Stg: ";
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			fprintf(stdout,"%4.0f  ",line_set[i].strength);
		}
		std::cout<<std::endl;
		std::cout<<"Blf: ";
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			fprintf(stdout,"%4.0f  ",line_set[i].beliefe);
		}
//Cost List::
		std::cout<<std::endl;
		std::cout<<"Map: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f  ",map_cost[i]);
		}
		std::cout<<std::endl;
		std::cout<<"Bel: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f  ",belief_cost[i]);
		}
		std::cout<<std::endl;
		std::cout<<"Tot: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f  ",total_cost[i]);
		}
		std::cout<<std::endl;
/*
		std::cout<<"LineSetStatus:\n";
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			std::cout<<"("<<line_set[i].age<<","<<line_set[i].strength<<","<<line_set[i].beliefe<<") ";
		}
		std::cout<<std::endl;
		*/
	}
	void getEageByCostAndGis(int &eage_1_index,int &eage_2_index,C_CostMap cost_map,C_GisFrame gisFrame,const int32_t _ms, C_InsFrame cur_ins)
	//return the lines whose cost is min
	{
#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
		fprintf(stdout,"getting EageByCostAndGis...\n");
#endif
		double total_cost[20];
		double map_cost[20]; //0~1
		double belief_cost[20]; //0~1
		double temp_x;
		double temp_y;
		double temp_i;
		//valuables for finding min_map_cost
		double min_cost_line_index;
		double min_cost_line_value;
		bool first_time_flag_1 = true;
		for(int i=0;i<20;i++)
		{
			belief_cost[i] = 1.0 - line_set[i].beliefe/16.0/((double)MAX_LINE_HIS_LEN);
			map_cost[i] = 0;
			for(int y=0;y<3000;y++)
			{
				temp_y = (double)y;
				temp_i = (double)i;
				temp_x = k*temp_y+b+temp_i*db;
				double dot_cost;
				int x;
				x = (int)temp_x;
				//int y;
				y = (int)temp_y;
				dot_cost = cost_map.getCostAt(x,y);
				map_cost[i]+=dot_cost;
			}
			map_cost[i] /= 3000.0;
			total_cost[i] = belief_cost[i]+map_cost[i];
			total_cost[i] /= 2.0;


			//finding min_map_cost
			if(first_time_flag_1 == true)
			{
				first_time_flag_1 = false;
				min_cost_line_index = i;
				min_cost_line_value = map_cost[i];
			}
			else
			{
				if(min_cost_line_value > map_cost[i])
				{
					min_cost_line_value = map_cost[i];
					min_cost_line_index = i;
				}
			}
		}
#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			std::cout<<"Set Cost Status:\n";
			std::cout<<"Idx: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4d ",i);
			}
			std::cout<<std::endl;
			std::cout<<"Map: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4.2f ",map_cost[i]);
			}
			std::cout<<std::endl;
			std::cout<<"Bel: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4.2f ",belief_cost[i]);
			}
			std::cout<<std::endl;
			std::cout<<"Tot: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4.2f ",total_cost[i]);
			}
			std::cout<<std::endl;
#endif



		int edge_1_regg;
		int edge_2_regg;
		if(regionGrowing(map_cost,20,min_cost_line_index,edge_1_regg,edge_2_regg,REGION_GROWING_MAX_DIST))
		{

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			fprintf(stdout,"Finding Eage by Region Growing...\n");
			fprintf(stdout,"edge_regg = %d\t%d\n",edge_1_regg,edge_2_regg);
#endif
		}



		int laneNumTotal = gisFrame.laneNum;
		if(laneNumTotal > 0 && laneNumTotal<=12)
		{
			int laneNumThis;
			if(laneNumTotal % 2 == 1)
			{
				laneNumThis = laneNumTotal;
			}
			else
			{
//				double temp = 0.5*(double)laneNumTotal;
//				laneNumThis = (int)temp;
				laneNumThis = laneNumTotal;
			}
			int lineNum = laneNumThis + 1;
#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			fprintf(stdout,"laneNumThis = %d\n",laneNumThis);
			fprintf(stdout,"lineNum     = %d\n",lineNum);
#endif
			double set_cost[20]; //0.0~1.0
			double set_num = 20 - laneNumThis;
			double min_set_cost;
			double min_set_cost_index;
			bool first_time_flag  = true;
			for(int i=0;i<set_num;i++)
			{
				set_cost[i] = 0.0;
				for(int j=0;j<lineNum;j++)
				{
					set_cost[i] += total_cost[i+j];
				}
				set_cost[i] /= (double)lineNum;
				if( first_time_flag == true )
				{
					first_time_flag = false;
					min_set_cost = set_cost[i];
					min_set_cost_index = i;
				}
				else
				{
					if(set_cost[i] < min_set_cost)
					{
						min_set_cost = set_cost[i];
						min_set_cost_index = i;
					}
				}
			}
			eage_1_index = min_set_cost_index;
			eage_2_index = min_set_cost_index+lineNum - 1;

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			std::cout<<"Set Cost Status:\n";
			std::cout<<"Idx: ";
			for(int i=0;i<set_num;i++)
			{
				fprintf(stdout,"%4d ",i);
			}
			std::cout<<std::endl;
			std::cout<<"Set: ";
			for(int i=0;i<set_num;i++)
			{
				fprintf(stdout,"%4.2f ",set_cost[i]);
			}
			std::cout<<std::endl;
#endif

		}
		else
		{
			fprintf(stderr,"ERROR: laneNumBidir == 0, Getting Eages without GIS\n");
			getEage(eage_1_index,eage_2_index,_ms,cur_ins);
		}
	}
	void getEdegByBorderAndGis(int &edge_1_index,int &edge_2_index,int Border_1_index,int Border_2_index,C_GisFrame gisFrame,const int32_t _ms, C_InsFrame cur_ins)
	{
		int laneNumTotal = gisFrame.laneNum;
		if(laneNumTotal > 0 && laneNumTotal<=12)
		{
			int laneNumThis;
			if(laneNumTotal % 2 == 1)//odd
			{
				laneNumThis = laneNumTotal;
			}
			else
			{
//				double temp = 0.5*(double)laneNumTotal;
//				laneNumThis = (int)temp;
				laneNumThis = laneNumTotal;
			}
			int lineNum = laneNumThis + 1;
#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			fprintf(stdout,"laneNumThis = %d\n",laneNumThis);
			fprintf(stdout,"lineNum     = %d\n",lineNum);
#endif
			edge_1_index = Border_2_index - (lineNum-1);
			edge_2_index = Border_2_index;
			}
		else
		{
			fprintf(stderr,"ERROR: laneNumBidir == 0, Getting Edges without GIS\n");
			getEage(edge_1_index,edge_2_index,_ms,cur_ins);
		}
	}

	void getEageByCostAndGis_2(int &eage_1_index,int &eage_2_index,C_CostMap cost_map,C_GisFrame gisFrame,const int32_t _ms, C_InsFrame cur_ins)
	// return the lines in the right side in border;
	{

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
		fprintf(stdout,"getting EageByCostAndGis...\n");
#endif
		double total_cost[20];
		double map_cost[20]; //0~1
		double belief_cost[20]; //0~1
		double temp_x;
		double temp_y;
		double temp_i;
		//valuables for finding min_map_cost
		double min_cost_line_index;
		double min_cost_line_value;
		bool first_time_flag_1 = true;
		for(int i=0;i<20;i++)
		{
			belief_cost[i] = 1.0 - line_set[i].beliefe/16.0/((double)MAX_LINE_HIS_LEN);
			map_cost[i] = 0;
			for(int y=0;y<3000;y++)
			{
				temp_y = (double)y;
				temp_i = (double)i;
				temp_x = k*temp_y+b+temp_i*db;
				double dot_cost;
				int x;
				x = (int)temp_x;
				//int y;
				y = (int)temp_y;
				dot_cost = cost_map.getCostAt(x,y);
				map_cost[i]+=dot_cost;
			}
			map_cost[i] /= 3000.0;
			total_cost[i] = belief_cost[i]+map_cost[i];
			total_cost[i] /= 2.0;


			//finding min_map_cost
			if(first_time_flag_1 == true)
			{
				first_time_flag_1 = false;
				min_cost_line_index = i;
				min_cost_line_value = map_cost[i];
			}
			else
			{
				if(min_cost_line_value > map_cost[i])
				{
					min_cost_line_value = map_cost[i];
					min_cost_line_index = i;
				}
			}
		}

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			std::cout<<"Set Cost Status:\n";
			std::cout<<"Idx: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4d ",i);
			}
			std::cout<<std::endl;
			std::cout<<"Map: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4.2f ",map_cost[i]);
			}
			std::cout<<std::endl;
			std::cout<<"Bel: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4.2f ",belief_cost[i]);
			}
			std::cout<<std::endl;
			std::cout<<"Tot: ";
			for(int i=0;i<20;i++)
			{
				fprintf(stdout,"%4.2f ",total_cost[i]);
			}
			std::cout<<std::endl;
#endif



		int edge_1_regg;
		int edge_2_regg;
		if(regionGrowing(map_cost,20,min_cost_line_index,edge_1_regg,edge_2_regg,REGION_GROWING_MAX_DIST))
		{
			fprintf(stdout,"Finding Eage by Region Growing...\n");
			fprintf(stdout,"edge_regg = %d\t%d\n",edge_1_regg,edge_2_regg);
		}



		int laneNumTotal = gisFrame.laneNum;
		if(laneNumTotal > 0 && laneNumTotal<=12)
		{
			int laneNumThis;
			if(laneNumTotal % 2 == 1)
			{
				laneNumThis = laneNumTotal;
			}
			else
			{
//				double temp = 0.5*(double)laneNumTotal;
//				laneNumThis = (int)temp;
				laneNumThis = laneNumTotal;
			}
			int lineNum = laneNumThis + 1;

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
			fprintf(stdout,"laneNumThis = %d\n",laneNumThis);
			fprintf(stdout,"lineNum     = %d\n",lineNum);
#endif
			eage_1_index = edge_2_regg - (lineNum-1);
			eage_2_index = edge_2_regg;

		}
		else
		{
			fprintf(stderr,"ERROR: laneNumBidir == 0, Getting Eages without GIS\n");
			getEage(eage_1_index,eage_2_index,_ms,cur_ins);
		}
	}


	int refreshCostList(C_CostMap cost_map)
	// input variable Xinyu region [left,right]
	{
		double temp_x;
		double temp_y;
		double temp_i;
		//valuables for finding min_map_cost
		double min_cost_line_index;
		double min_cost_line_value;
		bool first_time_flag_1 = true;
		for(int i=0;i<20;i++)
		{
			belief_cost[i] = 1.0 - line_set[i].beliefe/16.0/((double)MAX_LINE_HIS_LEN);
			map_cost[i] = 0;
			/* note for song wenjie
			 * IF X = k*Y + b + i*db in Xinyu region [left,right]
			 *     map_cost[i] = 0;
			 * else
			 *     map_cost[i] = 3000;
			 * end
			 *
			 * kill the following loop;
			 */
			for(int y=0;y<3000;y++)
			{
				temp_y = (double)y;
				temp_i = (double)i;
				temp_x = k*temp_y+b+temp_i*db;
				double dot_cost;
				int x;
				x = (int)temp_x;
				//int y;
				y = (int)temp_y;
				dot_cost = cost_map.getCostAt(x,y);
				map_cost[i]+=dot_cost;
			}
			map_cost[i] /= 3000.0;
			total_cost[i] = belief_cost[i]+map_cost[i];
			total_cost[i] /= 2.0;

			//finding min_map_cost
			if(first_time_flag_1 == true)
			{
				first_time_flag_1 = false;
				min_cost_line_index = i;
				min_cost_line_value = map_cost[i];
			}
			else
			{
				if(min_cost_line_value > map_cost[i])
				{
					min_cost_line_value = map_cost[i];
					min_cost_line_index = i;
				}
			}
		}
		return min_cost_line_index;
	}

	int refreshCostList(double le_dis, double ri_dis)
	// input variable Xinyu region [left,right]
	{
		//valuables for finding min_map_cost
		double min_cost_line_index;
		double min_cost_line_value;
		bool first_time_flag_1 = true;
		for(int i=0;i<20;i++)
		{
			belief_cost[i] = 1.0 - line_set[i].beliefe/16.0/((double)MAX_LINE_HIS_LEN);

			// -------------------------------------------------------------------------------
			// parameter
			double coe = 0.5;
			double LaneWidth = fabs(db); // approximate
			double lT1 = -le_dis - coe * LaneWidth * 0.8;
			double lT2 = -le_dis - coe * LaneWidth;
			double rT1 = ri_dis + coe * LaneWidth * 0.8;
			double rT2 = ri_dis + coe * LaneWidth;
			// new code
			double cur_dis = - (1500-1000*k-b-i*db) / sqrt(1+k*k);
			if(cur_dis < lT2 || cur_dis > rT2)
			{
				map_cost[i] = 3000;
			}
			else if(cur_dis > lT1 && cur_dis < rT1)
			{
				map_cost[i] = 0;
			}
			else
			{
				map_cost[i] = last_map_cost[i];
			}
			last_map_cost[i] = map_cost[i];
			// -------------------------------------------------------------------------------

			map_cost[i] /= 3000.0;
			total_cost[i] = belief_cost[i]+map_cost[i];
			total_cost[i] /= 2.0;

			//finding min_map_cost
			if(first_time_flag_1 == true)
			{
				first_time_flag_1 = false;
				min_cost_line_index = i;
				min_cost_line_value = map_cost[i];
			}
			else
			{
				if(min_cost_line_value > map_cost[i])
				{
					min_cost_line_value = map_cost[i];
					min_cost_line_index = i;
				}
			}
		}
		return min_cost_line_index;
	}

	void getBorderByCostAndGisAndRoadEdge(int &border_1_index,int &border_2_index,C_CostMap cost_map,C_GisFrame gisFrame,C_RoadEdgeFrame road_edge_frame)
	{
		int min_cost_line_index;
		//-------------------------------------------------
		double dx = k;
		double dy = 1.0;
		double tmp_ang = 180 * atan2(dy, dx) / 3.14; // degree
		tmp_ang -= 90;
		double model_angle = -tmp_ang;
		double DeltaAngle = road_edge_frame.Angle - model_angle;
		double AngleThre = 20.0; // degree
		if(DeltaAngle>AngleThre || DeltaAngle<-AngleThre)
		{
			min_cost_line_index = refreshCostList(cost_map);
		}
		else
		{
			min_cost_line_index = refreshCostList(road_edge_frame.leftdis*50, road_edge_frame.rightdis*50);
		}
		//-------------------------------------------------
		regionGrowing(map_cost,20,min_cost_line_index,border_1_index,border_2_index,REGION_GROWING_MAX_DIST);
		if(border_1_index > border_2_index)
		{
			int temp = border_1_index;
			border_1_index = border_2_index;
			border_2_index = temp;
		}

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
		fprintf(stdout,"Finding Border by Region Growing...\n");
		fprintf(stdout,"edge_regg = %d\t%d\n",border_1_index,border_2_index);
		std::cout<<"Border Cost Status:\n";
		std::cout<<"Idx: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4d ",i);
		}
		std::cout<<std::endl;
		std::cout<<"Map: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f ",map_cost[i]);
		}
		std::cout<<std::endl;
		std::cout<<"Bel: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f ",belief_cost[i]);
		}
			std::cout<<std::endl;
		std::cout<<"Tot: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f ",total_cost[i]);
		}
		std::cout<<std::endl;
#endif
	}

	void getBorderByCostAndGis(int &border_1_index,int &border_2_index,C_CostMap cost_map,C_GisFrame gisFrame)
	{
		int min_cost_line_index;
		min_cost_line_index = refreshCostList(cost_map);
		regionGrowing(map_cost,20,min_cost_line_index,border_1_index,border_2_index,REGION_GROWING_MAX_DIST);
		if(border_1_index > border_2_index)
		{
			int temp = border_1_index;
			border_1_index = border_2_index;
			border_2_index = temp;
		}

#if PRINT_DEBUG_LANE && PRINT_DEBUG_REGIONGROWING
		fprintf(stdout,"Finding Border by Region Growing...\n");
		fprintf(stdout,"edge_regg = %d\t%d\n",border_1_index,border_2_index);
		std::cout<<"Border Cost Status:\n";
		std::cout<<"Idx: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4d ",i);
		}
		std::cout<<std::endl;
		std::cout<<"Map: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f ",map_cost[i]);
		}
		std::cout<<std::endl;
		std::cout<<"Bel: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f ",belief_cost[i]);
		}
			std::cout<<std::endl;
		std::cout<<"Tot: ";
		for(int i=0;i<20;i++)
		{
			fprintf(stdout,"%4.2f ",total_cost[i]);
		}
		std::cout<<std::endl;
#endif
	}

	int getEageByHis(int &eage_1_index, int &eage_2_index, C_InsFrame cur_ins, const int32_t _ms)
	{
		int tmp = getHistoryID(cur_ins, eage_1_index, eage_2_index, _ms);
		if(!(tmp == -1))
		{
			logHistoryEdge(eage_1_index, eage_2_index, cur_ins, _ms);
		}
		return tmp;
	}

	int getEageByGis(int &eage_1_index, int &eage_2_index, C_GisFrame gisFrame, C_RoadEdgeFrame road_edge_frame, C_InsFrame cur_ins, const int32_t _ms)
	{
		//------------------------start-------------------------
/*
		int line_num_in_road = 0;
		for(int i=0; i<MAX_LINE_NUM; i++)
		{
			if(map_cost[i] == 0)
			{
				line_num_in_road++;
			}
		}
		int lane_num_in_road = line_num_in_road - 1;
		if(lane_num_in_road != gisFrame.laneNum)
		{
			eage_1_index = last_eage_1_index;
			eage_2_index = last_eage_2_index;
			return 0;
		}
*/
		//------------------------end---------------------------

		double ModelAngle = getModelAngle();
		double DeltaAngle = fabs(road_edge_frame.Angle - ModelAngle);
		if(DeltaAngle > 45)
		{
			return -1;
		}

		double edge_centre_dis = (-road_edge_frame.leftdis + road_edge_frame.rightdis) / 2.0; // m
		edge_centre_dis *= 50.0;
		double PosThreInit = db * 0.41;
		double NegThreInit = -db * 0.61;
		double PosThre = db * 0.35;
		double NegThre = -db * 0.55;
		ObjID = -1;

		if(ObjIDInitial)
		{
			ObjIDInitial = false;
			for(int i=0; i<MAX_LINE_NUM; i++)
			{
				double cur_dis = - (1500-1000*k-b-i*db) / sqrt(1+k*k);
				double tmpdelta = edge_centre_dis - cur_dis;
				if(tmpdelta<PosThreInit && tmpdelta>NegThreInit)
				{
					ObjID = i;
					break;
				}
			}
			if(ObjID == -1)
			{
				return -1;
			}
		}
		else
		{
			for(int i=0; i<MAX_LINE_NUM; i++)
			{
				double cur_dis = - (1500-1000*k-b-i*db) / sqrt(1+k*k);
				double tmpdelta = edge_centre_dis - cur_dis;
				if(tmpdelta<PosThre && tmpdelta>NegThre)
				{
					ObjID = i;
					break;
				}
			}
			if(ObjID == -1)
			{
				ObjID = getLastObjID(cur_ins);
			}
		}
//		ROS_ERROR("ObjID = %d", ObjID);

		logLastObjLine(ObjID, cur_ins);

		eage_1_index = ObjID - gisFrame.laneNum / 2;
		eage_2_index = ObjID + gisFrame.laneNum / 2;

		if(gisFrame.laneNum%2 == 1)
		{
			double OddThreSmall = -db * 0.25;
			double OddThreBig   = -db * 0.05;
			int index_11 = eage_1_index - 1;
			int index_22 = eage_2_index + 1;
			double cur_dis_1 = - (1500-1000*k-b-index_11*db) / sqrt(1+k*k);
			double cur_dis_2 = - (1500-1000*k-b-index_22*db) / sqrt(1+k*k);
//			ROS_ERROR("cur_dis_1 = %lf", cur_dis_1);
//			ROS_ERROR("cur_dis_2 = %lf", cur_dis_2);
			double tmpdelta_1 = fabs(edge_centre_dis - cur_dis_1);
			double tmpdelta_2 = fabs(edge_centre_dis - cur_dis_2);
//			ROS_ERROR("edge_centre_dis = %lf", edge_centre_dis);
			if(tmpdelta_1 - tmpdelta_2 < OddThreSmall)
			{
				eage_1_index--;
				OddLastDir = -1;
			}
			else if(tmpdelta_1 - tmpdelta_2 > OddThreBig)
			{
				eage_2_index++;
				OddLastDir = 1;
			}
			else
			{
				if(OddLastDir == -1)
				{
					eage_1_index--;
				}
				else if(OddLastDir == 1)
				{
					eage_2_index++;
				}
			}
		}

		logHistoryEdge(eage_1_index, eage_2_index, cur_ins, _ms);

		return 0;
	}

	void getEageByGis(int &eage_1_index,int &eage_2_index,C_GisFrame gisFrame,const int32_t _ms,C_InsFrame cur_ins)
	{
		int on_left_side_of_Gis_index;
		int on_right_side_of_Gis_index;
		Point ref_p;
		double ref_angle3K;
		gisFrame.initialAlignedData().getRawPoint(ref_p,ref_angle3K);
		int index_1 = -1;
		int index_2 = -1;
		bool ispositive_now;
		bool ispositive_last;
		bool first_time_flag = true;
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			if(first_time_flag == true)
			{
				first_time_flag = false;
				if(Discriminant(i,ref_p.x,ref_p.y) >=0 )
					ispositive_now = true;
				else
					ispositive_now = false;
				ispositive_last = ispositive_now;
			}
			else
			{
				if(Discriminant(i,ref_p.x,ref_p.y) >=0 )
					ispositive_now = true;
				else
					ispositive_now = false;
				if(ispositive_last != ispositive_now)
				{
					index_1 = i-1;
					index_2 = i;
					break;
				}
				ispositive_last = ispositive_now;
			}
		}//END:for(int i=0;i<MAX_LINE_NUM;i++)
		if(index_1 == -1)
		{
			on_left_side_of_Gis_index = -1;
			on_right_side_of_Gis_index = -1;
		}
		else
		{
			on_left_side_of_Gis_index = index_1;
			on_right_side_of_Gis_index = index_2;
		}


		int laneNumTotal = gisFrame.laneNum;
		if(laneNumTotal > 0 && laneNumTotal<=12)
		{
			eage_1_index = on_left_side_of_Gis_index;
			eage_2_index = on_right_side_of_Gis_index;
			int laneNumThis;
			//-----------------------------------------------------------------------------
			if(laneNumTotal % 2 == 1)
			{
				laneNumThis = laneNumTotal;
			}
			else
			{
//				double temp = 0.5*(double)laneNumTotal;
//				laneNumThis = (int)temp;
				laneNumThis = laneNumTotal;
			}
			//-----------------------------------------------------------------------------
			int laneToadd = laneNumThis - 1;
			bool isleft = true;
			for(int i=0;i<laneToadd;i++)
			{
				if(isleft == true)
				{
					isleft = false;
					if(eage_1_index>0)
						eage_1_index -=1;
				}
				else
				{
					isleft = true;
					if(eage_2_index<MAX_LINE_NUM-1)
						eage_2_index +=1;
				}
			}
			logHistoryEdge(eage_1_index, eage_2_index, cur_ins, _ms);
		}
		else
		{
			if(laneNumTotal == 0)
				fprintf(stderr,"ERROR: laneNumTotal == 0, Getting Eages without GIS\n");
			else
				fprintf(stderr,"ERROR: laneNumTotal > 12, Getting Eages without GIS\n");
			int detected_left_index;
			int detected_right_index;
			getEage(detected_left_index,detected_right_index,_ms,cur_ins);
			eage_1_index = detected_left_index;
			eage_2_index = detected_right_index;
		}
	}
	void getEage(int &eage_1_index,int &eage_2_index,const int32_t _ms,C_InsFrame cur_ins)
	{
		eage_1_index = MAX_LINE_NUM;
		for(int i=0;i<MAX_LINE_NUM;i++)
		{
			if(line_set[i].beliefe < 0 || line_set[i].strength<0 )
				continue;
			double temp = line_set[i].beliefe/line_set[i].strength;
			if(temp > EAGE_THRESHOLD_1 || line_set[i].strength > EAGE_THRESHOLD_2)
			{
				eage_1_index=i;
				break;
			}
		}
		eage_2_index = -1;
		for(int i=MAX_LINE_NUM-1;i>=0;i--)
		{
			if(line_set[i].beliefe < 0 || line_set[i].strength<0 )
				continue;
			double temp = line_set[i].beliefe/line_set[i].strength;
			if(temp > EAGE_THRESHOLD_1 || line_set[i].strength > EAGE_THRESHOLD_2)
			{
				eage_2_index=i;
				break;
			}
		}
		if(eage_1_index == eage_2_index)
		{
			int left_index = eage_1_index -1;
			int right_index = eage_1_index +1;
			if(left_index < 0)
				eage_2_index = right_index;
			else
				if(right_index >= MAX_LINE_NUM) {
					eage_1_index = left_index;
				}
				else {
					if( line_set[left_index].beliefe >  line_set[right_index].beliefe )
						eage_1_index = left_index;
					else
						eage_2_index = right_index;
				}

		}
		if(eage_2_index < eage_1_index)
		{
			int swap = eage_2_index;
			eage_2_index = eage_1_index;
			eage_1_index = swap;

		}
		logHistoryEdge(eage_1_index, eage_2_index, cur_ins, _ms);
	}
	double directAgnle(C_InsFrame dst_Ins)
	{
		double temp = dst_Ins.attitude[2]+atan(k)*180.0/PI;
		temp = fmod(temp+360.0,360.0);
		return temp;
	}
	double getLaneWidth()
	{
		return db/sqrt(k*k+1.0);
	}
	double getAngle()
	//int 3000 coordinate system
	{
		return atan(k)*180.0/PI;
	}
	void Draw(Mat &Image,int i1,int i2,Scalar color = LinearSetColor,bool puttext = true,int line_thickness = 5)
	{
		if(i1<0 || i1>=MAX_LINE_NUM)
			return;
		if(i2<0 || i2>=MAX_LINE_NUM)
			return;
		if(i1>i2)
		{
			int temp = i2;
			i2 = i1;
			i1 = temp;
		}
		for(int i=i1;i<=i2;i++)
		{
			Draw(Image,i,color,puttext,line_thickness);
		}
	}
	void Draw(Mat &Image,int i = 10,Scalar color = LinearSetColor,bool puttext = true,int line_thickness = 5)
	{
		//x = k*y+b+i*db;
		int y_1 = 0;
		int x_1 = k*y_1 + b + i*db;
		int y_2 = 3000;
		int x_2 = k*y_2 + b + i*db;
		line(Image,Point(x_1,3000-y_1),Point(x_2,3000-y_2),color,line_thickness);
		int y_3 = 850;
		int x_3 = k*y_3 + b + i*db;
		if(puttext == true)
		{
			char str[3];
			sprintf(str,"%d",i);
			int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
			double fontScale = 2.5;
			int thickness = 6;
			putText(Image, str, Point(x_3,3000-y_3), fontFace, fontScale, Scalar(200,200,200), thickness+12,8);
			putText(Image, str, Point(x_3,3000-y_3), fontFace, fontScale, color, thickness,8);
		}
	}
	double Discriminant(int index, int x,int y)
	{
		double double_i = (double) index;
		double double_x = (double) x;
		double double_y = (double) y;
		double funValue = double_x-k*double_y-b-double_i*db;
		return funValue;
	}
	bool getMid(C_LinearModel &MidModel,int l1_index,int l2_index,C_InsFrame dst_Ins)
	{
		C_LinearModel mid;
		mid.copyfrom(*this);
		mid.updateTo(dst_Ins);
		int index_1;
		int index_2 = -1;
		double min_value;
		bool firsttime_flag_1 = true;
		for(int i=l1_index;i<=l2_index;i++)
		{
			if(firsttime_flag_1 == true)
			{
				firsttime_flag_1 = false;
				index_1 = i;
				min_value = mid.distanceToLine(i,1500,1000);
			}
			else
			{
				double temp = mid.distanceToLine(i,1500,1000);
				if(temp<min_value)
				{
					index_1 = i;
					min_value = temp;
				}
			}
		}
		bool firsttime_flag_2 = true;
		for(int i=l1_index;i<=l2_index;i++)
		{
			if(i != index_1)
			{
				if(firsttime_flag_2 == true)
				{
					firsttime_flag_2 = false;
					index_2 = i;
					min_value = mid.distanceToLine(i,1500,1000);
				}
				else
				{
					double temp = mid.distanceToLine(i,1500,1000);
					if(temp<min_value)
					{
						index_2 = i;
						min_value = temp;
					}
				}
			}
		}
		if(index_2 == -1)
		{
			fprintf(stderr,"ERROR 1:C_LinearModel getMid(C_InsFrame dst_Ins)\n");
			return false;
		}
		if(index_1 > index_2)
		{
			int swap = index_1;
			index_1 = index_2;
			index_2 = swap;
		}
		if(index_1 != index_2-1)
		{
			fprintf(stderr,"index_1: %d ,index_2: %d\n",index_1,index_2);
			fprintf(stderr,"ERROR 2:C_LinearModel getMid(C_InsFrame dst_Ins)\n");
			return false;
		}
		double discrim_1 = Discriminant(index_1,1500,1000);
		double discrim_2 = Discriminant(index_2,1500,1000);
		if(discrim_1 * discrim_2 <= 0)
		{
			if(discrim_1 != discrim_2)
			{
				//mid.b+10*mid.db = temp.b+(index_1+0.5)*temp.db
				mid.b = mid.b+(((double)index_1)+0.5-10.0)*mid.db;
				MidModel.copyfrom(mid);
				return true;
			}
		}
		return false;
	}
	bool getMid(Point gisRawPoint,C_LinearModel &MidModel,C_InsFrame dst_Ins)
	//find two lines on each side of the gisRawPoint, and return the mid of these two line
	{
		C_LinearModel mid;
		mid.copyfrom(*this);
		mid.updateTo(dst_Ins);
		int index_1;
		int index_2 = -1;
		double min_value;
		bool firsttime_flag_1 = true;
		for(int i=0;i<=MAX_LINE_NUM;i++)
		{
			if(firsttime_flag_1 == true)
			{
				firsttime_flag_1 = false;
				index_1 = i;
				min_value = mid.distanceToLine(i,gisRawPoint.x,gisRawPoint.y);
			}
			else
			{
				double temp = mid.distanceToLine(i,gisRawPoint.x,gisRawPoint.y);
				if(temp<min_value)
				{
					index_1 = i;
					min_value = temp;
				}
			}
		}
		bool firsttime_flag_2 = true;
		for(int i=0;i<=MAX_LINE_NUM;i++)
		{
			if(i != index_1)
			{
				if(firsttime_flag_2 == true)
				{
					firsttime_flag_2 = false;
					index_2 = i;
					min_value = mid.distanceToLine(i,gisRawPoint.x,gisRawPoint.y);
				}
				else
				{
					double temp = mid.distanceToLine(i,gisRawPoint.x,gisRawPoint.y);
					if(temp<min_value)
					{
						index_2 = i;
						min_value = temp;
					}
				}
			}
		}
		if(index_2 == -1)
		{
			fprintf(stderr,"ERROR 1:C_LinearModel getMid(C_InsFrame dst_Ins)\n");
			return false;
		}
		if(index_1 > index_2)
		{
			int swap = index_1;
			index_1 = index_2;
			index_2 = swap;
		}
		if(index_1 != index_2-1)
		{
			fprintf(stderr,"index_1: %d ,index_2: %d\n",index_1,index_2);
			fprintf(stderr,"ERROR 2:C_LinearModel getMid(C_InsFrame dst_Ins)\n");
			return false;
		}
		double discrim_1 = Discriminant(index_1,gisRawPoint.x,gisRawPoint.y);
		double discrim_2 = Discriminant(index_2,gisRawPoint.x,gisRawPoint.y);
		if(discrim_1 * discrim_2 <= 0)
		{
			if(discrim_1 != discrim_2)
			{
				//mid.b+10*mid.db = temp.b+(index_1+0.5)*temp.db
				mid.b = mid.b+(((double)index_1)+0.5-10.0)*mid.db;
				MidModel.copyfrom(mid);
				return true;
			}
		}
		return false;
	}
	bool getMid(C_LinearModel &MidModel,C_InsFrame dst_Ins,int line_1_index,int line_2_index)
	{
		double temp_i;
		temp_i = line_1_index+line_2_index;
		temp_i = temp_i/2.0;
		if(fmod(temp_i,1.0) == 0.0)
			temp_i += 0.5;

		C_LinearModel mid;
		mid.copyfrom(*this);
		mid.updateTo(dst_Ins);
		mid.b = mid.b + (temp_i - 10.0)*mid.db;
		MidModel.copyfrom(mid);
		return true;
	}
	void getPerpenditualFoot(int line_index,double src_x,double src_y,double &foot_x,double &foot_y)
	{
		//x - k*y -b-i*db = 0;
		double i = (double)line_index;
		double A = 1;
		double B = -k;
		double C = -b - db*i;
		getPerpendicularFoot(A,B,C,src_x,src_y,foot_x,foot_y);
	}
	void getPerpenditualFoot(int line_index,int src_x,int src_y,int &foot_x,int &foot_y)
	{
			//x - k*y -b-i*db = 0;
		double i = (double)line_index;
		double A = 1;
		double B = -k;
		double C = -b - db*i;
		getPerpendicularFoot(A,B,C,src_x,src_y,foot_x,foot_y);
	}
	void getPerpenditualFoot(int line_index,int src_x,int src_y,double &foot_x,double &foot_y)
		{
			//x - k*y -b-i*db = 0;
			double i = (double)line_index;
			double A = 1;
			double B = -k;
			double C = -b - db*i;
			double x = (double)src_x;
			double y = (double)src_y;
			getPerpendicularFoot(A,B,C,x,y,foot_x,foot_y);
		}

	C_StoplineModel getVert()
	{
		//x = k*y+b+i*db;
		C_StoplineModel vert;
		int f_x;
		int f_y;
		double a = 1;
		double b = -k;

		double c = -b -10.0*db;
		getPerpendicularFoot(a,b,c,1500,1000,f_x,f_y);
		vert.setWith(1500.0,1000.0,-k);
		return vert;
	}

	void logHistoryEdge(int id_1, int id_2, C_InsFrame cur_ins, const int32_t _ms)
	{
		Point tmpt_0(k*1000+b+id_1*db, 1000);
		Point tmpt_1(k*2000+b+id_1*db, 2000);
		HistoryEdge[0][0] = conv3000toENU(tmpt_0.x, tmpt_0.y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		HistoryEdge[0][1] = conv3000toENU(tmpt_1.x, tmpt_1.y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);

		tmpt_0 = Point(k*1000+b+id_2*db, 1000);
		tmpt_1 = Point(k*2000+b+id_2*db, 2000);
		HistoryEdge[1][0] = conv3000toENU(tmpt_0.x, tmpt_0.y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		HistoryEdge[1][1] = conv3000toENU(tmpt_1.x, tmpt_1.y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);

		HistoryEdge_TS = _ms;
		haveHisEdge = true;
	}

	void logLastObjLine(int id, C_InsFrame cur_ins)
	{
		Point tmpt_0(k*1000+b+id*db, 1000);
		Point tmpt_1(k*2000+b+id*db, 2000);
		CvPoint2D64f ena_0 = conv3000toENU(tmpt_0.x, tmpt_0.y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		CvPoint2D64f ena_1 = conv3000toENU(tmpt_1.x, tmpt_1.y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		LastObjTwoPtsEarth[0].x = ena_0.x;
		LastObjTwoPtsEarth[0].y = ena_0.y;
		LastObjTwoPtsEarth[1].x = ena_1.x;
		LastObjTwoPtsEarth[1].y = ena_1.y;
	}

	int getHistoryID(C_InsFrame cur_ins, int &id_1, int &id_2, const int32_t _ms)
	{
		if(HistoryEdge_TS - _ms > HIS_EDGE_TS_THRE)
		{
			haveHisEdge = false;
			return -1;
		}

		CvPoint local_0 = convENUto3000(HistoryEdge[0][0].x, HistoryEdge[0][0].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		CvPoint local_1 = convENUto3000(HistoryEdge[0][1].x, HistoryEdge[0][1].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);

		double A,B,C;
		if(local_0.y - local_1.y == 0)
		{
			A = 0;
			B = 1;
			C = -local_0.y;
		}
		else
		{
			A = 1;
			B = -(local_0.x - local_1.x)/(local_0.y - local_1.y);
			C = -local_0.x - B * local_0.y;
		}

		double his_dis = - (A*1500+B*1000+C) / sqrt(A*A + B*B);
		double tmpdeltamin = 99999;
		double tdid = MAX_LINE_NUM/2;
		for(int i=0; i<MAX_LINE_NUM; i++)
		{
			double cur_dis = - (1500-1000*k-b-i*db) / sqrt(1+k*k);
			double tmpdelta = fabs(his_dis - cur_dis);
			if(tmpdelta < tmpdeltamin)
			{
				tdid = i;
				tmpdeltamin = tmpdelta;
			}
		}
		id_1 = tdid;

		local_0 = convENUto3000(HistoryEdge[1][0].x, HistoryEdge[1][0].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		local_1 = convENUto3000(HistoryEdge[1][1].x, HistoryEdge[1][1].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);

		if(local_0.y - local_1.y == 0)
		{
			A = 0;
			B = 1;
			C = -local_0.y;
		}
		else
		{
			A = 1;
			B = -(local_0.x - local_1.x)/(local_0.y - local_1.y);
			C = -local_0.x - B * local_0.y;
		}

		his_dis = - (A*1500+B*1000+C) / sqrt(A*A + B*B);
		tmpdeltamin = 99999;
		tdid = MAX_LINE_NUM/2;
		for(int i=0; i<MAX_LINE_NUM; i++)
		{
			double cur_dis = - (1500-1000*k-b-i*db) / sqrt(1+k*k);
			double tmpdelta = fabs(his_dis - cur_dis);
			if(tmpdelta < tmpdeltamin)
			{
				tdid = i;
				tmpdeltamin = tmpdelta;
			}
		}
		id_2 = tdid;

		return 0;
	}

	int getLastObjID(C_InsFrame cur_ins)
	{
		CvPoint local_0 = convENUto3000(LastObjTwoPtsEarth[0].x, LastObjTwoPtsEarth[0].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
		CvPoint local_1 = convENUto3000(LastObjTwoPtsEarth[1].x, LastObjTwoPtsEarth[1].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);

		double A,B,C;
		if(local_0.y - local_1.y == 0)
		{
			A = 0;
			B = 1;
			C = -local_0.y;
		}
		else
		{
			A = 1;
			B = -(local_0.x - local_1.x)/(local_0.y - local_1.y);
			C = -local_0.x - B * local_0.y;
		}

		double his_dis = - (A*1500+B*1000+C) / sqrt(A*A + B*B);
		double tmpdeltamin = 99999;
		double tdid = MAX_LINE_NUM/2;
		for(int i=0; i<MAX_LINE_NUM; i++)
		{
			double cur_dis = - (1500-1000*k-b-i*db) / sqrt(1+k*k);
			double tmpdelta = fabs(his_dis - cur_dis);
			if(tmpdelta < tmpdeltamin)
			{
				tdid = i;
				tmpdeltamin = tmpdelta;
			}
		}

		return tdid;
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

	double getModelAngle()
	{
		double dx = k;
		double dy = 1.0;
		double tmp_ang = 180 * atan2(dy, dx) / 3.14; // degree
		tmp_ang -= 90;
		double model_angle = -tmp_ang;

		return model_angle;
	}

};

class C_LinearModelSet
{
private:
	C_LineSample associatedSample[MAX_LINEAR_MODEL_NUM][MAX_LINE_NUM];
public:
	C_LinearModel LinearModelSet[MAX_LINEAR_MODEL_NUM];

	int likiestModel_index;
	int line_1_index;
	int line_2_index;
	int border_1_index;
	int border_2_index;
	int cur_len;
	int GisOrEdge;
	int LastGisOrEdge;
	int ContiUseGisCnter;
	long int RecoveryCnter;
	C_LinearModelSet()
	{
		cur_len = 0;
		likiestModel_index = -1;
		line_1_index = -1;
		line_2_index = -1;
		border_1_index = -1;
		border_2_index = -1;
		GisOrEdge = 1;
		LastGisOrEdge = 1;
		ContiUseGisCnter = 0;
		RecoveryCnter = 0;
	}

	void init_associatedSample()
	{
		for(int i=0;i<MAX_LINEAR_MODEL_NUM;i++)
		{
			for(int j=0;j<MAX_LINE_NUM;j++)
			{
				associatedSample[i][j].init();
			}
		}
	}

	void addSample(C_LineSample *newLineSamples,int len)
		{
			int new_len = cur_len;
			init_associatedSample();
	#if PRINT_DEBUG_LANE
			std::cout << "\n### LineSample Len:" << len << "\tLinearModelSet Len:" << cur_len << " ###\n";
			ROS_WARN("\n### LineSample Len:%d \tLinearModelSet Len:%d ###", len, cur_len);
	#endif
			if(len>0)
			{
				if(newLineSamples[0].withIns == true)
					updateTo(newLineSamples[0].InsData);
				bool not_enougth_linear_model_space = false;
				while (1)
				{
					for (int i = 0; i < new_len; i++) //loop for all existed model
					{
						for (int j = 0; j < len; j++) //loop for all newSample
						{
//							if (newLineSamples[j].valid == 0)
//								continue;
							int match_line_index;
							double fittness = LinearModelSet[i].getFittest(
									newLineSamples[j], match_line_index);
	#if PRINT_DEBUG_LANE
							fprintf(stdout,"line %02d with Old Set %02d %02d Fit %06.1f\n",j,i,match_line_index,fittness);
							ROS_WARN("line %02d with Old Set %02d %02d Fit %06.1f\n",j,i,match_line_index,fittness);
	#endif
							//std::cout << "line " << j << " with Old Set " << i << " " << match_line_index << " Fit " << fittness << "\n";
//							if (fittness < FITNESS_THRESHOLD)
							if (fittness < LinearModelSet[i].default_lane_width*0.35)
							{
								associatedSample[i][match_line_index].copyfrom(newLineSamples[j]);
								newLineSamples[j].valid = 0;
							}

						}
					}
					for (int j = 0; j < len; j++) // add to new Model
					{
						if (newLineSamples[j].valid == 1)
						{
							if(new_len == MAX_LINEAR_MODEL_NUM)
							{
								not_enougth_linear_model_space = true;
								break;
							}
							//C_LinearModel newLinearModel(newLineSamples[j]);
							//LinearModelSet[new_len].copyfrom(newLinearModel);
							LinearModelSet[new_len].initWith(newLineSamples[j]);
	#if PRINT_DEBUG_LANE
							fprintf(stdout,"line %02d with New Set %02d %02d Fit %06.1f\n",j,new_len,10,0.0);
							ROS_WARN("line %02d with New Set %02d %02d Fit %06.1f\n",j,new_len,10,0.0);
	#endif
							//std::cout << "line " << j << " with New Set " << new_len << " " << 10 << " Fit " << 0 << "\n";
							associatedSample[new_len][10].copyfrom(newLineSamples[j]);
							new_len++;
							newLineSamples[j].valid = 0;
							break;
						}
					}
					if(not_enougth_linear_model_space == true)
						break;
					bool no_new_left = true;
					for (int j = 0; j < len; j++)
					{
						if (newLineSamples[j].valid == 1)
						{
							no_new_left = false;
							break;
						}
					}
					if (no_new_left == true)
					{
						break;
					}
				} //END: while(1)
			}

			//fprintf(stderr,"##BreakPoint 01##\n");
			for (int i = 0; i < new_len; i++)
			{
				for (int j = 0; j < MAX_LINE_NUM; j++)
				{
					LinearModelSet[i].addLineSampe(j, associatedSample[i][j]);
					//LinearModelSet[i].line_set[j].addNewFrame(associatedSample[i][j]);
				}

			}
			cur_len = new_len;
			//fprintf(stderr,"cur_len: %d\n",cur_len);
		}

//	void addSample(C_LineSample *newLineSamples,int len)
//	{
//		int new_len = cur_len;
//		init_associatedSample();
//#if PRINT_DEBUG_LANE
//		std::cout << "\n### LineSample Len:" << len << "\tLinearModelSet Len:" << cur_len << " ###\n";
//		ROS_WARN("\n### LineSample Len:%d \tLinearModelSet Len:%d ###", len, cur_len);
//#endif
//		if(len>0)
//		{
//			if(newLineSamples[0].withIns == true)
//				updateTo(newLineSamples[0].InsData);
//			bool not_enougth_linear_model_space = false;
//			while (1)
//			{
//				for (int i = 0; i < new_len; i++) //loop for all existed model
//				{
//					for (int j = 0; j < len; j++) //loop for all newSample
//					{
//						if (newLineSamples[j].valid == 0)
//							continue;
//						int match_line_index;
//						double fittness = LinearModelSet[i].getFittest(
//								newLineSamples[j], match_line_index);
//#if PRINT_DEBUG_LANE
//						fprintf(stdout,"line %02d with Old Set %02d %02d Fit %06.1f\n",j,i,match_line_index,fittness);
//						ROS_WARN("line %02d with Old Set %02d %02d Fit %06.1f\n",j,i,match_line_index,fittness);
//#endif
//						//std::cout << "line " << j << " with Old Set " << i << " " << match_line_index << " Fit " << fittness << "\n";
////						if (fittness < FITNESS_THRESHOLD)
//						if (fittness < LinearModelSet[i].default_lane_width*0.35)
//						{
//							associatedSample[i][match_line_index].copyfrom(newLineSamples[j]);
//							newLineSamples[j].valid = 0;
//						}
//
//					}
//				}
//				for (int j = 0; j < len; j++) // add to new Model
//				{
//					if (newLineSamples[j].valid == 1)
//					{
//						if(new_len == MAX_LINEAR_MODEL_NUM)
//						{
//							not_enougth_linear_model_space = true;
//							break;
//						}
//						//C_LinearModel newLinearModel(newLineSamples[j]);
//						//LinearModelSet[new_len].copyfrom(newLinearModel);
//						LinearModelSet[new_len].initWith(newLineSamples[j]);
//#if PRINT_DEBUG_LANE
//						fprintf(stdout,"line %02d with New Set %02d %02d Fit %06.1f\n",j,new_len,10,0.0);
//						ROS_WARN("line %02d with New Set %02d %02d Fit %06.1f\n",j,new_len,10,0.0);
//#endif
//						//std::cout << "line " << j << " with New Set " << new_len << " " << 10 << " Fit " << 0 << "\n";
//						associatedSample[new_len][10].copyfrom(newLineSamples[j]);
//						new_len++;
//						newLineSamples[j].valid = 0;
//						break;
//					}
//				}
//				if(not_enougth_linear_model_space == true)
//					break;
//				bool no_new_left = true;
//				for (int j = 0; j < len; j++)
//				{
//					if (newLineSamples[j].valid == 1)
//					{
//						no_new_left = false;
//						break;
//					}
//				}
//				if (no_new_left == true)
//				{
//					break;
//				}
//			} //END: while(1)
//		}
//
//		//fprintf(stderr,"##BreakPoint 01##\n");
//		for (int i = 0; i < new_len; i++)
//		{
//			for (int j = 0; j < MAX_LINE_NUM; j++)
//			{
//				LinearModelSet[i].addLineSampe(j, associatedSample[i][j]);
//				//LinearModelSet[i].line_set[j].addNewFrame(associatedSample[i][j]);
//			}
//
//		}
//		cur_len = new_len;
//		//fprintf(stderr,"cur_len: %d\n",cur_len);
//	}

	void addAge(void)
	{
		for(int i=0;i<cur_len;i++)
		{
			LinearModelSet[i].addAge();
		}
	}

	void DeleteSet(int ModelIndex)
	{
		if(cur_len == 0 || ModelIndex >= cur_len || ModelIndex < 0)
		{
			fprintf(stderr,"ERROR in  Delete Set");
			return;
		}
		for(int i=ModelIndex;i<cur_len-1;i++)
		{
			LinearModelSet[i].copyfrom(LinearModelSet[i+1]);
		}
		LinearModelSet[cur_len-1].init();
		cur_len --;

	}

	double getLaneWidth()
	{
		return LinearModelSet[likiestModel_index].db/sqrt(LinearModelSet[likiestModel_index].k*LinearModelSet[likiestModel_index].k+1.0);
	}
	void SetFilter_type2()
	{
#if PRINT_DEBUG_LANE
		std::cout<<"Set Filting Tpye2... ";
#endif
		for(int i=0;i<cur_len;i++)
		{
			if(LinearModelSet[i].strength < 0.1)
			{
				DeleteSet(i);
#if PRINT_DEBUG_LANE
				std::cout<<i<<"S ";
#endif
			}
		}
		for(int i=0;i<cur_len;i++)
		{
			if(LinearModelSet[i].age > 2)
			{
				double lane_width = LinearModelSet[i].db/sqrt(LinearModelSet[i].k*LinearModelSet[i].k+1.0);
				lane_width = fabs(lane_width);
				if(lane_width<MIN_LANE_WIDTH || lane_width > MAX_LANE_WIDTH)
				{
					DeleteSet(i);
#if PRINT_DEBUG_LANE
					std::cout<<i<<"W "<<lane_width;
#endif
				}
			}
		}
		if(cur_len > MAX_LINEAR_MODEL_NUM-1 )
		{
			int min_index = -1;
			double min_val;
			bool first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				if(LinearModelSet[i].age == MAX_LINE_HIS_LEN )
				{
					if(first_time_flag == true)
					{
						first_time_flag = false;
						min_index = i;
						min_val = LinearModelSet[i].beliefe;
					}
					else
					{
						if(LinearModelSet[i].beliefe<min_val)
						{
							min_index = i;
							min_val = LinearModelSet[i].beliefe;
						}
					}
				}
			}//end: for(int i=0;i<cur_len;i++)
			if(min_index != -1)
			{
				DeleteSet(min_index);
#if PRINT_DEBUG_LANE
				std::cout<<min_index<<" ";
#endif
			}
		}
#if PRINT_DEBUG_LANE
		std::cout<<std::endl;
#endif
	}
	void SetFilter(void)
	{
#if PRINT_DEBUG_LANE
		std::cout<<"Set Filting Tpye1... ";
#endif
		for(int i=0;i<cur_len;i++)
		{
			if(LinearModelSet[i].age == MAX_LINE_HIS_LEN )
			{
				if(LinearModelSet[i].beliefe < MODEL_BELIEFE_THRESHOLD )
				{
					DeleteSet(i);
#if PRINT_DEBUG_LANE
					std::cout<<i<<"B ";
#endif
					continue;
				}
				double lane_width = LinearModelSet[i].db/sqrt(LinearModelSet[i].k*LinearModelSet[i].k+1.0);
				lane_width = fabs(lane_width);
				if(lane_width<MIN_LANE_WIDTH  || lane_width > MAX_LANE_WIDTH)
				{
					DeleteSet(i);
#if PRINT_DEBUG_LANE
					std::cout<<i<<"W "<<lane_width;
#endif
					continue;
				}
			}
		}
#if PRINT_DEBUG_LANE
		std::cout<<std::endl;
#endif
	}

	int GetLikeliestModel(const int32_t _ms,C_InsFrame cur_ins)
	{
		if(cur_len == 0)
		{
			likiestModel_index = -1;
		}
		if(cur_len == 1)
		{

			if(LinearModelSet[0].strength > LIKELIEST_LINEAR_MODEL_THREASHOLD_1)
			{
				likiestModel_index = 0;
			}
			else
			{
				likiestModel_index = -1;
			}
		}
		else
		{
			int likeliest_index = 0;
			double likeliest_value = 0;
			double first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				if(LinearModelSet[i].strength > 0)
				{
					if(first_time_flag==true)
					{
						first_time_flag = false;
						likeliest_index = i;
						likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
						//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
						//	likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
						//else
						//	likeliest_value = LinearModelSet[i].strength;
					}
					else
					{
						double temp_value ;
						temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
						//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
						//	temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
						//else
						//	temp_value = LinearModelSet[i].strength;
						if(temp_value > likeliest_value)
						{
							likeliest_value = temp_value;
							likeliest_index = i;
						}
					}
				} //END: if(LinearModelSet[i].strength > 0)
			}
			if(first_time_flag == false)
			{
				likiestModel_index = likeliest_index;
			}
			else
			{
				likiestModel_index = -1;
			}
		}//END: if(cur_len == 1) else
		LinearModelSet[likiestModel_index].getEage(line_1_index,line_2_index,_ms,cur_ins);
		return likiestModel_index;
	}

	int GetLikeliestModelByCostAndGisAndRoadEdgeAndHis(C_GisFrame gisFrame,C_CostMap costmap,C_RoadEdgeFrame road_edge_frame,C_InsFrame cur_ins,const int32_t _ms)
		{
			RecoveryCnter++;
			if(cur_len == 0)
			{
				likiestModel_index = -1;
			}
			if(cur_len == 1)
			{

				if(LinearModelSet[0].strength > LIKELIEST_LINEAR_MODEL_THREASHOLD_1)
				{
					double angle_Error = byangle3KDiff(LinearModelSet[0].getAngle(),gisFrame.angle);
					if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
						likiestModel_index = 0;
					else
						likiestModel_index = -1;
				}
				else
				{
					likiestModel_index = -1;
				}
			}
			else
			{
				int likeliest_index = 0;
				double likeliest_value = 0;
				double first_time_flag = true;
				for(int i=0;i<cur_len;i++)
				{
					double angle_Error = byangle3KDiff(LinearModelSet[i].getAngle(),gisFrame.angle);
					if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
					{
						if(LinearModelSet[i].strength > 0)
						{
							if(first_time_flag==true)
							{
								first_time_flag = false;
								likeliest_index = i;
								likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
								//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
								//	likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
								//else
								//	likeliest_value = LinearModelSet[i].strength;
							}
							else
							{
								double temp_value ;
								temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
								//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
								//	temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
								//else
								//	temp_value = LinearModelSet[i].strength;
								if(temp_value > likeliest_value)
								{
									likeliest_value = temp_value;
									likeliest_index = i;
								}
							}
						} //END: if(LinearModelSet[i].strength > 0)
					}
				}
				if(first_time_flag == false)
				{
					likiestModel_index = likeliest_index;
				}
				else
				{
					likiestModel_index = -1;
				}
			}//END: if(cur_len == 1) else

			//get line_1_index, line_2_index and edge_1_index, edge_2_index
			ROS_ERROR("likiestModel_index = %d", likiestModel_index);
			if(likiestModel_index != -1)
			{
				LinearModelSet[likiestModel_index].getBorderByCostAndGisAndRoadEdge(border_1_index,border_2_index,costmap,gisFrame,road_edge_frame);
				if(gisFrame.laneNum > 0 && gisFrame.laneNum <100)
				{
					if(RecoveryCnter > (0.3*1000/10)) // 0.3s
					{
						RecoveryCnter = (0.3*1000/10);
						LastGisOrEdge = 1;
						LinearModelSet[likiestModel_index].ObjIDInitial = true;
						LinearModelSet[likiestModel_index].OddLastDir = 1;
					}
	//----------------------------------------------------------------------------------------------------------------
	//				int line_num_in_road = 0;
	//				for(int i=0; i<MAX_LINE_NUM; i++)
	//				{
	//					if(LinearModelSet[likiestModel_index].map_cost[i] == 0)
	//					{//----------------------------------------------------------------------------------------------------------------
	//						line_num_in_road++;
	//					}
	//				}
	//				int lane_num_in_road = line_num_in_road - 1;
	//
	//				if(abs(lane_num_in_road-gisFrame.laneNum) > 0)
	//				{
	//					LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,cur_ins);
	//				}
	//				else
	//----------------------------------------------------------------------------------------------------------------
					double edge_road_width = road_edge_frame.leftdis + road_edge_frame.rightdis; // m
					edge_road_width *= 50.0;
					double gis_road_width = gisFrame.laneNum * LinearModelSet[likiestModel_index].db;
					double SmallThre_1 = gis_road_width + LinearModelSet[likiestModel_index].db * (-1.2);
					double SmallThre_2 = gis_road_width + LinearModelSet[likiestModel_index].db * (-0.40);
					double BigThre_1   = gis_road_width + LinearModelSet[likiestModel_index].db * (0.40);
					double BigThre_2   = gis_road_width + LinearModelSet[likiestModel_index].db * (0.95);
					if(edge_road_width < SmallThre_1 || edge_road_width > BigThre_2)
					{
						GisOrEdge = 0;
					}
					else if(edge_road_width > SmallThre_2 && edge_road_width < BigThre_1)
					{
						GisOrEdge = 1;
					}
					else
					{
						GisOrEdge = LastGisOrEdge;
					}
					LastGisOrEdge = GisOrEdge;

					if(GisOrEdge == 0)
					{
						Point gisRawPoint;
						double gisRawPoint_angle3K;
						gisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K);
						if(angle3KDiff(gisRawPoint_angle3K,LinearModelSet[likiestModel_index].getAngle()) > HIS_GIS_ANG_THRE && LinearModelSet[likiestModel_index].haveHisEdge)
						{
							double tmp = LinearModelSet[likiestModel_index].getEageByHis(line_1_index,line_2_index,cur_ins,_ms);
							if(tmp == -1)
							{
								LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
							}
						}
						else
						{
							LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
						}
						ContiUseGisCnter++;
						if(ContiUseGisCnter > (0.3*1000/10)) // 0.3s
						{
							ContiUseGisCnter = (0.3*1000/10);
							LinearModelSet[likiestModel_index].ObjIDInitial = true;
							LinearModelSet[likiestModel_index].OddLastDir = 1;
						}
					}
					else
	//----------------------------------------------------------------------------------------------------------------
					{
						ContiUseGisCnter = 0;
						if(LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,road_edge_frame,cur_ins,_ms))
						{
							LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
						}
					}
					RecoveryCnter = 0;
				}
				else
				{
					LinearModelSet[likiestModel_index].getEage(line_1_index,line_2_index,_ms,cur_ins);
				}
			}
			else
			{
				line_1_index = -1;
				line_2_index = -1;
				border_1_index = -1;
				border_2_index = -1;
			}
			return likiestModel_index;
		}

	int GetLikeliestModelByCostAndGisAndRoadEdge(C_GisFrame gisFrame,C_CostMap costmap,C_RoadEdgeFrame road_edge_frame,C_InsFrame cur_ins,const int32_t _ms)
	{
		RecoveryCnter++;
		if(cur_len == 0)
		{
			likiestModel_index = -1;
		}
		if(cur_len == 1)
		{

			if(LinearModelSet[0].strength > LIKELIEST_LINEAR_MODEL_THREASHOLD_1)
			{
				double angle_Error = byangle3KDiff(LinearModelSet[0].getAngle(),gisFrame.angle);
				if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
					likiestModel_index = 0;
				else
					likiestModel_index = -1;
			}
			else
			{
				likiestModel_index = -1;
			}
		}
		else
		{
			int likeliest_index = 0;
			double likeliest_value = 0;
			double first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				double angle_Error = byangle3KDiff(LinearModelSet[i].getAngle(),gisFrame.angle);
				if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
				{
					if(LinearModelSet[i].strength > 0)
					{
						if(first_time_flag==true)
						{
							first_time_flag = false;
							likeliest_index = i;
							likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
							//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
							//	likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
							//else
							//	likeliest_value = LinearModelSet[i].strength;
						}
						else
						{
							double temp_value ;
							temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
							//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
							//	temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
							//else
							//	temp_value = LinearModelSet[i].strength;
							if(temp_value > likeliest_value)
							{
								likeliest_value = temp_value;
								likeliest_index = i;
							}
						}
					} //END: if(LinearModelSet[i].strength > 0)
				}
			}
			if(first_time_flag == false)
			{
				likiestModel_index = likeliest_index;
			}
			else
			{
				likiestModel_index = -1;
			}
		}//END: if(cur_len == 1) else

		//get line_1_index, line_2_index and edge_1_index, edge_2_index
		if(likiestModel_index != -1)
		{
			LinearModelSet[likiestModel_index].getBorderByCostAndGisAndRoadEdge(border_1_index,border_2_index,costmap,gisFrame,road_edge_frame);
			if(gisFrame.laneNum > 0 && gisFrame.laneNum <100)
			{
				if(RecoveryCnter > (0.3*1000/10)) // 0.3s
				{
					RecoveryCnter = (0.3*1000/10);
					LastGisOrEdge = 1;
					LinearModelSet[likiestModel_index].ObjIDInitial = true;
					LinearModelSet[likiestModel_index].OddLastDir = 1;
				}
//----------------------------------------------------------------------------------------------------------------
//				int line_num_in_road = 0;
//				for(int i=0; i<MAX_LINE_NUM; i++)
//				{
//					if(LinearModelSet[likiestModel_index].map_cost[i] == 0)
//					{//----------------------------------------------------------------------------------------------------------------
//						line_num_in_road++;
//					}
//				}
//				int lane_num_in_road = line_num_in_road - 1;
//
//				if(abs(lane_num_in_road-gisFrame.laneNum) > 0)
//				{
//					LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,cur_ins);
//				}
//				else
//----------------------------------------------------------------------------------------------------------------
				double edge_road_width = road_edge_frame.leftdis + road_edge_frame.rightdis; // m
				edge_road_width *= 50.0;
				double gis_road_width = gisFrame.laneNum * LinearModelSet[likiestModel_index].db;
				double SmallThre_1 = gis_road_width + LinearModelSet[likiestModel_index].db * (-1.2);
				double SmallThre_2 = gis_road_width + LinearModelSet[likiestModel_index].db * (-0.40);
				double BigThre_1   = gis_road_width + LinearModelSet[likiestModel_index].db * (0.40);
				double BigThre_2   = gis_road_width + LinearModelSet[likiestModel_index].db * (0.95);
				if(edge_road_width < SmallThre_1 || edge_road_width > BigThre_2)
				{
					GisOrEdge = 0;
				}
				else if(edge_road_width > SmallThre_2 && edge_road_width < BigThre_1)
				{
					GisOrEdge = 1;
				}
				else
				{
					GisOrEdge = LastGisOrEdge;
				}
				LastGisOrEdge = GisOrEdge;

				if(GisOrEdge == 0)
				{
					LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
					ContiUseGisCnter++;
					if(ContiUseGisCnter > (0.3*1000/10)) // 0.3s
					{
						ContiUseGisCnter = (0.3*1000/10);
						LinearModelSet[likiestModel_index].ObjIDInitial = true;
						LinearModelSet[likiestModel_index].OddLastDir = 1;
					}
				}
				else
//----------------------------------------------------------------------------------------------------------------
				{
					ContiUseGisCnter = 0;
					if(LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,road_edge_frame,cur_ins,_ms))
					{
						LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
					}
				}
				RecoveryCnter = 0;
			}
			else
			{
				LinearModelSet[likiestModel_index].getEage(line_1_index,line_2_index,_ms,cur_ins);
			}
		}
		else
		{
			line_1_index = -1;
			line_2_index = -1;
			border_1_index = -1;
			border_2_index = -1;
		}
		return likiestModel_index;
	}

	int GetLikeliestModelByCostAndGis(C_GisFrame gisFrame,C_CostMap costmap,const int32_t _ms,C_InsFrame cur_ins)
	{
		if(cur_len == 0)
		{
			likiestModel_index = -1;
		}
		if(cur_len == 1)
		{

			if(LinearModelSet[0].strength > LIKELIEST_LINEAR_MODEL_THREASHOLD_1)
			{
				double angle_Error = byangle3KDiff(LinearModelSet[0].getAngle(),gisFrame.angle);
				if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
					likiestModel_index = 0;
				else
					likiestModel_index = -1;
			}
			else
			{
				likiestModel_index = -1;
			}
		}
		else
		{
			int likeliest_index = 0;
			double likeliest_value = 0;
			double first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				double angle_Error = byangle3KDiff(LinearModelSet[i].getAngle(),gisFrame.angle);
				if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
				{
					if(LinearModelSet[i].strength > 0)
					{
						if(first_time_flag==true)
						{
							first_time_flag = false;
							likeliest_index = i;
							likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
							//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
							//	likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
							//else
							//	likeliest_value = LinearModelSet[i].strength;
						}
						else
						{
							double temp_value ;
							temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
							//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
							//	temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
							//else
							//	temp_value = LinearModelSet[i].strength;
							if(temp_value > likeliest_value)
							{
								likeliest_value = temp_value;
								likeliest_index = i;
							}
						}
					} //END: if(LinearModelSet[i].strength > 0)
				}
			}
			if(first_time_flag == false)
			{
				likiestModel_index = likeliest_index;
			}
			else
			{
				likiestModel_index = -1;
			}
		}//END: if(cur_len == 1) else

		//get line_1_index, line_2_index and edge_1_index, edge_2_index
		if(likiestModel_index != -1)
		{
			LinearModelSet[likiestModel_index].getBorderByCostAndGis(border_1_index,border_2_index,costmap,gisFrame);
			if(gisFrame.laneNum > 0 && gisFrame.laneNum <100)
			{
				LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
			}
			else
			{
				LinearModelSet[likiestModel_index].getEage(line_1_index,line_2_index,_ms,cur_ins);
			}
		}
		else
		{
			line_1_index = -1;
			line_2_index = -1;
			border_1_index = -1;
			border_2_index = -1;
		}
		return likiestModel_index;
	}

	int GetLikeliestModelByGis(C_GisFrame gisFrame,const int32_t _ms,C_InsFrame cur_ins)
	{
		if(cur_len == 0)
		{
			likiestModel_index = -1;
		}
		if(cur_len == 1)
		{

			if(LinearModelSet[0].strength > LIKELIEST_LINEAR_MODEL_THREASHOLD_1)
			{
				double angle_Error = angle3KDiff(LinearModelSet[0].getAngle(),gisFrame.angle);
				if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
					likiestModel_index = 0;
				else
					likiestModel_index = -1;
			}
			else
			{
				likiestModel_index = -1;
			}
		}
		else
		{
			int likeliest_index = 0;
			double likeliest_value = 0;
			double first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				double angle_Error = angle3KDiff(LinearModelSet[i].getAngle(),gisFrame.angle);
				if(angle_Error < LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE)
				{
					if(LinearModelSet[i].strength > 0)
					{
						if(first_time_flag==true)
						{
							first_time_flag = false;
							likeliest_index = i;
							likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
							//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
							//	likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
							//else
							//	likeliest_value = LinearModelSet[i].strength;
						}
						else
						{
							double temp_value ;
							temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength + LinearModelSet[i].strength*8;
							//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
							//	temp_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
							//else
							//	temp_value = LinearModelSet[i].strength;
							if(temp_value > likeliest_value)
							{
								likeliest_value = temp_value;
								likeliest_index = i;
							}
						}
					} //END: if(LinearModelSet[i].strength > 0)
				}
			}
			if(first_time_flag == false)
			{
				likiestModel_index = likeliest_index;
			}
			else
			{
				likiestModel_index = -1;
			}
		}//END: if(cur_len == 1) else
		LinearModelSet[likiestModel_index].getEageByGis(line_1_index,line_2_index,gisFrame,_ms,cur_ins);
		return likiestModel_index;
	}

	void updateTo(C_InsFrame dst_Ins)
	{
		for (int i=0;i<cur_len;i++)
		{
			//if (LinearModelSet[i].with_data == true)
			{
				LinearModelSet[i].updateTo(dst_Ins);
			}
		}
	}

	void printModelSetStatus()
	{
		std::cout<<"#"<<"\t"<<"Age\t"<<"Stg\t"<<"Bel\t"<<"Width\t"<<std::endl;
		for(int i=0;i<cur_len;i++)
		{
			double lane_width = LinearModelSet[i].db/sqrt(LinearModelSet[i].k*LinearModelSet[i].k+1.0);
			lane_width = fabs(lane_width);
			std::cout<<i<<"\t"<<LinearModelSet[i].age<<"\t"<<LinearModelSet[i].strength<<"\t"<<LinearModelSet[i].beliefe<<"\t"<<lane_width<<"\t"<<std::endl;
		}
	}

	void printLikeliestModel()
	{
		fprintf(stdout,"Likeliest:%2d  \tborder\tline\tline\tborder\n",likiestModel_index);
		fprintf(stdout,"           \t%2d\t%2d\t%2d\t%2d\n",border_1_index,line_1_index,line_2_index,border_2_index);

		std::cout<<"Bor: ";
		for(int i=0;i<20;i++)
		{
			if(i == border_1_index)
			{
				fprintf(stdout,"  B1  ");
				continue;
			}
			if(i == border_2_index)
			{
				fprintf(stdout,"  B2  ");
				continue;
			}
			fprintf(stdout,"      ");
		}
		std::cout<<std::endl;
		std::cout<<"Lin: ";
		for(int i=0;i<20;i++)
		{
			if(i == line_1_index)
			{
				fprintf(stdout,"  L1  ");
				continue;
			}
			if(i == line_2_index)
			{
				fprintf(stdout,"  L2  ");
				continue;
			}
			fprintf(stdout,"      ");
		}
		std::cout<<std::endl;

		if(likiestModel_index!=-1)
			LinearModelSet[likiestModel_index].printLineSetStatus();
	}

	void DrawLikeliestModel(Mat Image,C_InsFrame dst_InsFrame, int LaneNum = 100)
	{
		if(likiestModel_index != -1)
		{
			LinearModelSet[likiestModel_index].updateTo(dst_InsFrame);
			if(border_1_index <=border_2_index)
			{
#if VERSION_SUB_URBAN
				if( LaneNum > 0 && border_2_index-border_1_index == LaneNum)
#endif
#if VERSION_URBAN
//				if( LaneNum > 0 && (border_2_index-border_1_index == LaneNum || 2*(border_2_index-border_1_index) == LaneNum))
				if( LaneNum > 0 && border_2_index-border_1_index == LaneNum)
#endif
				{
					LinearModelSet[likiestModel_index].Draw(Image,border_1_index,border_1_index,Scalar(70,150,250),false,30);
					LinearModelSet[likiestModel_index].Draw(Image,border_2_index,border_2_index,Scalar(70,150,250),false,30);
				}
				else
				{
					LinearModelSet[likiestModel_index].Draw(Image,border_1_index,border_1_index,Scalar(0,50,100),false,20);
					LinearModelSet[likiestModel_index].Draw(Image,border_2_index,border_2_index,Scalar(0,50,100),false,20);
				}
			}
			if(line_1_index <= line_2_index)
			{
				LinearModelSet[likiestModel_index].Draw(Image,line_1_index,line_2_index);
			}
		}
	}

	void Draw(Mat &Image)
	{
		for (int i=0;i<cur_len;i++)
		{
			//fprintf(stderr,"  #ModelSet:%d,k%lf,b%lf,db%lf\n",i,LinearModelSet[i].k,LinearModelSet[i].b,LinearModelSet[i].db);
			LinearModelSet[i].Draw(Image);
		}
	}

	bool getMid(C_LinearModel &mid,C_StoplineModel &midVert,Point &Cross_Point,C_InsFrame dst_Ins)
	{
		if(likiestModel_index != -1)
		{
			if(LinearModelSet[likiestModel_index].getMid(mid,line_1_index,line_2_index,dst_Ins) == true)
			{
				double a = 1;
				double b = -mid.k;
				double c = -mid.b -10.0*mid.db;
				double f_x,f_y;
				getPerpendicularFoot(a,b,c,1500.0,1000.0,f_x,f_y);
				Cross_Point.x = f_x;
				Cross_Point.y = f_y;
				midVert.setWith(1500.0,1000.0,-mid.k);
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}

	bool getMid(Point GisRawPoint,C_LinearModel &mid,C_InsFrame dst_Ins)
	{
		if(likiestModel_index != -1)
		{
			if(LinearModelSet[likiestModel_index].getMid(GisRawPoint,mid,dst_Ins) == true)
			{
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}
	bool getMid(C_LinearModel &mid,C_InsFrame dst_Ins,int line_1_index,int line_2_index)
	{
		if(likiestModel_index != -1)
		{
			if(LinearModelSet[likiestModel_index].getMid(mid,dst_Ins,line_1_index,line_2_index) == true)
			{
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}

	void disGisOrEdgeOnImage(Mat &img, Point pos = Point(50,350), Scalar color = Scalar(0,0,255), int fontFace = FONT_HERSHEY_SIMPLEX,
			double fontScale = 2, int thickness = 4)
	{
		char title_data[50];
		if(GisOrEdge == 0)
		{
			sprintf(title_data, "GIS or Edge: GIS");
		}
		else if(GisOrEdge == 1)
		{
			sprintf(title_data, "GIS or Edge: Edge");
		}
		putText(img, title_data, pos, fontFace, fontScale, color, thickness);
	}

};


#endif /* C_LINEAR_H_ */
