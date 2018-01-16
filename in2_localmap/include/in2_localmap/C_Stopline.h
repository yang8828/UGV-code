/*
 * C_Stopline.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_STOPLINE_H_
#define C_STOPLINE_H_

#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>


#include <in2_localmap/C_Param.h>
#include <in2_localmap/C_Ins.h>
#include <in2_localmap/C_Line.h>
#include <in2_localmap/C_Gis.h>
using namespace cv;

class C_GisFrame;

class C_StoplineModel
{
	//y = k*x+b;
public:
	double k,b;
	int age;
	double strength;
	double strength_record;
	double beliefe;
	double beliefe_record;
	bool withIns;
	int time_since_last_sample;
	C_InsFrame InsData;
	C_LineSampleHis stopline;
	C_StoplineModel()
	{
		age = 0;
		beliefe = 0.0;
		beliefe_record = 0.0;
		strength = 0.0;
		strength_record = 0.0;
		stopline.init();
		time_since_last_sample = 0;
		withIns = false;
	}
	C_StoplineModel(C_LineSample newSample)
	{
		initWith(newSample);
	}
	void initWith(C_LineSample newSample)
	{
		Mat A = (Mat_<double>(2, 2)<<newSample.points_x[0],1,newSample.points_x[30],1);
		Mat Y = (Mat_<double>(2, 1)<<newSample.points_y[0],newSample.points_y[30]);
		Mat AtA = A.t()*A;
		Mat p = AtA.inv()*A.t()*Y;
		double temp_k = p.at<double>(0,0);
		double temp_b = p.at<double>(1,0);
		k = temp_k;
		b = temp_b;

		age = 0;
		beliefe = 0.0;
		beliefe_record = 0.0;
		strength = 0.0;
		strength_record = 0.0;
		stopline.init();
		time_since_last_sample = 0;
		if(newSample.withIns == true)
		{
			InsData.copyfrom(newSample.InsData);
			withIns = true;
		}
	}
	void setWith(double point_x,double point_y,double slope_k)
	{
		k = slope_k;
		b = -k*point_x+point_y;
	}
	void copyfrom(C_StoplineModel src)
	{
		k	= src.k;
		b	= src.b;
		age = src.age;
		strength = src.strength;
		strength_record = src.strength_record;
		beliefe = src.beliefe;
		beliefe_record = src.beliefe_record;
		stopline.copyfrom(src.stopline);
		time_since_last_sample = src.time_since_last_sample;
		withIns = src.withIns;
		InsData.copyfrom(src.InsData);
	}
	void updateTo(C_InsFrame dst_Ins)
	{
		Mat XY = stopline.getXYat(dst_Ins);
		if(XY.rows == 0)
		{
			double x1 = 0.0;
			double y1 = k*x1 + b;
			double x2 = 3000.0;
			double y2 = k*x2 + b;
			double dst_x1;
			double dst_y1;
			double dst_x2;
			double dst_y2;
			doRT2(x1,y1,dst_x1,dst_y1,InsData.position[0],InsData.position[1],dst_Ins.position[0],dst_Ins.position[1],InsData.attitude[2],dst_Ins.attitude[2]);
			doRT2(x2,y2,dst_x2,dst_y2,InsData.position[0],InsData.position[1],dst_Ins.position[0],dst_Ins.position[1],InsData.attitude[2],dst_Ins.attitude[2]);

			Mat A = (Mat_<double>(2,2)<<dst_x1,1,dst_x2,1);
			Mat Y = (Mat_<double>(2,1)<<dst_y1,dst_y2);
			Mat AtA = A.t()*A;
			Mat p = AtA.inv()*A.t()*Y;
			double temp_k = p.at<double>(0,0);
			double temp_b = p.at<double>(1,0);
			k = temp_k;
			b = temp_b;
			InsData.copyfrom(dst_Ins);
			withIns = true;
			return;
		}
		else
		{
			Mat A = Mat::ones(XY.rows,2,CV_64F);
			XY(Range(0,XY.rows),Range(0,1)).copyTo(A(Range(0,XY.rows),Range(0,1)));
			Mat Y = XY(Range(0,XY.rows),Range(1,2));
			Mat AtA = A.t()*A;
			Mat p = AtA.inv()*A.t()*Y;
			double temp_k = p.at<double>(0,0);
			double temp_b = p.at<double>(1,0);
			k = temp_k;
			b = temp_b;
			InsData.copyfrom(dst_Ins);
			withIns = true;
		}
	}
	double distanceToModel(int x,int y)
	{	//y = k*x+b;
		double double_x = (double) x;
		double double_y = (double) y;
		double distance = fabs(double_y-k*double_x-b)/sqrt(1+k*k);
		return distance;
	}
	double getFittest(C_LineSample sample)
	{
		double fittness;
		fittness = 0.0;
		fittness += distanceToModel(sample.points_x[0],sample.points_y[0]);
		fittness += distanceToModel(sample.points_x[30],sample.points_y[30]);
		fittness /=2.0;
		return fittness;
	}
	void addLineSampe(C_LineSample newframe)
	{
		stopline.addNewFrame(newframe);
		if(stopline.line_his[MAX_LINE_HIS_LEN-1].type!=0)
		{
			strength --;
			beliefe -= stopline.line_his[MAX_LINE_HIS_LEN-1].beliefe;
		}
		if(newframe.type !=0)
		{
			time_since_last_sample = 0;
			strength ++;
			beliefe += newframe.beliefe;
		}
		if(strength_record < strength)
		{
			strength_record = strength;
		}
		if(beliefe_record < beliefe)
		{
			beliefe_record = beliefe;
		}
	}
	void addAge()
	{
		time_since_last_sample++;
		age++;
		if(age > MAX_LINE_HIS_LEN) age = MAX_LINE_HIS_LEN;
	}
	void printStoplineStatus(bool print_title = true,bool print_data = true)
	{
		if(print_title == true)
		{
			std::cout<<"StoplineStatus:\n";
			fprintf(stdout,"Age \tStg \tStgRc \tBlf \tBlfRc \tT_SLS \tK \tb \n");
		}
		if(print_data == true)
		{
			fprintf(stdout,"%d \t%4.1lf \t%4.1lf \t%4.1lf \t%4.1lf \t%4d \t%4.1lf \t%4.1lf \n",
					age,strength,strength_record,beliefe,beliefe_record,time_since_last_sample,k,b);
		}
	}
	void Draw(Mat &Image,Scalar color = StoplineSetColor)
	{
		//y = k*x+b;
		int x_1 = 0;
		int y_1 = k*x_1 + b;
		int x_2 = 3000;
		int y_2 = k*x_2 + b;
		line(Image,Point(x_1,3000-y_1),Point(x_2,3000-y_2),color,15);
	}
	void getPerpenditualFoot(double src_x,double src_y,double &foot_x,double &foot_y)
	{
		//y - k*x - b = 0;
		double A = -k;
		double B = 1;
		double C = -b;
		getPerpendicularFoot(A,B,C,src_x,src_y,foot_x,foot_y);
	}
	double getNormAngle()
	//int 3000 coordinate system
	{
		return atan(-k)*180.0/PI;
	}
};

class C_StoplineModelSet
{
	C_LineSample associatedSample[MAX_STOPLINE_MODEL_NUM];
public:
	C_StoplineModel StopLineModelSet[MAX_STOPLINE_MODEL_NUM];
	int cur_len;
public:
	int likiestModel_index;
public:
	C_StoplineModelSet()
	{
		cur_len = 0;
		likiestModel_index = -1;
	}
	void init_associatedSample()
	{
		for(int i=0;i<MAX_STOPLINE_MODEL_NUM;i++)
		{
			associatedSample[i].init();
		}
	}
	void updateTo(C_InsFrame dst_Ins)
	{
		for (int i=0;i<cur_len;i++)
		{
			//if (LinearModelSet[i].with_data == true)
			{
				StopLineModelSet[i].updateTo(dst_Ins);
			}
		}
	}
	void addSample(C_LineSample *newLineSamples,int len)
	{
		int new_len = cur_len;
		init_associatedSample();
#if PRINT_DEBUG_LANE
		std::cout << "\n### LineSample Len:" << len << "\tStoplineModelSet Len:" << cur_len << " ###\n";
#endif
		if(len>0)
		{
#if PRINT_DEBUG_LANE
			std::cout << "updateTo ins"<<std::endl;
#endif
			if(newLineSamples[0].withIns == true)
				updateTo(newLineSamples[0].InsData);
#if PRINT_DEBUG_LANE
			std::cout << "while(1)"<<std::endl;
#endif
			bool not_enougth_linear_model_space = false;
			while (1)
			{
				for (int i = 0; i < new_len; i++) //loop for all existed model
				{
					for (int j = 0; j < len; j++) //loop for all newSample
					{
						if (newLineSamples[j].valid == 0)
							continue;
						double fittness = StopLineModelSet[i].getFittest(newLineSamples[j]);
#if PRINT_DEBUG_LANE
						fprintf(stdout,"line %02d with Old Set %02d Fit %06.1f\n",j,i,fittness);
#endif
						//std::cout << "line " << j << " with Old Set " << i << " Fit " << fittness << "\n";
						if (fittness < FITNESS_THRESHOLD)
						{
							associatedSample[i].copyfrom(newLineSamples[j]);
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
						StopLineModelSet[new_len].initWith(newLineSamples[j]);
#if PRINT_DEBUG_LANE
						fprintf(stdout,"line %02d with New Set %02d Fit %06.1f\n",j,new_len,0.0);
#endif
						//std::cout << "line " << j << " with New Set " << new_len << " " << 10 << " Fit " << 0 << "\n";
						associatedSample[new_len].copyfrom(newLineSamples[j]);
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

#if PRINT_DEBUG_LANE
		std::cout << "addLineSampe"<<std::endl;
#endif
		//fprintf(stderr,"##BreakPoint 01##\n");
		for (int i = 0; i < new_len; i++)
		{
			StopLineModelSet[i].addLineSampe( associatedSample[i]);
			//LinearModelSet[i].line_set[j].addNewFrame(associatedSample[i][j]);
		}
		cur_len = new_len;
		//fprintf(stderr,"cur_len: %d\n",cur_len);

	}
	void addAge(void)
	{
		for(int i=0;i<cur_len;i++)
		{
			StopLineModelSet[i].addAge();
		}
	}
	void DeleteSet(int ModelIndex)
	{
		if(cur_len == 0 || ModelIndex >= cur_len || ModelIndex<0)
		{
			fprintf(stderr,"ERROR in  Delete Set");
			return;
		}
		if(ModelIndex == cur_len)
		{
			cur_len --;
		}
		else
		{
			for(int i=ModelIndex;i<cur_len-1;i++)
			{
				StopLineModelSet[i].copyfrom(StopLineModelSet[i+1]);
			}
			cur_len --;
		}
	}
	void SetFilter_Type2()
	{
#if PRINT_DEBUG_LANE
		std::cout<<"Set Filting Tpye2... ";
#endif
		for(int i=0;i<cur_len;i++)
		{
			if(StopLineModelSet[i].strength < 0.1)
			{
				if(StopLineModelSet[i].time_since_last_sample > 75)
				{
					DeleteSet(i);
#if PRINT_DEBUG_LANE
					std::cout<<i<<"T ";
#endif
				}
			}
		}
		if(cur_len > MAX_STOPLINE_MODEL_NUM-1 )
		{
			int min_index = -1;
			double min_val;
			bool first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				if(StopLineModelSet[i].age == MAX_LINE_HIS_LEN )
				{
					if(first_time_flag == true)
					{
						first_time_flag = false;
						min_index = i;
						min_val = StopLineModelSet[i].beliefe;
					}
					else
					{
						if(StopLineModelSet[i].beliefe<min_val)
						{
							min_index = i;
							min_val = StopLineModelSet[i].beliefe;
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
	void SetFilter()
	{
#if PRINT_DEBUG_LANE
		std::cout<<"Set Filting... ";
#endif
		for(int i=0;i<cur_len;i++)
		{
			if(StopLineModelSet[i].strength < 0.1)
			{
				DeleteSet(i);
#if PRINT_DEBUG_LANE
				std::cout<<i<<"S ";
#endif
			}
			if(StopLineModelSet[i].time_since_last_sample > 150)
			{
				DeleteSet(i);
#if PRINT_DEBUG_LANE
				std::cout<<i<<"T ";
#endif
			}
		}
		if(cur_len > MAX_STOPLINE_MODEL_NUM-1 )
		{
			int min_index = -1;
			double min_val;
			bool first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				if(StopLineModelSet[i].age == MAX_LINE_HIS_LEN )
				{
					if(first_time_flag == true)
					{
						first_time_flag = false;
						min_index = i;
						min_val = StopLineModelSet[i].beliefe;
					}
					else
					{
						if(StopLineModelSet[i].beliefe<min_val)
						{
							min_index = i;
							min_val = StopLineModelSet[i].beliefe;
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
	int  getLikeliestModelByGis(C_GisFrame *gisFrame);
	int  GetLikeliestModel_Type2(void)
	{
		if(cur_len == 0)
		{
			likiestModel_index = -1;
		}
		if(cur_len == 1)
		{

			if(StopLineModelSet[0].beliefe_record> LIKELIEST_STOPLINE_MODEL_THREASHOLD_1 && StopLineModelSet[0].strength > 0)
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
			bool first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				if(StopLineModelSet[i].time_since_last_sample < LIKELIEST_STOPLINE_MODEL_THREASHOLD_2)
				{
					if(first_time_flag==true)
					{
						first_time_flag = false;
						likeliest_index = i;
						likeliest_value = StopLineModelSet[i].beliefe_record;
					}
					else
					{
						double temp_value ;
						temp_value = StopLineModelSet[i].beliefe_record;
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
				if(StopLineModelSet[likiestModel_index].strength < 0.1)
				{
					int temp_index;
					double temp_value;
					bool temp_first = true;
					for(int i=0;i<cur_len;i++)
					{
						if(StopLineModelSet[i].strength >= 0.1)
						{
							if(temp_first == true)
							{
								temp_first = false;
								temp_index = i;
								temp_value = StopLineModelSet[i].beliefe;
							}
							else
							{
								if(temp_value<StopLineModelSet[i].strength)
								{
									temp_index = i;
									temp_value = StopLineModelSet[i].beliefe;
								}
							}
						}
					}
					if(temp_first == false)
					{
						likiestModel_index = temp_index;
					}
				}
			}
			else
			{
				likiestModel_index = -1;
			}
		}//END: if(cur_len == 1) else
		return likiestModel_index;
	}
	int  GetLikeliestModel(const int32_t _ms)
	{
		if(cur_len == 0)
		{
			likiestModel_index = -1;
		}
		if(cur_len == 1)
		{

			if(StopLineModelSet[0].beliefe/StopLineModelSet[0].strength > LIKELIEST_STOPLINE_MODEL_THREASHOLD_3 && StopLineModelSet[0].strength > 0)
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
				if(StopLineModelSet[i].strength > 0)
				{
					if(first_time_flag==true)
					{
						first_time_flag = false;
						likeliest_index = i;
						likeliest_value = StopLineModelSet[i].beliefe/StopLineModelSet[i].strength + StopLineModelSet[i].strength*8;
						//if(LinearModelSet[i].beliefe/LinearModelSet[i].strength > LinearModelSet[i].strength)
						//	likeliest_value = LinearModelSet[i].beliefe/LinearModelSet[i].strength;
						//else
						//	likeliest_value = LinearModelSet[i].strength;
					}
					else
					{
						double temp_value ;
						temp_value = StopLineModelSet[i].beliefe/StopLineModelSet[i].strength + StopLineModelSet[i].strength*8;
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
		return likiestModel_index;
	}
	void printModelSetStatus()
	{
		StopLineModelSet[0].printStoplineStatus(true,false);
		for(int i=0;i<cur_len;i++)
		{
			StopLineModelSet[i].printStoplineStatus(false,true);
		}
	}
	void printLikeliestModel()
	{
		std::cout<<"Likeliest: "<<likiestModel_index;
		if(likiestModel_index!=-1)
			StopLineModelSet[likiestModel_index].printStoplineStatus();
	}
	void DrawLikeliestModel(Mat Image,C_InsFrame dst_InsFrame)
	{
		if(likiestModel_index != -1)
		{
			StopLineModelSet[likiestModel_index].updateTo(dst_InsFrame);
			StopLineModelSet[likiestModel_index].Draw(Image);
		}
	}

	void Draw(Mat &Image)
	{
		for (int i=0;i<cur_len;i++)
		{
			//fprintf(stderr,"  #ModelSet:%d,k%lf,b%lf,db%lf\n",i,LinearModelSet[i].k,LinearModelSet[i].b,LinearModelSet[i].db);
			StopLineModelSet[i].Draw(Image);
		}
	}
};

#endif /* C_STOPLINE_H_ */
