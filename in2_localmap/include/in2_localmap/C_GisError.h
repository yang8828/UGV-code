/*
 * C_GisError.h
 *
 *  Created on: 2013-10-14
 *      Author: greensky
 */

#ifndef C_GISERROR_H_
#define C_GISERROR_H_

#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include "in2_localmap/C_Ins.h"
#include "in2_localmap/C_BasicOp.h"

#define GISERROR_FRAME_HIS_LEN 50
#define GISERROR_SET_LEN 3
#define GISERRORSET_NEWSET_THRESHOLD 45 //in degree

using namespace cv;

class C_GisErrorFrame
{
public:
	long timestamp;
	double gis_dir_azimuth;
	double lane_dir_azimuth;
	double dist_to_intersection;
	double error_east;  // Error Measured
	double error_north;
	double error_dir_azimuth;
	C_GisErrorFrame()
	{
		timestamp = 0.0;
	}
	C_GisErrorFrame(long _timestamp,double _gis_dir_azimuth,double _lane_dir_azimuth,double _dist_to_intersection,double _error_east,double _error_north,double _error_dir_azimuth)
	{
		timestamp = _timestamp;
		gis_dir_azimuth = _gis_dir_azimuth;
		lane_dir_azimuth = _lane_dir_azimuth;
		dist_to_intersection = _dist_to_intersection;
		error_east = _error_east;
		error_north  = _error_north;
		error_dir_azimuth = _error_dir_azimuth;
	}
	C_GisErrorFrame(long _timestamp,double _error_east,double _error_north,double _error_dir_azimuth)
	{
		timestamp = _timestamp;
		gis_dir_azimuth = 0.0;
		lane_dir_azimuth = 0.0;
		dist_to_intersection = 0.0;
		error_east = _error_east;
		error_north  = _error_north;
		error_dir_azimuth = _error_dir_azimuth;
	}
	void copyfrom(C_GisErrorFrame src)
	{
		timestamp = src.timestamp;
		gis_dir_azimuth = src.gis_dir_azimuth;
		lane_dir_azimuth = src.lane_dir_azimuth;
		dist_to_intersection = src.dist_to_intersection;
		error_east = src.error_east;
		error_north = src.error_north;
		error_dir_azimuth = src.error_dir_azimuth;
	}
	//bool with_ins;
	//C_InsFrame ins;
};

class C_GisErrorFrameHis
{
public:
	C_GisErrorFrame List[GISERROR_FRAME_HIS_LEN];
	bool HighWeightList[GISERROR_FRAME_HIS_LEN];
	int cur_len;
	double gis_dir_azimuth;
	double gis_dir_azimuth_variance;
	double lane_dir_azimuth;
	double lane_dir_azimuth_variance;
	double error_dir_azimuth;  // Error Measured
	double error_dir_azimuth_variance;
	long last_timestamp;
	int ID;
	C_GisErrorFrameHis()
	{
		cur_len = 0;
	}
	void init(int id)
	{
		cur_len = 0;
		ID = id;
	}
	void addGisErrorFrame(C_GisErrorFrame newframe,bool isHighWeight = false)
	{
		for(int i=GISERROR_FRAME_HIS_LEN-1;i>0;i--)
		{
			List[i].copyfrom(List[i-1]);
			HighWeightList[i] = HighWeightList[i-1];
		}
		List[0].copyfrom(newframe);
		HighWeightList[0] = isHighWeight;
		last_timestamp = newframe.timestamp;
		cur_len ++;
		if (cur_len > GISERROR_FRAME_HIS_LEN)
			cur_len = GISERROR_FRAME_HIS_LEN;
		refresh();
	}
	void refresh()
	{
		if(cur_len > 0)
		{
			gis_dir_azimuth 			= 0.0;
			lane_dir_azimuth 			= 0.0;
			//error_dir_azimuth 			= 0.0;
			for(int i=0;i<cur_len;i++)
			{
				gis_dir_azimuth += List[i].gis_dir_azimuth;
				lane_dir_azimuth += List[i].lane_dir_azimuth;
				//error_dir_azimuth += List[i].error_dir_azimuth;
			}
			gis_dir_azimuth 			/= ((double)cur_len);
			lane_dir_azimuth 			/= ((double)cur_len);
			//error_dir_azimuth 			/= ((double)cur_len);

			gis_dir_azimuth_variance 	= 0.0;
			lane_dir_azimuth_variance 	= 0.0;
			//error_dir_azimuth_variance 	= 0.0;
			for(int i=0;i<cur_len;i++)
			{
				gis_dir_azimuth_variance 	+= (List[i].gis_dir_azimuth	 - gis_dir_azimuth)	 * (List[i].gis_dir_azimuth	 - gis_dir_azimuth);
				lane_dir_azimuth_variance 	+= (List[i].lane_dir_azimuth - lane_dir_azimuth) * (List[i].lane_dir_azimuth - lane_dir_azimuth);
			//error_dir_azimuth_variance 	+= (List[i].error_dir_azimuth- error_dir_azimuth)* (List[i].error_dir_azimuth- error_dir_azimuth);
			}
			gis_dir_azimuth_variance 			/= ((double)cur_len);
			lane_dir_azimuth_variance 			/= ((double)cur_len);
			//error_dir_azimuth_variance 			/= ((double)cur_len);
			error_dir_azimuth = List[cur_len-1].error_dir_azimuth;
		}
	}
	void copyfrom(C_GisErrorFrameHis src)
	{
		for(int i=0;i<GISERROR_FRAME_HIS_LEN;i++)
		{
			List[i].copyfrom(src.List[i]);
			HighWeightList[i] = src.HighWeightList[i];
		}
		cur_len 					= src.cur_len;
		gis_dir_azimuth 			= src.gis_dir_azimuth;
		gis_dir_azimuth_variance 	= src.gis_dir_azimuth_variance;
		lane_dir_azimuth 			= src.lane_dir_azimuth;
		lane_dir_azimuth_variance 	= src.lane_dir_azimuth_variance;
		error_dir_azimuth 			= src.error_dir_azimuth;
		error_dir_azimuth_variance 	= src.error_dir_azimuth_variance;
		last_timestamp 				= src.last_timestamp;
		ID 							= src.ID;
	}
	Mat getXY()
	{
		Mat XY(cur_len, 3,CV_64F);
		for(int i=0;i<cur_len;i++)
		{
			XY.at<double>(i,0) = List[i].error_east;
			XY.at<double>(i,1) = List[i].error_north;
			XY.at<double>(i,2) = List[i].error_east*List[i].error_east + List[i].error_north*List[i].error_north;
		}
		return XY;
	}
	Mat getXY_smoothed()
	{
		Mat XY(cur_len, 3,CV_64F);
		double norm[GISERROR_FRAME_HIS_LEN];
		for(int i=0;i<cur_len;i++)
		{
			norm[i] = List[i].error_east*List[i].error_east + List[i].error_north*List[i].error_north;
		}
		if(cur_len >=3)
		{
			XY.at<double>(0,0) = List[0].error_east;
			XY.at<double>(0,1) = List[0].error_north;
			XY.at<double>(0,2) = List[0].error_east*List[0].error_east + List[0].error_north*List[0].error_north;
			for(int j=1;j<cur_len-1;j++)
			{
				int i;
				if((norm[j-1]<=norm[j] && norm[j-1]>=norm[j+1]) || (norm[j-1]>=norm[j] && norm[j-1]<=norm[j+1]))
					i = j-1;
				if((norm[j]<=norm[j-1] && norm[j]>=norm[j+1]) || (norm[j]>=norm[j-1] && norm[j]<=norm[j+1]))
					i = j;
				if((norm[j+1]<=norm[j] && norm[j+1]>=norm[j-1]) || (norm[j+1]>=norm[j] && norm[j+1]<=norm[j-1]))
					i = j+1;
				XY.at<double>(j,0) = List[i].error_east;
				XY.at<double>(j,1) = List[i].error_north;
				XY.at<double>(j,2) = List[i].error_east*List[i].error_east + List[i].error_north*List[i].error_north;
			}
			XY.at<double>(cur_len-1,0) = List[cur_len-1].error_east;
			XY.at<double>(cur_len-1,1) = List[cur_len-1].error_north;
			XY.at<double>(cur_len-1,2) = List[cur_len-1].error_east*List[cur_len-1].error_east + List[cur_len-1].error_north*List[cur_len-1].error_north;
		}
		else
		{
			for(int i=0;i<cur_len;i++)
			{
				XY.at<double>(i,0) = List[i].error_east;
				XY.at<double>(i,1) = List[i].error_north;
				XY.at<double>(i,2) = List[i].error_east*List[i].error_east + List[i].error_north*List[i].error_north;
			}
		}
		return XY;
	}
	Mat getWeight()
	{
		Mat W(cur_len, 1,CV_64F);
		double weight_sum = 0;
		for(int i=0;i<cur_len;i++)
		{
			if(HighWeightList[i] == true)
				W.at<double>(i,0) = GISERROR_ERRORVECTOR_WEIGHT_RADIO;
			else
				W.at<double>(i,0) = 1;
			weight_sum += W.at<double>(i,0);
		}
		//normlize:
		W = W/weight_sum;
		return W;
	}
};

class C_GisError
{
public:
	double Error_E_estimated;
	double Error_N_estimated;
	int estimate_good; //-1: no estimation yet, 1: estimated by 1 set, 2:estimated by 2 set;
	bool InitComplited;
	C_GisErrorFrameHis Set[GISERROR_SET_LEN];
	bool LastFrameExist;
	C_GisErrorFrame LastFrame;
	int cur_len;
	int new_id;
	C_GisError()
	{
		LastFrameExist = false;
		estimate_good = false;
		InitComplited = true;
		cur_len = 0;
		new_id = 0;
	}
	void Init_GisError()
	{
		LastFrameExist = false;
		estimate_good = false;
		InitComplited = true;
		cur_len = 0;
		new_id = 0;
	}
	void Clear_GisErrorSample()
	{
		LastFrameExist = false;
		estimate_good = false;
		cur_len = 0;
		new_id = 0;
	}
	void addGisErrorFrame(C_GisErrorFrame newframe,bool isHighWeight = false)
	{

#if PRINT_DEBUG_GISERROR
		if(isHighWeight == true)
			std::cout<<"\n\n##########################   addGisErrorFrame  High Weight   #############################\n";
		else
			std::cout<<"\n\n##########################   addGisErrorFrame  Low Weight   #############################\n";
		std::cout<<"                                 cur_len = "<<cur_len<<"\n";
		fprintf(stdout,"newframe: timestamp = %ld\n",newframe.timestamp);
		fprintf(stdout,"              error = (%lf,%lf)\n",newframe.error_east,newframe.error_north);
		fprintf(stdout,"            azimuth = %lf\n",newframe.error_dir_azimuth);

#endif
		LastFrame.copyfrom(newframe);
		LastFrameExist = true;
		if(cur_len == 0)
		{
			Set[0].init(new_id);
			Set[0].addGisErrorFrame(newframe,isHighWeight);
			new_id++;
			cur_len = 1;
#if PRINT_DEBUG_GISERROR
		std::cout<<"Add to New ID = "<<new_id-1<<"\n";
#endif
		}
		else
		{
			int minindex;
			double minvalue;
			bool first_time_flag = true;
			for(int i=0;i<cur_len;i++)
			{
				if(first_time_flag == true)
				{
					first_time_flag = false;
					minindex = i;
					minvalue = lineAngleDiff(Set[i].error_dir_azimuth,newframe.error_dir_azimuth);
				}
				else
				{
					double temp = lineAngleDiff(Set[i].error_dir_azimuth,newframe.error_dir_azimuth);
					if(temp < minvalue)
					{
						minvalue = temp;
						minindex = i;
					}
				}
			}//END:for(int i=0;i<GISERROR_Set_LEN;i++)
#if PRINT_DEBUG_GISERROR
		std::cout<<"minvalue "<<minvalue<<"\n";
#endif
			if(minvalue < GISERRORSET_NEWSET_THRESHOLD) // add to exist Set
			{
				Set[minindex].addGisErrorFrame(newframe,isHighWeight);
#if PRINT_DEBUG_GISERROR
		std::cout<<"add to exist Set "<<minindex<<"\n";
		fprintf(stdout,"            Set[%d].cur_len = %d\n",minindex,Set[minindex].cur_len);
#endif
			}
			else //add to new Set new_id
			{
				if(cur_len < GISERROR_SET_LEN)  // enougth space
				{
					Set[cur_len].init(new_id);
					Set[cur_len].addGisErrorFrame(newframe,isHighWeight);
					new_id++;
					cur_len ++;
#if PRINT_DEBUG_GISERROR
		std::cout<<"add to New Set "<<cur_len-1<<" and ID "<<new_id-1<<"\n";
		fprintf(stdout,"            Set[%d].cur_len = %d\n",cur_len-1,Set[cur_len-1].cur_len);
#endif
				}
				else  //no enougth space
				{
					setFilter(); // delete the useless set

					if(cur_len < GISERROR_SET_LEN)
					{
						Set[cur_len].init(new_id);
						Set[cur_len].addGisErrorFrame(newframe,isHighWeight);
						new_id++;
						cur_len ++;
#if PRINT_DEBUG_GISERROR
		std::cout<<"filted\n";
		std::cout<<"add to new Set "<<cur_len-1<<" and ID "<<new_id-1<<"\n";
		fprintf(stdout,"            Set[%d].cur_len = %d\n",cur_len-1,Set[cur_len-1].cur_len);
#endif
					}
					else
					{
						fprintf(stderr,"ERROR: NO Space For New GISERRORSET");
					}
				}
			} //END: if(minvalue < GISERRORSET_NEWSET_THRESHOLD)... else...
		} //END: if(cur_len == 0)... else...
	}
	void deleteSet(int index)
	{
		if(cur_len == 0 || index >= cur_len || index < 0)
		{
			fprintf(stderr,"ERROR in  Delete Set");
			return;
		}
		if(index == cur_len)
		{
			cur_len --;
		}
		else
		{
			for(int i=index;i<cur_len-1;i++)
			{
				Set[i].copyfrom(Set[i+1]);
			}
			cur_len --;
		}
	}
	void setFilter()
	// delete the useless set
	{
		long minvalue;
		int minindex;
		bool first_time_flag = true;
		for(int i=0;i<cur_len;i++)
		{
			if(first_time_flag == true)
			{
				first_time_flag = false;
				minvalue = Set[i].last_timestamp;
				minindex = i;
			}
			else
			{
				if(minvalue > Set[i].last_timestamp)
				{
					minvalue = Set[i].last_timestamp;
					minindex = i;
				}
			}
		}
#if PRINT_DEBUG_GISERROR
		std::cout<<"deleting "<<minindex<<" with timestamp "<<minvalue<<"\n";
#endif
		deleteSet(minindex);
	}
	bool estimateError(double &EastError, double &NorthError)
	{
#if PRINT_DEBUG_GISERROR
		std::cout<<"(1/2) estimating...\n";
		for(int i=0;i<cur_len;i++)
		{
			fprintf(stdout,"Set[%d].cur_len = %d\n",i,Set[i].cur_len);
		}
		std::cout<<"(2/2) estimating Begin...\n";

#endif
		if(cur_len == 0)
		{
#if PRINT_DEBUG_GISERROR
		std::cout<<"estimate_good = -1\n";
		std::cout<<"##################################################################################\n\n\n";
#endif
			estimate_good = -1;
			return false;
		}
		else
		{
			if(cur_len == 1)
			{
				if(Set[0].cur_len == GISERROR_FRAME_HIS_LEN )
				{
					Mat XY = Set[0].getXY_smoothed();
					Mat W  = Set[0].getWeight();
					//Method 1: Least Square (if is not collinear
					//...
					//Method 2: Get Average  （if is vary colliear
					Mat average = W.t()*XY;
					EastError = average.at<double>(0,0);
					NorthError = average.at<double>(0,1);
					Error_E_estimated = EastError;
					Error_N_estimated = NorthError;
					estimate_good = 1;

#if PRINT_DEBUG_GISERROR
					std::cout<<"estimate_good = 1 with len == 1\n";
					std::cout<<"##################################################################################\n\n\n";
#endif
					return true;
				}
				else
				{
#if PRINT_DEBUG_GISERROR
					std::cout<<"estimate_good = -1 with len == 1 but NO Set[0].cur_len == GISERROR_FRAME_HIS_LEN\n";
					std::cout<<"##################################################################################\n\n\n";
#endif
					estimate_good = -1;
					return false;
				}
			}
			else
			{
				long latest_1;
				int index_1 = -1;
				bool first_time_flag = true;
				for(int i=0;i<cur_len;i++)
				{
					if(Set[i].cur_len == GISERROR_FRAME_HIS_LEN )
					{
						if(first_time_flag == true)
						{
							first_time_flag = false;
							latest_1 = Set[i].last_timestamp;
							index_1 = i;
						}
						else
						{
							if(Set[i].last_timestamp > latest_1)
							{
								latest_1 = Set[i].last_timestamp;
								index_1 = i;
							}
						}
					}
				}
				long latest_2;
				int index_2 = -1;
				first_time_flag = true;
				for(int i=0;i<cur_len;i++)
				{
					if(Set[i].cur_len == GISERROR_FRAME_HIS_LEN && i!=index_1)
					{
						if(first_time_flag == true)
						{
							first_time_flag = false;
							latest_2 = Set[i].last_timestamp;
							index_2 = i;
						}
						else
						{
							if(Set[i].last_timestamp > latest_2)
							{
								latest_2 = Set[i].last_timestamp;
								index_2 = i;
							}
						}
					}
				}
				if(index_1 == -1 && index_2 == -1)
				{
#if PRINT_DEBUG_GISERROR
		std::cout<<"estimate_good = -1 with len > 1 but NO Set.cur_len == GISERROR_FRAME_HIS_LEN\n";
		std::cout<<"##################################################################################\n\n\n";
#endif
					estimate_good = -1;
					return false;
				}
				else
				{
					if(index_1 >=0 && index_2 == -1)
					{
						Mat XY = Set[index_1].getXY_smoothed();
						Mat W  = Set[index_1].getWeight();
						Mat average = W.t()*XY;
						EastError = average.at<double>(0,0);
						NorthError = average.at<double>(0,1);
						Error_E_estimated = EastError;
						Error_N_estimated = NorthError;
						estimate_good = 1;
	#if PRINT_DEBUG_GISERROR
						std::cout<<"estimate_good = 1 with valid_len == 1\n";
						fprintf(stdout,"index_1 = %d\n",index_1);
						fprintf(stdout," %lf\n",Set[index_1].error_dir_azimuth);
						std::cout<<"##################################################################################\n\n\n";
	#endif
						return true;
					}
					else
					{
						Mat XY1 = Set[index_1].getXY_smoothed();
						Mat W1  = Set[index_1].getWeight();
						XY1 = repeat(W1,1,3).mul(XY1);
						Mat XY2 = Set[index_2].getXY_smoothed();
						Mat W2  = Set[index_2].getWeight();
						XY2 = repeat(W2,1,3).mul(XY2);
						//std::cout<<"XY1:\n";
						//std::cout<<XY1<<std::endl;
						//std::cout<<"XY2:\n";
						//std::cout<<XY2<<std::endl;
						Mat XY(XY1.rows+XY2.rows,3,CV_64F);
						XY1.copyTo(XY(Range(0,XY1.rows),Range(0,3)));
						XY2.copyTo(XY(Range(XY1.rows,XY1.rows+XY2.rows),Range(0,3)));
						//std::cout<<"XY:\n";
						//std::cout<<XY;

						Mat X = XY(Range(0,XY.rows),Range(0,2));
						Mat Y = XY(Range(0,XY.rows),Range(2,3));

						Mat XtX = X.t()*X;
						Mat Theta = XtX.inv()*X.t()*Y;
						EastError = Theta.at<double>(0,0);
						NorthError = Theta.at<double>(1,0);
						Error_E_estimated = EastError;
						Error_N_estimated = NorthError;
						estimate_good = 2;
	#if PRINT_DEBUG_GISERROR
						std::cout<<"estimate_good = 2 with valid_len > 1\n";
						fprintf(stdout,"index_1 %d,\tindex_2 %d\n",index_1,index_2);
						fprintf(stdout,"%lf,\t%lf\n",Set[index_1].error_dir_azimuth,Set[index_2].error_dir_azimuth);
						std::cout<<"##################################################################################\n\n\n";
	#endif
						return true;
					}
				}

			}//END: if(cur_len == 1)... else...
		}//END: if(cur_len == 0)... else...


	}
	void DrawErrorVactor(Mat image,C_InsFrame ins)
	{
		if(estimate_good > 0)
		{
			double azimuth = ins.attitude[2]*PI/180.0;
			int x = (int)(Error_E_estimated*cos(azimuth) - Error_N_estimated*sin(azimuth));
			int y = (int)(Error_E_estimated*sin(azimuth) + Error_N_estimated*cos(azimuth));
			int o_x = 1500;
			int o_y = 2000;
			if(estimate_good > 1)
				line(image,Point(o_x,o_y),Point(o_x+x,o_y-y),Scalar(100,222,15),15,2);
			else
			{
				line(image,Point(o_x,o_y),Point(o_x+x,o_y-y),Scalar(222,100,15),10,2);
			}
		}
		int fontFace = FONT_HERSHEY_SIMPLEX;
		double fontScale = 1.5;
		int thickness = 3;
		char title_data[30];
		if(InitComplited == 1)
		{
			sprintf(title_data,"Auto Correcting: ON");
			putText(image, title_data, Point(250,250), fontFace, fontScale, Scalar(0,0,0), 2*thickness);
			putText(image, title_data, Point(250,250), fontFace, fontScale, Scalar(0,180,0), thickness);
		}
		else
		{
			sprintf(title_data,"Auto Correcting: OFF");
			putText(image, title_data, Point(250,250), fontFace, fontScale, Scalar(0,0,0), 2*thickness);
			putText(image, title_data, Point(250,250), fontFace, fontScale, Scalar(0,0,180), thickness);
		}
	}
	void SetInitComplited()
	{
		if(InitComplited == true)
			InitComplited = false;
		else
			InitComplited = true;
	}
};


#endif /* C_GISERROR_H_ */
