/*
 * C_Stopline.cpp
 *
 *  Created on: 2013-10-12
 *      Author: greensky
 */
#include "in2_localmap/C_Stopline.h"

int C_StoplineModelSet::getLikeliestModelByGis(C_GisFrame *gisFrame)
{
	if(cur_len == 0)
	{
		likiestModel_index = -1;
	}
	if(cur_len == 1)
	{
		double angle_Error = byangle3KDiff(StopLineModelSet[0].getNormAngle() , gisFrame->angle);
		if(angle_Error < LIKELIEST_STOPLINE_MODEL_THREASHOLD_Angel)
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
				double angle_Error = byangle3KDiff(StopLineModelSet[0].getNormAngle(),gisFrame->angle);
				if(angle_Error < LIKELIEST_STOPLINE_MODEL_THREASHOLD_Angel)
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
					double angle_Error = byangle3KDiff(StopLineModelSet[0].getNormAngle() , gisFrame->angle);
					if(angle_Error < LIKELIEST_STOPLINE_MODEL_THREASHOLD_Angel)
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


