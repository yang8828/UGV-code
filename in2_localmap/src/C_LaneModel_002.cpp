/*
 * C_LaneModel_002.cpp
 *
 *  Created on: 2013-10-12
 *      Author: greensky
 */

#include "in2_localmap/C_LaneModel_002.h"
#include <cmath>
	C_LocalMap::C_LocalMap(ros::NodeHandle node, ros::NodeHandle priv_nh)
	{
		temp_counter = 0;
		LocalMapCnter_10ms = 0;
		LocalMapCnter_10min = 0;
		LaneCnter_10ms = 0;
		LaneCnter_10min = 0;

		haveLastGisrecMsg = false;

#if USING_SCANINFOV2
		scaninfo_sub = node.subscribe("ScanInfoV2", 10,
		                          	  	  &C_LocalMap::transformTopic_ScanInfo, this,
		                          	  	  ros::TransportHints().tcpNoDelay(true));
#endif

#if USING_SCANINFOBG
		scaninfo_sub = node.subscribe("ScanInfoBackGround", 10,
		                          	  	  &C_LocalMap::transformTopic_ScanInfoBG, this,
		                          	  	  ros::TransportHints().tcpNoDelay(true));
#endif

		roadedge_sub = node.subscribe("RoadEdge", 10,
				                          	  	  &C_LocalMap::transformTopic_RoadEdge, this,
				                          	  	  ros::TransportHints().tcpNoDelay(true));

		laneinfo_sub = node.subscribe("LaneInfoV2", 10,
		                          	  	  &C_LocalMap::transformTopic_LaneInfo, this,
		                          	  	  ros::TransportHints().tcpNoDelay(true));
		ins_sub = node.subscribe("ins", 10,
											&C_LocalMap::transformTopic_InsInfo, this,
											ros::TransportHints().tcpNoDelay(true));
		gis_sub = node.subscribe("GISudp", 10,
				                          &C_LocalMap::transformTopic_GisInfo, this,
				                          ros::TransportHints().tcpNoDelay(true));
		cp_sub = node.subscribe("ControlPanelInfo", 10,
                							&C_LocalMap::transformTopic_ControlPanelInfo, this,
                							ros::TransportHints().tcpNoDelay(true));
		udp_publisher_ = node.advertise<in2_msgs::UdpGeneral>("GISRect", 10);
		udp_publisher_laneinfo_ = node.advertise<in2_msgs::LaneInfoV2>("LaneModelInfo", 10);
		//gis_vector_pub = node.advertise<in2_msgs::GisVector>("GisVectorInfo", 10);
	}

	void C_LocalMap::transformTopic_ControlPanelInfo(const in2_msgs::UdpGeneralShort::ConstPtr &cp_sub)
	{
		switch(cp_sub->data[2])
		{
		case 'w':
			GisFrame.NorthInc();
			break;
		case 's':
			GisFrame.NorthDec();
			break;
		case 'd':
			GisFrame.EastInc();
			break;
		case 'a':
			GisFrame.EastDec();
			break;
		case 'p':
			GisError.SetInitComplited();
			break;
		case 'i':
			GisError.Clear_GisErrorSample();
			GisFrame.setInitError(0.0,0.0);
			break;
		case 'k':
			SetGisErrorToHere();
			break;
		}
	}

	void C_LocalMap::transformTopic_GisInfo(const in2_msgs::UdpGeneral::ConstPtr &gisinfo_sub)
	{
		LastGisInfo_TS = GisFrame.SetWithSample(gisinfo_sub->data,gisinfo_sub->x,gisinfo_sub->y,InsFrameHis);
//-----------------------------------------------------------------------------------------------
		if(GisFrame.isInit)
		{
			GisError.Clear_GisErrorSample();
			GisFrame.setInitError(0.0,0.0);
			GisFrame.isInit = false;
			GisFrame.DelayDeadCnter = DELAY_DEAD_NUM;
			GisFrame.InitCnter++;
		}
		if(GisFrame.isSendHigh)
		{
			GisFrame.SendLvl = SEND_HIGH_LVL;
			GisFrame.isSendHigh = false;
			GisFrame.SendHighCnter++;
		}
		if(GisFrame.isSendLow)
		{
			GisFrame.SendLvl = SEND_LOW_LVL;
			GisFrame.isSendLow = false;
			GisFrame.SendLowCnter++;
		}
		//GisFrame.initGisErrorByLevel(GisError);
//-----------------------------------------------------------------------------------------------
		//CorrectGis();
		//EstimateGisError();
		C_InsFrame cur_ins;
		if(!InsFrameHis.GetLastInsFrame(cur_ins))
		{
			return;
		}
		CorrectGisError_4(LastGisInfo_TS,cur_ins);
		//GisFrame.printError();
		C_InsFrame dst_ins;
		InsFrameHis.GetLastInsFrame(dst_ins);
		C_GisFrame now_gis;
		GisFrame.RTto(now_gis,dst_ins);
//		gisrec_msg.data[0] = LastGisInfo_TS;
//		for(int i=0;i<301;i++)
//		{
//			gisrec_msg.x[i] = (long)now_gis.AlignedData().x[i];
//			gisrec_msg.y[i] = (long)now_gis.AlignedData().y[i];
//		}
//		udp_publisher_.publish(gisrec_msg);

//---------------------------------------------Dead History Start--------------------------------------------------
		if(GisFrame.isDeadHis || GisFrame.DelayDeadCnter>0)
//		if(GisFrame.isDeadHis)
		{
			if(GisFrame.DelayDeadCnter>0)
				GisFrame.DelayDeadCnter--;
			if(GisFrame.haveLastGis)
			{
				GisFrame.DeadHisCnter++;

				CvPoint tmp_gis[301];
				C_InsFrame cur_ins;
				if(!InsFrameHis.GetLastInsFrame(cur_ins))
				{
					return;
				}
				for(int i=0; i<301; i++)
				{
					tmp_gis[i] = convENUto3000(last_gis_earth[i].x, last_gis_earth[i].y, cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
				}
				gisrec_msg.data[0] = LastGisInfo_TS;
				for(int i=0;i<301;i++)
				{
					gisrec_msg.x[i] = (long)tmp_gis[i].x;
					gisrec_msg.y[i] = (long)tmp_gis[i].y;
				}
				udp_publisher_.publish(gisrec_msg);
			}
			else
			{
				gisrec_msg.data[0] = LastGisInfo_TS;
				for(int i=0;i<301;i++)
				{
					gisrec_msg.x[i] = (long)now_gis.AlignedData().x[i];
					gisrec_msg.y[i] = (long)now_gis.AlignedData().y[i];
				}
				udp_publisher_.publish(gisrec_msg);

				C_InsFrame cur_ins;
				if(!InsFrameHis.GetLastInsFrame(cur_ins))
				{
					return;
				}
				for(int i=0; i<301; i++)
				{
					last_gis_earth[i] = conv3000toENU((int)(gisrec_msg.x[i]), (int)(gisrec_msg.y[i]), cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
				}

				GisFrame.haveLastGis = true;
			}
		}
		else
		{
			gisrec_msg.data[0] = LastGisInfo_TS;
			for(int i=0;i<301;i++)
			{
				gisrec_msg.x[i] = (long)now_gis.AlignedData().x[i];
				gisrec_msg.y[i] = (long)now_gis.AlignedData().y[i];
			}
			udp_publisher_.publish(gisrec_msg);

			C_InsFrame cur_ins;
			if(!InsFrameHis.GetLastInsFrame(cur_ins))
			{
				return;
			}
			for(int i=0; i<301; i++)
			{
				last_gis_earth[i] = conv3000toENU((int)(gisrec_msg.x[i]), (int)(gisrec_msg.y[i]), cur_ins.position[0], cur_ins.position[1], cur_ins.attitude[2]);
			}

			GisFrame.haveLastGis = true;
		}
//---------------------------------------------Dead History end--------------------------------------------------

		//---------------start---------------//
//		if(haveLastGisrecMsg)
//		{
//			long thre =
//
//			long last_min_disdis = -1;
//			for(int i=0; i<301; i++)
//			{
//				long dx = 1500 - last_gisrec_msg.x[i];
//				long dy = 1000 - last_gisrec_msg.y[i];
//				long cur_disdis = dx*dx + dy*dy;
//				if(i == 0)
//				{
//					last_min_disdis = cur_disdis;
//				}
//				else
//				{
//					if(cur_disdis < last_min_disdis)
//					{
//						last_min_disdis = cur_disdis;
//					}
//				}
//			}
//
//			long min_disdis = -1;
//			for(int i=0; i<301; i++)
//			{
//				long dx = 1500 - gisrec_msg.x[i];
//				long dy = 1000 - gisrec_msg.y[i];
//				long cur_disdis = dx*dx + dy*dy;
//				if(i == 0)
//				{
//					min_disdis = cur_disdis;
//				}
//				else
//				{
//					if(cur_disdis < min_disdis)
//					{
//						min_disdis = cur_disdis;
//					}
//				}
//			}
//
//			long tmp_abs = (last_min_disdis - min_disdis < 0) ? (min_disdis - last_min_disdis) : (last_min_disdis - min_disdis);
//		}
//		else
//		{
//			// log in last_gisrec_msg...
//
//			haveLastGisrecMsg = true;
//		}
		//---------------end---------------//

		//---------------start---------------//
		double EastErrorTmp = (now_gis.getEastErrorInit() + now_gis.getEastErrorLocal()) / 50.0;
		double NorthErrorTmp = (now_gis.getNorthErrorInit() + now_gis.getNorthErrorLocal()) / 50.0;
		/*
		gisvectorinfo_msg.TimeStamp = 0;
		gisvectorinfo_msg.data[0] = GisFrame.SendLvl;
		gisvectorinfo_msg.EastError = EastErrorTmp;
		gisvectorinfo_msg.NorthError = NorthErrorTmp;
		gis_vector_pub.publish(gisvectorinfo_msg);
		*/
		//---------------end---------------//
	}

#if USING_SCANINFOV2
	void C_LocalMap::transformTopic_ScanInfo(const in2_msgs::ScanInfoV2::ConstPtr &scaninfo_sub)
	{
		LocalMapCnter_10ms++;
		if(LocalMapCnter_10ms >= 60000) // 10min
		{
			LocalMapCnter_10ms = 0;
			LocalMapCnter_10min++;
		}

		LastScanInfo_TS = ScanFrameHis.addSample(scaninfo_sub->sendtime,scaninfo_sub->ScanInfoX,scaninfo_sub->ScanInfoY,InsFrameHis);
		C_ScanFrame lastScanFrame;
		ScanFrameHis.GetLastFrame(lastScanFrame);
		CostMap.SetCostMat(lastScanFrame);
	}
#endif
#if USING_SCANINFOBG
	void C_LocalMap::transformTopic_ScanInfoBG(const in2_msgs::ScanInfoV2::ConstPtr &scaninfo_sub)
	{
		LastScanInfo_TS = ScanFrameHis.addSample(scaninfo_sub->sendtime,scaninfo_sub->ScanInfoX,scaninfo_sub->ScanInfoY,InsFrameHis);
		C_ScanFrame lastScanFrame;
		ScanFrameHis.GetLastFrame(lastScanFrame);
		CostMap.SetCostMat(lastScanFrame);
	}
#endif
	void C_LocalMap::transformTopic_RoadEdge(const in2_msgs::RoadEdge::ConstPtr &roadedge_sub)
	{
		RoadEdgeFrame.setWithSample(roadedge_sub);
	}

	void C_LocalMap::transformTopic_LaneInfo(const in2_msgs::LaneInfoV2::ConstPtr &laneinfo_sub)
	{
//		ROS_ERROR("Localmap get a lane information!");
		//-------------------------------------------------------------
		long int delta_ms = (LocalMapCnter_10min-LaneCnter_10min)*10*600 + (LocalMapCnter_10ms-LaneCnter_10ms)*10; // ms
		if(delta_ms > 300) // ms
		{
			LaneModel.LinearSet.LastGisOrEdge = 1;
			LaneModel.LinearSet.LinearModelSet[LaneModel.LinearSet.likiestModel_index].ObjIDInitial = true;
			LaneModel.LinearSet.LinearModelSet[LaneModel.LinearSet.likiestModel_index].OddLastDir = 1;
		}
		LaneCnter_10min = LocalMapCnter_10min;
		LaneCnter_10ms  = LocalMapCnter_10ms;
		//-------------------------------------------------------------
		//LastLaneInfo_TS = LaneModel.addSample(laneinfo_sub->sendtime,laneinfo_sub->r,laneinfo_sub->theta,laneinfo_sub->type,laneinfo_sub->beliefe,laneinfo_sub->isCurve,laneinfo_sub->points_x,laneinfo_sub->points_y,InsFrameHis,GisFrame);
		C_InsFrame cur_ins;
		if(!InsFrameHis.GetLastInsFrame(cur_ins))
		{
			return;
		}
		LastLaneInfo_TS = LaneModel.addSample(laneinfo_sub->sendtime,laneinfo_sub->r,laneinfo_sub->theta,
				laneinfo_sub->type,laneinfo_sub->beliefe,laneinfo_sub->isCurve,
				laneinfo_sub->points_x,laneinfo_sub->points_y,
				InsFrameHis,GisFrame,CostMap,RoadEdgeFrame,cur_ins);
		laneinfo_msg.sendtime = laneinfo_sub->sendtime;
//		ROS_ERROR("%d", laneinfo_sub->sendtime);

		int counter = 0;
		if(LaneModel.LinearSet.likiestModel_index != -1)
		{
			//----------------------------------------
			double model_angle = LaneModel.LinearSet.LinearModelSet[LaneModel.LinearSet.likiestModel_index].getModelAngle();
			double DeltaAngle = fabs(RoadEdgeFrame.Angle - model_angle);
			if(DeltaAngle < 6)
			{
			//----------------------------------------
				for(int i=LaneModel.LinearSet.line_1_index;i<=LaneModel.LinearSet.line_2_index;i++)
				{
					laneinfo_msg.r[counter] = 0;
					laneinfo_msg.theta[counter] = 0;
					laneinfo_msg.type[counter] = 1;
					laneinfo_msg.isCurve[counter] = 0;
					laneinfo_msg.beliefe[counter] = 15;
					double k,b,db;
					k = LaneModel.LinearSet.LinearModelSet[LaneModel.LinearSet.likiestModel_index].k;
					b = LaneModel.LinearSet.LinearModelSet[LaneModel.LinearSet.likiestModel_index].b;
					db = LaneModel.LinearSet.LinearModelSet[LaneModel.LinearSet.likiestModel_index].db;
					for(int j=0;j<31;j++)
					{
						double temp_y = (double)j*100.0;
						double temp_i = (double)i;
						laneinfo_msg.points_x[counter*31+j] = (int)(k*temp_y + b + temp_i*db);
						laneinfo_msg.points_y[counter*31+j] = temp_y;
					}
					counter++;
					if(counter>=20)
					{
						goto KEY;
						fprintf(stderr,"ERROR: counter(%d)>=20!!%d in Linear\n",counter,i);
					}
				}
			//----------------------------------------
			}
			else
			{
				
			}
			//----------------------------------------
		}
		if(LaneModel.StoplineSet.likiestModel_index != -1)
		{
			laneinfo_msg.r[counter] = 0;
			laneinfo_msg.theta[counter] = 0;
			laneinfo_msg.type[counter] = 5;
			laneinfo_msg.isCurve[counter] = 0;
			laneinfo_msg.beliefe[counter] = 15;
			double k,b;
			k = LaneModel.StoplineSet.StopLineModelSet[LaneModel.StoplineSet.likiestModel_index].k;
			b = LaneModel.StoplineSet.StopLineModelSet[LaneModel.StoplineSet.likiestModel_index].b;
			for(int j=0;j<31;j++)
			{
				double temp_x = (double)j*100.0;
				laneinfo_msg.points_x[counter*31+j] = (int)temp_x;
				laneinfo_msg.points_y[counter*31+j] = (int)(k*temp_x + b);
			}
			counter++;
			if(counter>=20)
			{
				goto KEY;
				fprintf(stderr,"ERROR: counter(%d)>=20!!%d in Stopline\n",counter,1);
			}
		}
		for(int i=0;i<LaneModel.c_len;i++)
		{
			laneinfo_msg.r[counter] = LaneModel.CurveSample[i].r;
			laneinfo_msg.theta[counter] = LaneModel.CurveSample[i].theta;
			laneinfo_msg.type[counter] = LaneModel.CurveSample[i].type;
			laneinfo_msg.isCurve[counter] = LaneModel.CurveSample[i].isCurve;
			laneinfo_msg.beliefe[counter] = LaneModel.CurveSample[i].beliefe;
			for(int j=0;j<31;j++)
			{
				laneinfo_msg.points_x[counter*31+j] = LaneModel.CurveSample[i].points_x[j];
				laneinfo_msg.points_y[counter*31+j] = LaneModel.CurveSample[i].points_y[j];
			}
			counter++;
			if(counter>=20)
			{
				goto KEY;
				fprintf(stderr,"ERROR: counter(%d)>=20!!%d in Curve\n",counter,i);
			}
		}
		while(counter<20)
		{
			laneinfo_msg.type[counter] = 0;
			counter++;
		}
		KEY:
		udp_publisher_laneinfo_.publish(laneinfo_msg);
	}

	void C_LocalMap::transformTopic_InsInfo(const in2_msgs::InsInfo::ConstPtr &insinfo_sub)
	{
		//fprintf(stdout,"##################%d\n",temp_counter++);
		C_InsFrame temp(insinfo_sub->sendtime,insinfo_sub->second,insinfo_sub->longitude,insinfo_sub->latitude,insinfo_sub->position,insinfo_sub->attitude,insinfo_sub->velocity);
		LastInsInfo_TS = InsFrameHis.addInsFrameHis(temp);
	}

	void C_LocalMap::DrawTimeStamp(Mat &Image)
	{

		int fontFace = FONT_HERSHEY_SIMPLEX;
		double fontScale = 1.5;
		int thickness = 3;
		char title[30];
		char data[30];
		//char delta[30];
		int x_0 = 2300;
		int x_1 = x_0 + 250;
		//int x_2 = x_1 + 300;
		int y_0 = 100;
		int d_y = 70;
		sprintf(title,"Ins_TS");
		sprintf(data,"%ld",LastInsInfo_TS);
		putText(Image, title, Point(x_0,y_0), fontFace, fontScale, InsInfoColor, thickness);
		putText(Image, data, Point(x_1,y_0), fontFace, fontScale, InsInfoColor, thickness,8);
		y_0 += d_y;
		sprintf(title,"Gis_TS");
		sprintf(data,"%ld", LastInsInfo_TS - LastGisInfo_TS);
		putText(Image, title, Point(x_0,y_0), fontFace, fontScale, GisPathColor, thickness);
		putText(Image, data, Point(x_1,y_0), fontFace, fontScale, GisPathColor, thickness,8);
		y_0 += d_y;
		sprintf(title,"Scan_TS");
		sprintf(data,"%ld", LastInsInfo_TS - LastScanInfo_TS);
		putText(Image, title, Point(x_0,y_0), fontFace, fontScale, ScanInfoColor, thickness);
		putText(Image, data, Point(x_1,y_0), fontFace, fontScale, ScanInfoColor, thickness,8);
		y_0 += d_y;
		sprintf(title,"Lane_TS");
		sprintf(data,"%ld", LastInsInfo_TS - LastLaneInfo_TS);
		putText(Image, title, Point(x_0,y_0), fontFace, fontScale, LinearSampleColor, thickness);
		putText(Image, data, Point(x_1,y_0), fontFace, fontScale, LinearSampleColor, thickness,8);
	}

	void C_LocalMap::InitialAlignmeng(Mat &Image)
	{
		C_InsFrame dst_Ins;
		InsFrameHis.GetLastInsFrame(dst_Ins);
		C_LinearModel Mid ;
		C_StoplineModel MidVertical;
		Point CrossPoint;
		if(LaneModel.LinearSet.getMid(Mid,MidVertical,CrossPoint,dst_Ins) == true)
		{
			Mid.Draw(Image,10,Scalar(0,0,0),false);
			MidVertical.Draw(Image,Scalar(0,0,0));

			int fontFace = FONT_HERSHEY_SIMPLEX;
			double fontScale = 1.5;
			int thickness = 3;
			char str[30];
			sprintf(str,"%f",Mid.directAgnle(dst_Ins));
			putText(Image, str, Point(100,100), fontFace, fontScale, Scalar(0,0,0), thickness);

			int gisPoint_index;
			Point gisPoint;
			double gisPoint_dirAngle;

			if(GisFrame.getReferencePoint(&MidVertical,gisPoint_index,gisPoint,gisPoint_dirAngle,dst_Ins) == true)
			{
				int fontFace = FONT_HERSHEY_SIMPLEX;
				double fontScale = 1.5;
				int thickness = 3;
				char str[30];
				sprintf(str,"%f",gisPoint_dirAngle);
				putText(Image, str, Point(100,200), fontFace, fontScale, Scalar(0,0,0), thickness);
				circle(Image,Point(gisPoint.x,3000-gisPoint.y),16,Scalar::all(0),-1);
				if(azimuthDiff(gisPoint_dirAngle,Mid.directAgnle(dst_Ins)) < 1.0)
				{
					Point foot;
					Mid.getPerpenditualFoot(10,gisPoint.x,gisPoint.y,foot.x,foot.y);
					line(Image,Point(foot.x,3000-foot.y),Point(gisPoint.x,3000-gisPoint.y),Scalar(255,100,100),10);
					double u_error = foot.x - gisPoint.x;
					double v_error = foot.y - gisPoint.y;
					GisFrame.AddNewError(u_error,v_error,dst_Ins);
					//GisFrame.SetLocalError(u_error,v_error,dst_Ins);
					char norm[30];
					sprintf(norm,"Norm:%f",sqrt(u_error*u_error + v_error*v_error));
					putText(Image, norm, Point(100,300), fontFace, fontScale, Scalar(0,0,0), thickness);
				}
			}
		}
	}

	void C_LocalMap::CorrectGis()
	{
		C_InsFrame dst_Ins;
		InsFrameHis.GetLastInsFrame(dst_Ins);
		C_LinearModel Mid ;
		C_StoplineModel MidVertical;
		Point CrossPoint;
		if(LaneModel.LinearSet.getMid(Mid,MidVertical,CrossPoint,dst_Ins) == true)
		{
			//double Lane_Width = Mid.getLaneWidth();
			int gisPoint_index;
			Point gisPoint;
			double gisPoint_dirAngle;
			if(GisFrame.getAbsolutePoint(&MidVertical,gisPoint_index,gisPoint,gisPoint_dirAngle,dst_Ins) == true)
			{
				if( azimuthDiff(gisPoint_dirAngle,Mid.directAgnle(dst_Ins)) < 1.0 )
				{
					Point foot;
					Mid.getPerpenditualFoot(10,gisPoint.x,gisPoint.y,foot.x,foot.y);
					//line(Image,Point(foot.x,3000-foot.y),Point(gisPoint.x,3000-gisPoint.y),Scalar(255,100,100),10);
					//double u_error = foot.x - gisPoint.x;
					//double v_error = foot.y - gisPoint.y;
					//GisFrame.AddNewError(u_error,v_error,dst_Ins);
					//GisFrame.SetLocalError(u_error,v_error,dst_Ins,0.49*Lane_Width);
/*
					if(GisFrame.dist_to_intersection >= 0)
					{
						static double E_1;
						static double E_2;
						static double N_1;
						static double N_2;
						static double A_1;
						static double A_2;
						static bool first = true;
						double azimuth = - dst_Ins.attitude[2]*PI/180.0;
						double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
						double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
						double Error_azimuth = atan2(North_error,East_error)*180.0/PI;
						double E_3 = East_error;
						double N_3 = North_error;
						double A_3 = Error_azimuth;

						if(first == true)
						{
							E_1 = East_error;
							E_2 = East_error;
							N_1 = North_error;
							N_2 = North_error;
							A_1 = Error_azimuth;
							A_2 = Error_azimuth;
						}
						else
						{
							E_1 = E_2;
							E_2 = East_error;
							N_1 = N_2;
							N_2 = North_error;
							A_1 = A_2;
							A_2 = Error_azimuth;
						}
						double Selected_E;
						double Selected_N;
						double Selected_A;
						if( ( A_1 <= A_2 && A_1 >= A_3 ) || ( A_1 <= A_3 && A_1 >= A_2 ) )
						{
							Selected_A = A_1;
							Selected_E = E_1;
							Selected_N = N_1;
						}
						if( ( A_2 <= A_1 && A_2 >= A_3 ) || ( A_2 <= A_3 && A_2 >= A_1 ) )
						{
							Selected_A = A_2;
							Selected_E = E_2;
							Selected_N = N_2;
						}
						if( ( A_3 <= A_2 && A_3 >= A_1 ) || ( A_3 <= A_1 && A_3 >= A_2 ) )
						{
							Selected_A = A_3;
							Selected_E = E_3;
							Selected_N = N_3;
						}
						C_GisErrorFrame newframe(GisFrame.timestamp,gisPoint_dirAngle,Mid.directAgnle(dst_Ins),GisFrame.dist_to_intersection,Selected_E,Selected_N,Selected_A);
						GisError.addGisErrorFrame(newframe);
						double estimate_E_error;
						double estimate_N_error;
						GisError.estimateError(estimate_E_error,estimate_N_error);
					}
					*/
				}
			}
		}
	}
	void C_LocalMap::EstimateGisError()
	{
		C_InsFrame dst_Ins;
		InsFrameHis.GetLastInsFrame(dst_Ins);
		C_LinearModel Mid ;
		C_StoplineModel MidVertical;
		Point CrossPoint;
		if(LaneModel.LinearSet.getMid(Mid,MidVertical,CrossPoint,dst_Ins) == true)
		{
			//double Lane_Width = Mid.getLaneWidth();
			int gisPoint_index;
			Point gisPoint;
			double gisPoint_dirAngle;
			//if(GisFrame.getRawPoint(&MidVertical,gisPoint_index,gisPoint,gisPoint_dirAngle,dst_Ins) == true)
			if(GisFrame.getRawPoint(CrossPoint,gisPoint_index,gisPoint,gisPoint_dirAngle,dst_Ins) == true)
			{
				fprintf(stdout,"\n\n\n");
				fprintf(stdout,"gisPoint_dirAngle        = %lf\n",gisPoint_dirAngle);
				fprintf(stdout,"Mid.directAgnle(dst_Ins) = %lf\n",Mid.directAgnle(dst_Ins));
				if( azimuthDiff(gisPoint_dirAngle,Mid.directAgnle(dst_Ins)) < 5.0 )
				{
					//Point foot;
					//Mid.getPerpenditualFoot(10,gisPoint.x,gisPoint.y,foot.x,foot.y);
					double foot_x;
					double foot_y;
					double gisPoint_x = (double)gisPoint.x;
					double gisPoint_y = (double)gisPoint.y;
					Mid.getPerpenditualFoot(10,gisPoint_x,gisPoint_y,foot_x,foot_y);

					//Mat image;
					//image.create(3000,3000,CV_8UC3);
					//namedWindow( "Error", 0 );
					//line(image,Point(foot.x,3000-foot.y),Point(gisPoint.x,3000-gisPoint.y),Scalar(255,100,100),10);
					//imshow("Error",image);
					//waitKey(5);
					double u_error = foot_x - gisPoint_x;
					double v_error = foot_y - gisPoint_y;
					//GisFrame.AddNewError(u_error,v_error,dst_Ins);
					//GisFrame.SetLocalError(u_error,v_error,dst_Ins,0.49*Lane_Width);

					fprintf(stdout,"gisPoint_dirAngle        = %lf\n",gisPoint_dirAngle);
					fprintf(stdout,"Mid.directAgnle(dst_Ins) = %lf\n",Mid.directAgnle(dst_Ins));
					fprintf(stdout,"foot %f,%f\n",foot_x,foot_y);
					fprintf(stdout,"gisp %f,%f\n",gisPoint_x,gisPoint_y);
					if(GisFrame.dist_to_intersection >= 0)
					{
						double azimuth = - dst_Ins.attitude[2]*PI/180.0;
						double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
						double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
						double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
						Error_azimuth = fmod(Error_azimuth+360.0,360.0);

						fprintf(stdout,"error %lf,%lf\n",u_error,v_error);

						C_GisErrorFrame newframe(GisFrame.timestamp,gisPoint_dirAngle,Mid.directAgnle(dst_Ins),GisFrame.dist_to_intersection,East_error,North_error,Error_azimuth);
						GisError.addGisErrorFrame(newframe);
						double estimate_E_error;
						double estimate_N_error;
						GisError.estimateError(estimate_E_error,estimate_N_error);
						if(GisError.estimate_good > 1)
						{
							GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
						}
					}
				}
			}
		}
	}

	void C_LocalMap::Correcting(Mat &Image)
	{
			C_InsFrame dst_Ins;
			InsFrameHis.GetLastInsFrame(dst_Ins);
			C_LinearModel Mid ;
			C_StoplineModel MidVertical;
			Point CrossPoint;
			if(LaneModel.LinearSet.getMid(Mid,MidVertical,CrossPoint,dst_Ins) == true)
			{
				Mid.Draw(Image,10,Scalar(0,0,0),false);
				//MidVertical.Draw(Image,Scalar(0,0,0));

				int fontFace = FONT_HERSHEY_SIMPLEX;
				double fontScale = 1.5;
				int thickness = 3;
				char str[30];
				sprintf(str,"%f",Mid.directAgnle(dst_Ins));
				putText(Image, str, Point(100,100), fontFace, fontScale, Scalar(0,0,0), thickness);

				int gisPoint_index;
				Point gisPoint;
				double gisPoint_dirAngle;

				if(GisFrame.getAbsolutePoint(&MidVertical,gisPoint_index,gisPoint,gisPoint_dirAngle,dst_Ins) == true)
				{
					int fontFace = FONT_HERSHEY_SIMPLEX;
					double fontScale = 1.5;
					int thickness = 3;
					char str[30];
					sprintf(str,"%f",gisPoint_dirAngle);
					putText(Image, str, Point(100,200), fontFace, fontScale, Scalar(0,0,0), thickness);
					circle(Image,Point(gisPoint.x,3000-gisPoint.y),16,Scalar::all(0),-1);
					if(azimuthDiff(gisPoint_dirAngle,Mid.directAgnle(dst_Ins)) < 1.0)
					{
						Point foot;
						Mid.getPerpenditualFoot(10,gisPoint.x,gisPoint.y,foot.x,foot.y);
						line(Image,Point(foot.x,3000-foot.y),Point(gisPoint.x,3000-gisPoint.y),Scalar(255,100,100),10);
						double u_error = foot.x - gisPoint.x;
						double v_error = foot.y - gisPoint.y;
						//GisFrame.AddNewError(u_error,v_error,dst_Ins);
						GisFrame.SetLocalError(u_error,v_error,dst_Ins,0.3*50.0);
						char norm[30];
						sprintf(norm,"Norm:%f",sqrt(u_error*u_error + v_error*v_error));
						putText(Image, norm, Point(100,300), fontFace, fontScale, Scalar(0,0,0), thickness);
					}
				}

			}
		}

	void C_LocalMap::CorrectGIsError()
	//use GisError.InitComplited == true to control set error
	{
		//INS
		tempDraw.init();
		C_InsFrame dst_Ins;
		if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
		{
			//Gis
			Point gisRawPoint;
			double gisRawPoint_angle3K;
			if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true )//&& GisFrame.CornerInStatus == 0 )//&& GisFrame.CornerOutStatus == 0)
			{
				Point gisInitPoint;
				double gisInitPoint_angle3K;
				GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
				tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);
				//GisFrame.printError();
				//Lane
				C_LinearModel Mid;
				if( LaneModel.LinearSet.getMid(gisInitPoint,Mid,dst_Ins) == true )
				{
					double gisInitPoint_Foot_x;
					double gisInitPoint_Foot_y;
					Mid.getPerpenditualFoot(10,gisInitPoint.x,gisInitPoint.y,gisInitPoint_Foot_x,gisInitPoint_Foot_y);

					double u_error_local = gisInitPoint_Foot_x - (double)gisInitPoint.x;
					double v_error_local = gisInitPoint_Foot_y - (double)gisInitPoint.y;
					tempDraw.add(Point(gisInitPoint_Foot_x,gisInitPoint_Foot_y),Scalar(0,0,0),20);
					GisFrame.SetLocalError(u_error_local,v_error_local,dst_Ins,Mid.getLaneWidth());
					if(angle3KDiff(gisRawPoint_angle3K,Mid.getAngle()) < 5.0)
					{
						double gisRawPoint_Foot_x;
						double gisRawPoint_Foot_y;
						Mid.getPerpenditualFoot(10,gisRawPoint.x,gisRawPoint.y,gisRawPoint_Foot_x,gisRawPoint_Foot_y);

						double u_error = gisRawPoint_Foot_x - (double)gisRawPoint.x;
						double v_error = gisRawPoint_Foot_y - (double)gisRawPoint.y;
						if(normof(u_error,v_error) < MAX_GISERROR_NORM)
						{
							double azimuth = - dst_Ins.attitude[2]*PI/180.0;
							double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
							double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
							double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
							Error_azimuth = fmod(Error_azimuth+360.0,360.0);

							fprintf(stdout,"error %lf,%lf\n",u_error,v_error);

							C_GisErrorFrame newframe(GisFrame.timestamp,East_error,North_error,Error_azimuth);
							GisError.addGisErrorFrame(newframe);
							double estimate_E_error;
							double estimate_N_error;
							GisError.estimateError(estimate_E_error,estimate_N_error);
							if(GisError.estimate_good > 1 && GisError.InitComplited == true)
							{
								GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
							}
						}
					}
				}
				else
				{
					GisFrame.SetLocalError(0,0,dst_Ins,10.0);
				}
			}
		}
	}

	void C_LocalMap::CorrectGisError_2()
	//use 1500,1000 to calculate the error
	{

		//INS
		tempDraw.init();
		C_InsFrame dst_Ins;
		if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
		{
			//Gis
			Point gisRawPoint;
			double gisRawPoint_angle3K;
			if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true && GisFrame.CornerInStatus == 0 )//&& GisFrame.CornerOutStatus == 0)
			{
				Point gisInitPoint;
				double gisInitPoint_angle3K;
				GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
				tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);
				//GisFrame.printError();
				//Lane
				C_LinearModel Mid;
				if(GisError.estimate_good <2 )
				{
					gisInitPoint.x = 1500;
					gisInitPoint.y = 1000;
				}
				if( LaneModel.LinearSet.getMid(gisInitPoint,Mid,dst_Ins) == true )
				{
					double gisInitPoint_Foot_x;
					double gisInitPoint_Foot_y;
					Mid.getPerpenditualFoot(10,gisInitPoint.x,gisInitPoint.y,gisInitPoint_Foot_x,gisInitPoint_Foot_y);

					double u_error_local = gisInitPoint_Foot_x - (double)gisInitPoint.x;
					double v_error_local = gisInitPoint_Foot_y - (double)gisInitPoint.y;
					tempDraw.add(Point(gisInitPoint_Foot_x,gisInitPoint_Foot_y),Scalar(0,0,0),20);
					GisFrame.SetLocalError(u_error_local,v_error_local,dst_Ins,Mid.getLaneWidth());
					if(angle3KDiff(gisRawPoint_angle3K,Mid.getAngle()) < 5.0)
					{
						double gisRawPoint_Foot_x;
						double gisRawPoint_Foot_y;
						Mid.getPerpenditualFoot(10,gisRawPoint.x,gisRawPoint.y,gisRawPoint_Foot_x,gisRawPoint_Foot_y);

						double u_error = gisRawPoint_Foot_x - (double)gisRawPoint.x;
						double v_error = gisRawPoint_Foot_y - (double)gisRawPoint.y;
						if(normof(u_error,v_error) < MAX_GISERROR_NORM)
						{
							double azimuth = - dst_Ins.attitude[2]*PI/180.0;
							double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
							double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
							double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
							Error_azimuth = fmod(Error_azimuth+360.0,360.0);

							fprintf(stdout,"error %lf,%lf\n",u_error,v_error);

							C_GisErrorFrame newframe(GisFrame.timestamp,East_error,North_error,Error_azimuth);
							GisError.addGisErrorFrame(newframe);
							double estimate_E_error;
							double estimate_N_error;
							GisError.estimateError(estimate_E_error,estimate_N_error);
 							if(GisError.estimate_good > 0)
							{
								GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
							}
						}
					}
				}
				else
				{
					GisFrame.SetLocalError(0,0,dst_Ins,10.0);
				}
			}
		}
	}


	void C_LocalMap::CorrectGisError_3()
	//use the mid get by BorderInfo to cal the error
	{

		double SetLocalError_Threashold_radio_by_LaneWide = 3.0;

		tempDraw.init();
		//Lane Num in Border
		int LaneNumInBorder = LaneModel.LinearSet.border_2_index - LaneModel.LinearSet.border_1_index;
		if(LaneNumInBorder>0 && ( GisFrame.laneNum == LaneNumInBorder || GisFrame.laneNum == LaneNumInBorder*2 ))
		{
			//set local error and add error sampe to GISERROR
			//INS
			C_InsFrame dst_Ins;
			if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
			{
				//Gis
				Point gisRawPoint;
				double gisRawPoint_angle3K;
				if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true )//&& GisFrame.CornerInStatus == 0 )//&& GisFrame.CornerOutStatus == 0)
				{
					Point gisInitPoint;
					double gisInitPoint_angle3K;
					GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
					tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);
					//GisFrame.printError();
					//Lane
					C_LinearModel Mid;
					if(  LaneModel.LinearSet.getMid(Mid,dst_Ins,LaneModel.LinearSet.line_1_index,LaneModel.LinearSet.line_2_index) == true )
					{
						double gisInitPoint_Foot_x;
						double gisInitPoint_Foot_y;
						Mid.getPerpenditualFoot(10,gisInitPoint.x,gisInitPoint.y,gisInitPoint_Foot_x,gisInitPoint_Foot_y);

						double u_error_local = gisInitPoint_Foot_x - (double)gisInitPoint.x;
						double v_error_local = gisInitPoint_Foot_y - (double)gisInitPoint.y;
						tempDraw.add(Point(gisInitPoint_Foot_x,gisInitPoint_Foot_y),Scalar(0,0,0),20);
						GisFrame.SetLocalError(u_error_local,v_error_local,dst_Ins,SetLocalError_Threashold_radio_by_LaneWide*Mid.getLaneWidth());
						if(angle3KDiff(gisRawPoint_angle3K,Mid.getAngle()) < 5.0)
						{
							double gisRawPoint_Foot_x;
							double gisRawPoint_Foot_y;
							Mid.getPerpenditualFoot(10,gisRawPoint.x,gisRawPoint.y,gisRawPoint_Foot_x,gisRawPoint_Foot_y);

							double u_error = gisRawPoint_Foot_x - (double)gisRawPoint.x;
							double v_error = gisRawPoint_Foot_y - (double)gisRawPoint.y;
							if(normof(u_error,v_error) < MAX_GISERROR_NORM)
							{
								double azimuth = - dst_Ins.attitude[2]*PI/180.0;
								double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
								double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
								double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
								Error_azimuth = fmod(Error_azimuth+360.0,360.0);

								fprintf(stdout,"error %lf,%lf\n",u_error,v_error);

								C_GisErrorFrame newframe(GisFrame.timestamp,East_error,North_error,Error_azimuth);
								GisError.addGisErrorFrame(newframe);
								double estimate_E_error;
								double estimate_N_error;
								GisError.estimateError(estimate_E_error,estimate_N_error);
								if(GisError.estimate_good > 0 && GisError.InitComplited == true)
								{
									GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
								}
							}
						}
					}
					else
					{
						GisFrame.SetLocalError(0,0,dst_Ins,10.0);
					}
				}
			}
		}
		else
		{
			//only set local error
			//INS
			C_InsFrame dst_Ins;
			if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
			{
				//Gis
				Point gisRawPoint;
				double gisRawPoint_angle3K;
				if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true )//&& GisFrame.CornerInStatus == 0 )//&& GisFrame.CornerOutStatus == 0)
				{
					Point gisInitPoint;
					double gisInitPoint_angle3K;
					GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
					//tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);
					//GisFrame.printError();
					//Lane
					C_LinearModel Mid;
					if(  LaneModel.LinearSet.getMid(Mid,dst_Ins,LaneModel.LinearSet.line_1_index,LaneModel.LinearSet.line_2_index) == true )
					{
						double gisInitPoint_Foot_x;
						double gisInitPoint_Foot_y;
						Mid.getPerpenditualFoot(10,gisInitPoint.x,gisInitPoint.y,gisInitPoint_Foot_x,gisInitPoint_Foot_y);

						double u_error_local = gisInitPoint_Foot_x - (double)gisInitPoint.x;
						double v_error_local = gisInitPoint_Foot_y - (double)gisInitPoint.y;
						//tempDraw.add(Point(gisInitPoint_Foot_x,gisInitPoint_Foot_y),Scalar(0,0,0),20);
						GisFrame.SetLocalError(u_error_local,v_error_local,dst_Ins,SetLocalError_Threashold_radio_by_LaneWide*Mid.getLaneWidth());
					}
				}
			}
		}
	}

	void C_LocalMap::CorrectGisError_4(const int32_t _ms, C_InsFrame cur_ins)
	{
		tempDraw.init();
		//Lane Num in Border
		int LaneNumInBorder = LaneModel.LinearSet.border_2_index - LaneModel.LinearSet.border_1_index;
#if VERSION_SUB_URBAN
		if( LaneNumInBorder>0 && GisFrame.laneNum == LaneNumInBorder )
#endif
#if VERSION_URBAN
//		if(LaneNumInBorder>0 && ( GisFrame.laneNum == LaneNumInBorder || GisFrame.laneNum == LaneNumInBorder*2 ))
		if( LaneNumInBorder>0 && GisFrame.laneNum == LaneNumInBorder )
#endif
		{
			//set local error and add error sampe to GISERROR
			//INS
			C_InsFrame dst_Ins;
			if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
			{
				//Gis
				Point gisRawPoint;
				double gisRawPoint_angle3K;
				if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true && GisFrame.CornerInStatus == 0 && GisFrame.CornerOutStatus == 0)
				{
					Point gisInitPoint;
					double gisInitPoint_angle3K;
					GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
					tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);
					//GisFrame.printError();
					//Lane
					int temp_line_1;
					int temp_line_2;
					LaneModel.LinearSet.LinearModelSet[ LaneModel.LinearSet.likiestModel_index].getEdegByBorderAndGis(temp_line_1,temp_line_2,LaneModel.LinearSet.border_1_index,LaneModel.LinearSet.border_2_index,GisFrame,_ms,cur_ins);
					C_LinearModel Mid;
					if(  LaneModel.LinearSet.getMid(Mid,dst_Ins,temp_line_1,temp_line_2) == true )
					{
						double gisInitPoint_Foot_x;
						double gisInitPoint_Foot_y;
						Mid.getPerpenditualFoot(10,gisInitPoint.x,gisInitPoint.y,gisInitPoint_Foot_x,gisInitPoint_Foot_y);
						tempDraw.add(Point(gisInitPoint_Foot_x,gisInitPoint_Foot_y),Scalar(0,0,0),20);
						if(angle3KDiff(gisRawPoint_angle3K,Mid.getAngle()) < 18.0)
//						if(angle3KDiff(gisRawPoint_angle3K,Mid.getAngle()) < 20.0)
						{
							double gisRawPoint_Foot_x;
							double gisRawPoint_Foot_y;
							Mid.getPerpenditualFoot(10,gisRawPoint.x,gisRawPoint.y,gisRawPoint_Foot_x,gisRawPoint_Foot_y);

							double u_error = gisRawPoint_Foot_x - (double)gisRawPoint.x;
							double v_error = gisRawPoint_Foot_y - (double)gisRawPoint.y;
							if(normof(u_error,v_error) < MAX_GISERROR_NORM)
							{
								double azimuth = - dst_Ins.attitude[2]*PI/180.0;
								double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
								double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
								double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
								Error_azimuth = fmod(Error_azimuth+360.0,360.0);

								fprintf(stdout,"error %lf,%lf\n",u_error,v_error);

								C_GisErrorFrame newframe(GisFrame.timestamp,East_error,North_error,Error_azimuth);
								GisError.addGisErrorFrame(newframe,true);
								double estimate_E_error;
								double estimate_N_error;
								GisError.estimateError(estimate_E_error,estimate_N_error);
								if(GisError.estimate_good > 0 && GisError.InitComplited == true)
								{
									GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
								}
							}
						}
					}
				}
			}
		}
		else
		{
			//set local error and add error sampe to GISERROR
			//INS
			C_InsFrame dst_Ins;
			if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
			{
				//Gis
				Point gisRawPoint;
				double gisRawPoint_angle3K;
				if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true && GisFrame.CornerInStatus == 0 && GisFrame.CornerOutStatus == 0)
				{
					Point gisInitPoint;
					double gisInitPoint_angle3K;
					GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
					tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);
					//GisFrame.printError();
					//Lane
					C_LinearModel Mid;
					if(  LaneModel.LinearSet.getMid(gisInitPoint,Mid,dst_Ins) == true )
					{
						double gisInitPoint_Foot_x;
						double gisInitPoint_Foot_y;
						Mid.getPerpenditualFoot(10,gisInitPoint.x,gisInitPoint.y,gisInitPoint_Foot_x,gisInitPoint_Foot_y);

						tempDraw.add(Point(gisInitPoint_Foot_x,gisInitPoint_Foot_y),Scalar(0,0,0),20);
 						if(angle3KDiff(gisRawPoint_angle3K,Mid.getAngle()) < 5.0)
						{
							double gisRawPoint_Foot_x;
							double gisRawPoint_Foot_y;
							Mid.getPerpenditualFoot(10,gisRawPoint.x,gisRawPoint.y,gisRawPoint_Foot_x,gisRawPoint_Foot_y);

							double u_error = gisRawPoint_Foot_x - (double)gisRawPoint.x;
							double v_error = gisRawPoint_Foot_y - (double)gisRawPoint.y;
							if(normof(u_error,v_error) < MAX_GISERROR_NORM)
							{
								double azimuth = - dst_Ins.attitude[2]*PI/180.0;
								double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
								double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
								double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
								Error_azimuth = fmod(Error_azimuth+360.0,360.0);

								fprintf(stdout,"error %lf,%lf\n",u_error,v_error);

								C_GisErrorFrame newframe(GisFrame.timestamp,East_error,North_error,Error_azimuth);
								/*？？？？？？？
								if(GisError.LastFrameExist == true)
								{
									double East_error_delta = East_error - GisError.LastFrame.error_east;
									double North_error_delta = North_error - GisError.LastFrame.error_north;
									double normof_delta = normof(East_error_delta,North_error_delta);
									if(normof_delta < Mid.getLaneWidth())
									{
										GisError.addGisErrorFrame(newframe);
									}
								}
								*/
								GisError.addGisErrorFrame(newframe,false);
								double estimate_E_error;
								double estimate_N_error;
								GisError.estimateError(estimate_E_error,estimate_N_error);
								if(GisError.estimate_good > 0 && GisError.InitComplited == true)
								{
									GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
								}
							}
						}
					}
				}
			}
		}
		return;
	}

	void C_LocalMap::SetGisErrorToHere()
	{
		tempDraw.init();
		//set local error and add error sampe to GISERROR
		//INS
		C_InsFrame dst_Ins;
		if(InsFrameHis.GetLastInsFrame(dst_Ins) == true)
		{
			//Gis
			Point gisRawPoint;
			double gisRawPoint_angle3K;
			if( GisFrame.getRawPoint(gisRawPoint,gisRawPoint_angle3K) == true)
			{
				Point gisInitPoint;
				double gisInitPoint_angle3K;
				GisFrame.initialAlignedData().getRawPoint(gisInitPoint,gisInitPoint_angle3K);
				tempDraw.add(Point(gisInitPoint.x,gisInitPoint.y),Scalar(255,0,0),20);

				double u_error = 1500.0 - (double)gisRawPoint.x;
				double v_error = 1000.0 - (double)gisRawPoint.y;

				double azimuth = - dst_Ins.attitude[2]*PI/180.0;
				double East_error  = u_error*cos(azimuth) - v_error*sin(azimuth);
				double North_error = u_error*sin(azimuth) + v_error*cos(azimuth);
				double Error_azimuth = atan2(East_error,North_error)*180.0/PI;
				Error_azimuth = fmod(Error_azimuth+360.0,360.0);

				fprintf(stdout,"error %lf,%lf\n",u_error,v_error);
				GisError.Clear_GisErrorSample();
				for(int i=0;i<GISERROR_FRAME_HIS_LEN;i++)
				{
					C_GisErrorFrame newframe(GisFrame.timestamp,East_error,North_error,Error_azimuth);
					GisError.addGisErrorFrame(newframe,false);
				}
				double estimate_E_error;
				double estimate_N_error;
				GisError.estimateError(estimate_E_error,estimate_N_error);
				if( GisError.estimate_good > 0 )
				{
					GisFrame.setInitError(GisError.Error_E_estimated,GisError.Error_N_estimated);
				}
			}
		}
	}

	CvPoint C_LocalMap::convENUto3000(double x, double y, double x0, double y0, double azimuth)
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

	CvPoint2D64f C_LocalMap::conv3000toENU(int x, int y, double x0, double y0, double azimuth)
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
