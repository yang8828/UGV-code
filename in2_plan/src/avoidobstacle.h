#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <list>

#define PI 3.14159265358
#define TargetPointDiss 37         //TargetPointDiss 14.8m = 37 * 0.4m

CvPoint roadpts_laser[301];
float FAN_INV[721]; //ÕÏ°­Îïµã·ŽÉšÃèÉÈÇøŸØÕó
float FAN_INV_3000[721]; //ÕÏ°­Îïµã·ŽÉšÃèÉÈÇøŸØÕó(3000×ø±êÏµ)
CvPoint Candid_Hope[38][64]; //Œ«ÏÞÕÏ°­ÎïµÄË÷ÒýŸØÕó
int planning_level=0;//¹æ»®²ãÊýÓÃÓÚËÙ¶ÈÏÞÖÆ
CvPoint2D32f PATH_PLANNING_POINT[38];//Â·Ÿ¶¹æ»®µã
int PATH_SECT[38];
CvPoint2D32f fused_scan[1000];
int RoamDirection = 0;
int gps_x = 0;
int gps_y = 0;
long speed_laser;
extern bool in_shi_gong;
long int shi_gong_tmr = 0;
double zk_angle = 0.0;
int min_angle = 360.0;
int max_angle = 360.0;

void processLaser(CvPoint* roadpts)
{
	if (in_shi_gong) shi_gong_tmr++;
	else 		 shi_gong_tmr = 0;
	
	zk_angle = 1.0 * shi_gong_tmr;
	if (zk_angle > 35.0) zk_angle = 35.0;
	
	min_angle = 340 - zk_angle * 2.0;
	max_angle = 380 + zk_angle * 2.0;
	
	min_angle = 273;
	max_angle = 453;

	///////////////GPSÒýµŒµã///////////////////////////
	float GPS_X = roadpts[250].x;
	float GPS_Y = roadpts[250].y;
	int NearCarIndex3 = 0;
	double distance_min3 = 100000000.0;
	double distance3;
	int corner;	
	double angle_int;
	double pointangle_diff2;
	double distance_corner;

	for(int i=0;i<301;i++)
	{
		distance3 = sqrt( (1500.0-(double)roadpts[i].x)*(1500.0-(double)roadpts[i].x) + 
				(880.0-(double)roadpts[i].y)*(880.0-(double)roadpts[i].y)
			      );
		if(distance3 < distance_min3)
		{
			NearCarIndex3 = i;
			distance_min3 = distance3;
		}
	}

	if(RoamDirection==1)
	{
		gps_x = 1500;
		gps_y = 1500;
		GPS_X = ((float)gps_y)/10.0;
		GPS_Y = 301-((float)gps_x)/10.0;
	}
	else if(RoamDirection==2)
	{
		gps_x = 1400;
		gps_y = 1500;
		GPS_X = ((float)gps_y)/10.0;
		GPS_Y = 301-((float)gps_x)/10.0;
	}
	else
	{
		corner = NearCarIndex3 + TargetPointDiss;

		if(corner>300)
		{
			corner = 300;
		}

		GPS_X = ((float)roadpts[corner].y)/10.0;
		GPS_Y = 301-((float)roadpts[corner].x)/10.0;
		gps_x = roadpts[corner].x;
		gps_y = roadpts[corner].y;
	}					
	////////////////////////////////²âÊÔ//////////////////////////////////
	//GPS_X = 200;
	//GPS_Y = 50;

	/////////////////////////////////////////////////////////////////////


	///////////////////////Œ€¹â±ÜÕÏ¹æ»®//////////////////////////////////////
	//float sector_rslt = 0.5;   //ÉÈÇøœÇ¶È·Ö±æÂÊ
	//³õÊŒ»¯ÕÏ°­Îïµã·ŽÉšÃèÉÈÇøŸØÕó£¬ÓÉÓÚÔÚŸÖ²¿µŒºœµØÍŒÖÐ£¬ŸàÀë³µÌå×ø±êÏµÔ­µã×îÔ¶ŸàÀëÎª£š-151£¬301£©ºÍ£š151£¬301£©Áœžöµã£¬ËùÒÔ³õÊŒ»¯ÉÈÇøŸàÀëÎª255¡£
	for(int i=0;i<721;i++)
	{
		FAN_INV[i] = 255; 
	}
	//³õÊŒ»¯Ìœ²âÉšÃè·¶Î§£¬¹²·ÖÎª£±£¹²ãÌœ²â£¬Œäžô£²Ã×Ò»²ã£¬ŽÓ£²Ã×¿ªÊŒµœ£³£žÃ×œáÊø£šÒÑ»»Ëã³ÉÕ€žñÊý£©
	float SCAN_RANGE[38];

	for(int i=0;i<38;i++)
	{
		SCAN_RANGE[i] = 5.0*(i+2); //¹²·ÖÎª38²ãÌœ²â£¬Œäžô1Ã×Ò»²ã
	}
	float SAFE_LATERAL = 12.0;//Éè¶šºáÏò¿ÉÍš¹ý°²È«ŸàÀëÎª3Ã×£šÒÔ»»Ëã³ÉÕ€žñÊýÎª15£©
	//³õÊŒ»¯Œ«ÏÞÕÏ°­ÎïµÄË÷ÒýŸØÕó£¬Ê¹Æäž³ÖµÈ«Îª0£¬ÐÐÊý±íÊŸµÚŒžžöŒ«ÏÞÕÏ°­Îï£¬µÚÒ»ÁÐ±íÊŸžÃŒ«ÏÞÕÏ°­ÎïµÄÆðÊŒÉÈÇøÐòºÅ£¬µÚ¶þÁÐ±íÊŸÖÕÖ¹ÉÈÇøÐòºÅ¡£
	for(int i=0;i<38;i++)
		for(int j=0;j<64;j++)
		{
			Candid_Hope[i][j].x = 0;
			Candid_Hope[i][j].y = 0;
		}
	float GPS_Distance = 0; //GPSÂ·µãÓë³µÌå×ø±êÏµÔ­µãÖ®ŒäµÄŸàÀë
	float GPS_Angle = 0; //GPSÂ·µãÓë³µÌå×ø±êÏµµÄœÇ¶È
	int GPS_Sect = 0; //GPSÂ·µãÓë³µÌå×ø±êÏµœÇ¶È¶ÔÓŠµÄÉÈÇøÊý
	//³õÊŒ»¯µŒºœÔ²¹æ»®µÄÂ·µã£¬¹²19žö
	//µŒºœÔ²¹æ»®Â·µãÐèÒªµÄœÇ¶È£¬¹²19žö
	//CvPoint2D32f PATH_PLANNING_POINT[19];
	float PATH_PLANNING_ANGLE[38];
	for(int i=0;i<38;i++)
	{
		PATH_PLANNING_POINT[i].x = 0;
		PATH_PLANNING_POINT[i].y = 0;
		PATH_PLANNING_ANGLE[i] = 0;
	}
	int Forecast_flag = 0; //Â·Ÿ¶¹æ»®×ŽÌ¬±êÖŸÎ»
	float VO_X = 101; //³µÌåÔ­µã×ø±êX
	float VO_Y = 151; //³µÌåÔ­µã×ø±êY

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	// žüÐÂÕÏ°­Îïµã·ŽÉšÃèÉÈÇøŸØÕó
	for(int i=0;i<720;i++)
	{
		float theta = atan2(fused_scan[i].y,fused_scan[i].x);
		int index = (int)((theta*180.0/PI+180.0)*2);
		FAN_INV[index] = sqrt(fused_scan[i].x*fused_scan[i].x+fused_scan[i].y*fused_scan[i].y)/0.2; 
	}
	GPS_Distance = floor(sqrt((GPS_Y - VO_Y)*(GPS_Y - VO_Y)+ (GPS_X - VO_X)*(GPS_X - VO_X))); //ŒÆËãGPSÂ·µãÓë³µÌå×ø±êÏµÔ­µãÖ®ŒäµÄŸàÀë
	GPS_Angle = atan2((GPS_Y - VO_Y), (GPS_X - VO_X))+PI; //ŒÆËãGPSÂ·µã¶ÔÓŠÓÚ³µÌå×ø±êÏµµÄŒ«×ø±êœÇ¶È
	GPS_Sect = round(GPS_Angle / 0.008726646259972);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ŒÆËã¹æ»®µŒºœÔ²Â·Ÿ¶ 
	// ¿ªÊŒÓÉ³µÌå×ø±êÔ­µãœøÐÐËÑË÷Œ«ÏÞÕÏ°­Îï¿ÉÍšÐÐµã
	//int P3_i = 1;   //¹æ»®Â·µãÐòºÅ
	float scan_limited = 0; //³õÊŒ»¯Ìœ²â·¶Î§
	float SAFE_ANGLE = 0; //³õÊŒ»¯°²È«œÇ
	int SAFE_Num = 0; //³õÊŒ»¯°²È«œÇ¶ÔÓŠµÄÉÈÇøÊý
	Forecast_flag = 0; //ÊÇ·ñÆô¶¯Ô€²âµŒºœÔ²µÄ±êÖŸÎ»
	int cand_i[38];//¶ÔÓŠÉšÃè²ãµÄŒ«ÏÞÕÏ°­ÎïÊý
	int Hoping;
	planning_level=0;
	for(int i=0;i<38;i++)
	{
		scan_limited = SCAN_RANGE[i];   //Éè¶šÌœ²â·¶Î§
		SAFE_ANGLE = 2.0 * asin(SAFE_LATERAL / (2.0 * scan_limited)); //ÖÐŒä¹ý³Ì£¬ŒÆËãÔÚµ±Ç°Ìœ²â·¶Î§ÄÚ¶ÔÓŠ¿ÉÍš¹ýµÄ°²È«œÇ¶È
		SAFE_Num = ceil(SAFE_ANGLE / 0.008726646259972); //ŒÆËã³ÉÉÈÇøÊý
		Hoping = 0;   //±íÊŸÊÇ·ñÒÑœøÈëµœŒ«ÏÞÕÏ°­ÎïµÄÅÐ¶ÏÖÐ£¬1±íÊŸÒÑ¿ªÊŒ£¬0±íÊŸÎŽ¿ªÊŒ»òÒÑÍË³ö
		cand_i[i] = 0;   //±íÊŸµÚŒžžöºòÑ¡Œ«ÏÞÕÏ°­Îï
		for (int fan_i = min_angle; fan_i <= max_angle-1; fan_i++)  //fan_iŽú±íÉÈÇøÐòºÅ£¬¿ªÊŒËÑË÷Œ«ÏÞÕÏ°­Îï£¬ÓÉÓÚÊÇ¹æ»®µŒºœÔ²£¬ËùÒÔÖ»ŒÆËã³µÌåÇ°·œ0-180¶ÈµÄ»·Ÿ³ÐÅÏ¢£¬Õâžö¿ÉÒÔÌÖÂÛ
		{
			if (FAN_INV[fan_i] > scan_limited)   //ÅÐ¶ÏÊÇ·ñÂú×ãŒ«ÏÞÕÏ°­ÎïµÄŸàÀëÌõŒþ
			{
                		if (Hoping == 0)    //Èç¹ûÂú×ã£¬ÅÐ¶ÏÊÇ·ñÊÇŒ«ÏÞÕÏ°­ÎïµÄÆðÊŒµã
				{
					Candid_Hope[i][cand_i[i]].x = fan_i;    //Èç¹ûÊÇ£¬œ«ÆðÊŒµãŒÇÂŒÏÂÀŽ
					Hoping = 1;    //œ«±êÖŸÎ»ÖÃÎª1£¬±íÊŸÒÑœøÈëÅÐ¶ÏŒ«ÏÞÕÏ°­Îï¹ý³ÌÖÐ
				}
				if ((fan_i == max_angle-1) && (Hoping == 1))    //ÅÐ¶Ï×îºóÒ»žöÉÈÇøµÄŒ«ÏÞÕÏ°­ÎïÇé¿ö
				{
					Hoping = 0;
					if ( (fan_i - Candid_Hope[i][cand_i[i]].x) >= SAFE_Num)
					{
						Candid_Hope[i][cand_i[i]].y = fan_i;
						cand_i[i] = cand_i[i] + 1;
                        
					}
				}
			}
			else //Èç¹û²»Âú×ãŒ«ÏÞÕÏ°­ÎïŸàÀëÌõŒþ
			{
				if (Hoping == 1)    //ÅÐ¶ÏÊÇ·ñ±êÖŸÎ»Îª1£¬ÒòÎªÕâ¿ÉÒÔËµÃ÷ÊÇ·ñÊÇÉÏÒ»žöŒ«ÏÞÕÏ°­ÎïµÄÖÕÖ¹µã
				{
					Hoping = 0;     //Èç¹ûÊÇ£¬œ«±êÖŸÎ»ÖÃÎª0£¬ÍË³öŒ«ÏÞÕÏ°­ÎïÅÐ¶Ï¹ý³Ì
					if ( (fan_i - 1 - Candid_Hope[i][cand_i[i]].x) >= SAFE_Num) //ÅÐ¶ÏÕâžöŒ«ÏÞÕÏ°­ÎïÊÇ·ñ¿ÉÍšÐÐ£¬ŒŽÊÇ·ñ¿íÓÚºáÏò°²È«Íš¹ýŸàÀë
					{
						Candid_Hope[i][cand_i[i]].y = (fan_i - 1);   //Èç¹ûÊÇ£¬œ«Ç°Ò»ËÑË÷ÉÈÇøºÅž³žøŒ«ÏÞÕÏ°­ÎïÖÕÖ¹Öµ
						cand_i[i] = cand_i[i] + 1;                      //Œ«ÏÞÕÏ°­ÎïÊýŒÓ1
					}
				}
			}
		}
		planning_level = i;
		//±ŸŽÎËÑË÷œáÊøÌõŒþ
		if(cand_i[i] < 1)
		{
			Forecast_flag = 1;                        //Forecast_flag = 1±íÊŸÂ·µãËÑË÷ÒòÎªÃ»ÓÐÕÒµœŒ«ÏÞÕÏ°­Îï¶øÖÐ¶Ï£¬ÐèÒªÖØÐÂ¹æ»®
			i = i - 1; 
			planning_level = i;
			break;
		}
		//if(GPS_Distance < scan_limited)              //ÔÝÊ±ŒÓÒ»žöœáÊøÌõŒþ£¬±íÊŸÂ·µãŸàÀëÐ¡ÓÚÌœ²â·¶Î§£¬¹æ»®µŒºœÔ²ÈÎÎñœáÊø¡£
		int temp__ = abs(GPS_X-VO_X)>20?abs(GPS_X-VO_X):20;
		
		if(temp__ < scan_limited)
		{
			Forecast_flag = 2;                       //Forecast_flag = 2±íÊŸÂ·µãËÑË÷ÒòÎªµœŽïÄ¿±êµã°ëŸ¶·¶Î§¶øœáÊø£¬²»ÓÃŒÌÐøÔ€²â¹æ»®ÁË
			break;
		}
		
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	int INV_Num = planning_level;   //±ŸŽÎ¹æ»®ËùÍê³ÉµÄÉšÃè²ãÊý
	//¶Ô±ŸÖÜÆÚÌœ²âÉšÃè·¶Î§ÄÚËùµÃµœµÄŒ«ÏÞÕÏ°­ÎïœøÐÐŽŠÀí£¬Íš¹ýÓÉÔ¶Œ°œüµÄ·œÊœœøÐÐ¹æ»®£¬ŒŽ×îÏÈŒÆËã×îÔ¶Â·µã£¬ÒÀŽÎÏò»ØËÑË÷
	for(int INV_i=INV_Num; INV_i>=0; INV_i--)
	{
		scan_limited = SCAN_RANGE[INV_i];//Éè¶šÌœ²â·¶Î§
		SAFE_ANGLE  = 2.0 * asin(SAFE_LATERAL / (2.0 * scan_limited)); //ÖÐŒä¹ý³Ì£¬ŒÆËãÔÚµ±Ç°Ìœ²â·¶Î§ÄÚ¶ÔÓŠ¿ÉÍš¹ýµÄ°²È«œÇ¶È
		SAFE_Num = ceil(SAFE_ANGLE /  0.008726646259972);   //ŒÆËã³ÉÉÈÇøÊý
		int delta_L = 721;
		int delta_R = 721;
		int delta_Num_L = 0;
		int delta_Num_R = 0;
		int Road_find = 0;  //Ñ°ÕÒÂ·µã±êÖŸÎ»£¬ÖµÎª1±íÊŸÒÑÕÒµœ
		float Fan_Width_K = 0.0; //¹æ»®µãËùÔÚÉÈÇø±È°²È«¿í¶ÈµÄ±¶Êý
		float Safe_Width_K = 0.0;
		//Èç¹ûÊÇ×îÔ¶µÄŒ«ÏÞÕÏ°­Îï£¬ÒÔGPSÂ·µãÎªÄ¿±êµãœøÐÐ¹æ»®
		
		double max_fan_k = 3.2;
		double min_fan_k = 1.2;
		
		if (INV_i == INV_Num)
		{
			if (GPS_Sect <min_angle)     //Èç¹ûžø¶šµÄÂ·µãÔÚ³µÌå×óºó·œ£¬
			{//ÔòÖ»ÐèÒªÕÒµœµÚÒ»žöºòÑ¡Œ«ÏÞÕÏ°­ÎïÉÈÇø£¬ÓÉ×ó±ßÔµ£¬ŒŽŽËŒ«ÏÞÕÏ°­ÎïµÚÒ»žöµã£¬ŒÓÉÏ1/2µÄÔÚŽËÌœË÷·¶Î§ÄÚµÄ°²È«¿í¶È
 				PATH_SECT[INV_i] = 0;   //Â·µãËùÔÚÉÈÇøºÅÎª0£»
                Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
				if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
				if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
				Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5);
				PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].x + Safe_Width_K * (SAFE_Num / 2.0)); 
                                 
			}
			else if (GPS_Sect >= max_angle)   //Èç¹ûžø¶šµÄÂ·µãÔÚ³µÌåÓÒºó·œ£¬
			{//ÔòÖ»ÐèÒªÕÒµœ×îºóÒ»žöºòÑ¡Œ«ÏÞÕÏ°­ÎïÉÈÇø£¬ÓÉÓÒ±ßÔµ£¬ŒŽŽËŒ«ÏÞÕÏ°­ÎïµÚ¶þžöµã£¬ŒõÈ¥1/2µÄÔÚŽËÌœË÷·¶Î§ÄÚµÄ°²È«¿í¶È
				PATH_SECT[INV_i] = (cand_i[INV_i] - 1);   //Â·µãËùÔÚÉÈÇøºÅÎª1£»
				Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
				if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
				if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
				Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5); 				
				PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Safe_Width_K * (SAFE_Num / 2.0));
                                  
			}
			else
			{
				for(int Ob_L_i = 0; Ob_L_i < cand_i[INV_i]; Ob_L_i++)
					//Ob_L_i±íÊŸŒ«ÏÞÕÏ°­ÎïµÄÐòºÅ£¬ËÑË÷ÀëÂ·µã×îœüµÄŒ«ÏÞÕÏ°­ÎïµÄ±ßÔµµã
				{
					int delta_GC_L = GPS_Sect - Candid_Hope[INV_i][Ob_L_i].x; 
					//delta_GC_L±íÊŸÂ·µãÓëŒ«ÏÞÕÏ°­Îï×ó±ßÔµµã£šŒŽÆðÊŒµã£©µÄÉÈÇøŒäžôÊýŸø¶ÔÖµ
					int delta_GC_R = GPS_Sect - Candid_Hope[INV_i][Ob_L_i].y; 
					//delta_GC_R±íÊŸÂ·µãÓëŒ«ÏÞÕÏ°­ÎïÓÒ±ßÔµµã£šŒŽÖÕÖ¹µã£©µÄÉÈÇøŒäžôÊýŸø¶ÔÖµ
					if ((delta_GC_L > 0) && (delta_GC_R < 0) && (delta_GC_L > (SAFE_Num / 2.0)) && (abs(delta_GC_R) > (SAFE_Num / 2.0)))
						//Èç¹ûÂ·µãÔÚŒ«ÏÞÕÏ°­ÎïÄÚ£¬²¢ÇÒŸàÆðÊŒµãÓëÖÕÖ¹µã¶ŒŽóÓÚ°²È«¿í¶È£¬
					{
						PATH_PLANNING_ANGLE[INV_i] = GPS_Sect;   //Ôò¹æ»®Â·µãÎªGPS_Sect
                        PATH_SECT[INV_i] = Ob_L_i;   //Â·µãËùÔÚÉÈÇøºÅÎªOb_L_i£»
						Road_find = 1;//Ñ°ÕÒÂ·µã±êÖŸÎ»£¬ÖµÎª1±íÊŸÒÑÕÒµœ
						break;
					}
					else
					{
						Road_find = 0;
					}
					delta_GC_L = abs(delta_GC_L);
					if(delta_L >= delta_GC_L)//œøÐÐ×îÐ¡ÖµËÑË÷
					{
						delta_L = delta_GC_L;//delta_L±íÊŸdelta_GC_LÖÐ×îÐ¡Öµ
						delta_Num_L = Ob_L_i;//delta_Num_L±íÊŸ×îÐ¡Öµ¶ÔÓŠµÄŒ«ÏÞÕÏ°­ÎïÐòºÅ
					}
					delta_GC_R = abs(delta_GC_R);
					if (delta_R >= delta_GC_R)  //œøÐÐ×îÐ¡ÖµËÑË÷
					{
						delta_R = delta_GC_R;   //delta_R±íÊŸdelta_GC_RÖÐ×îÐ¡Öµ
						delta_Num_R = Ob_L_i;   //delta_Num_R±íÊŸ×îÐ¡Öµ¶ÔÓŠµÄŒ«ÏÞÕÏ°­ÎïÐòºÅ
					}
				}
				if (Road_find == 1) //Èç¹ûÂ·µãÔÚŒ«ÏÞÕÏ°­ÎïÄÚ£¬²¢ÇÒŸàÆðÊŒµãÓëÖÕÖ¹µã¶ŒŽóÓÚ°²È«¿í¶È£¬
					Road_find = 0;
				else if (delta_L >= delta_R)   //±ÈœÏ×óÓÒ±ßÔµµãµÄÉÈÇøŒäžôÊýÖµ£¬Èç¹ûÓÒµãŒäžôÐ¡£¬
				{
                    PATH_SECT[INV_i] = delta_Num_R;   //Â·µãËùÔÚÉÈÇøºÅÎªdelta_Num_R£»
					Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
					if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
					if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
					Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5); 					

					PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Safe_Width_K  * (SAFE_Num / 2.0));    //ÔòÓÃÓÒµãŒõÈ¥°²È«¿í¶ÈµÄ1/2

				}
				else
				{
                    PATH_SECT[INV_i] = delta_Num_L;   //Â·µãËùÔÚÉÈÇøºÅÎªdelta_Num_L£»
					Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
					if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
					if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
					Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5);					
					PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].x + Safe_Width_K * (SAFE_Num / 2.0));    //·ñÔò£¬ÓÃ×óµãŒÓÉÏ°²È«¿í¶ÈµÄ1/2

	             }
			}
		}
		else //Èç¹û²»ÊÇ×îÔ¶µÄŒ«ÏÞÕÏ°­Îï£¬ÔòÒÔÔ¶Ò»²ã¹æ»®ºÃµÄÂ·µãÎªÄ¿±êµãœøÐÐ¹æ»®
		{
			int Further_Sect = PATH_PLANNING_ANGLE[INV_i + 1]; //ÒÔÔ¶Ò»²ãÂ·µã·œÏòÎªÄ¿±ê£¬œøÐÐ¹æ»®
			 if (Further_Sect <min_angle)    //Èç¹ûžø¶šµÄÂ·µãÔÚ³µÌå×óºó·œ£¬
			 {//ÔòÖ»ÐèÒªÕÒµœµÚÒ»žöºòÑ¡Œ«ÏÞÕÏ°­ÎïÉÈÇø£¬ÓÉ×ó±ßÔµ£¬ŒŽŽËŒ«ÏÞÕÏ°­ÎïµÚÒ»žöµã£¬ŒÓÉÏ1/2µÄÔÚŽËÌœË÷·¶Î§ÄÚµÄ°²È«¿í¶È
                PATH_SECT[INV_i] = 0;   //Â·µãËùÔÚÉÈÇøºÅÎª0£» 				
				Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
				if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
				if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
				Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5); 				 
				PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].x + Safe_Width_K * (SAFE_Num / 2.0)); 
                  
			 }
			 else if (Further_Sect >= max_angle)    //Èç¹ûžø¶šµÄÂ·µãÔÚ³µÌåÓÒºó·œ£¬
			 {//ÔòÖ»ÐèÒªÕÒµœ×îºóÒ»žöºòÑ¡Œ«ÏÞÕÏ°­ÎïÉÈÇø£¬ÓÉÓÒ±ßÔµ£¬ŒŽŽËŒ«ÏÞÕÏ°­ÎïµÚ¶þžöµã£¬ŒõÈ¥1/2µÄÔÚŽËÌœË÷·¶Î§ÄÚµÄ°²È«¿í¶È
 				PATH_SECT[INV_i] = (cand_i[INV_i] -1);   //Â·µãËùÔÚÉÈÇøºÅÎªcand_i[INV_i] -1£» 				
				Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
				if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
				if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
				Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5);  
				PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Safe_Width_K * (SAFE_Num / 2.0));   
                                
			 }
			 else
			 {
				for(int Ob_L_i = 0; Ob_L_i<cand_i[INV_i]; Ob_L_i++) //Ob_L_i±íÊŸŒ«ÏÞÕÏ°­ÎïµÄÐòºÅ£¬ËÑË÷ÀëÂ·µã×îœüµÄŒ«ÏÞÕÏ°­ÎïµÄ±ßÔµµã
				{
					int delta_GC_L = Further_Sect - Candid_Hope[INV_i][Ob_L_i].x; //delta_GC_L±íÊŸÂ·µãÓëŒ«ÏÞÕÏ°­Îï×ó±ßÔµµã£šŒŽÆðÊŒµã£©µÄÉÈÇøŒäžôÊýŸø¶ÔÖµ
					int delta_GC_R = Further_Sect - Candid_Hope[INV_i][Ob_L_i].y;//delta_GC_R±íÊŸÂ·µãÓëŒ«ÏÞÕÏ°­ÎïÓÒ±ßÔµµã£šŒŽÖÕÖ¹µã£©µÄÉÈÇøŒäžôÊýŸø¶ÔÖµ
					if ((delta_GC_L > 0) && (delta_GC_R < 0) && (delta_GC_L > (SAFE_Num / 2.0)) && (abs(delta_GC_R) > (SAFE_Num / 2.0))) 
						//Èç¹ûÂ·µãÔÚŒ«ÏÞÕÏ°­ÎïÄÚ£¬²¢ÇÒŸàÆðÊŒµãÓëÖÕÖ¹µã¶ŒŽóÓÚ°²È«¿í¶È£¬
					{
						PATH_PLANNING_ANGLE[INV_i] = Further_Sect; //Ôò¹æ»®Â·µãÎªFurther_Sect
                                                PATH_SECT[INV_i] = Ob_L_i;   //Â·µãËùÔÚÉÈÇøºÅÎªOb_L_i£»
						Road_find = 1;   //Ñ°ÕÒÂ·µã±êÖŸÎ»£¬ÖµÎª1±íÊŸÒÑÕÒµœ
						break;
					}
					else
					{
						Road_find = 0;
					}
					delta_GC_L = abs(delta_GC_L);
					if (delta_L >= delta_GC_L)  //œøÐÐ×îÐ¡ÖµËÑË÷
					{
						delta_L = delta_GC_L;   //delta_L±íÊŸdelta_GC_LÖÐ×îÐ¡Öµ
						delta_Num_L = Ob_L_i;   //delta_Num_L±íÊŸ×îÐ¡Öµ¶ÔÓŠµÄŒ«ÏÞÕÏ°­ÎïÐòºÅ
					}
					delta_GC_R = abs(delta_GC_R);
					if (delta_R >= delta_GC_R)  //œøÐÐ×îÐ¡ÖµËÑË÷
					{
						delta_R = delta_GC_R;   //delta_R±íÊŸdelta_GC_RÖÐ×îÐ¡Öµ
						delta_Num_R = Ob_L_i;   //delta_Num_R±íÊŸ×îÐ¡Öµ¶ÔÓŠµÄŒ«ÏÞÕÏ°­ÎïÐòºÅ
					}
				}
				if (Road_find == 1) //Èç¹ûÂ·µãÔÚŒ«ÏÞÕÏ°­ÎïÄÚ£¬²¢ÇÒŸàÆðÊŒµãÓëÖÕÖ¹µã¶ŒŽóÓÚ°²È«¿í¶È£¬
					Road_find = 0;
				else if (delta_L >= delta_R)  //±ÈœÏ×óÓÒ±ßÔµµãµÄÉÈÇøŒäžôÊýÖµ£¬Èç¹ûÓÒµãŒäžôÐ¡£¬
				{	
					PATH_SECT[INV_i] = delta_Num_R;   //Â·µãËùÔÚÉÈÇøºÅÎªdelta_Num_R£»
					Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
					if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
					if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
					Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5);  
					PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Safe_Width_K * (SAFE_Num / 2.0));  //ÔòÓÃÓÒµãŒõÈ¥°²È«¿í¶ÈµÄ1/2
					
				}
				else
				{	
					PATH_SECT[INV_i] = delta_Num_L;   //Â·µãËùÔÚÉÈÇøºÅÎªdelta_Num_L£»
					Fan_Width_K = (float)(Candid_Hope[INV_i][PATH_SECT[INV_i]].y - Candid_Hope[INV_i][PATH_SECT[INV_i]].x) / (float)(SAFE_Num);
					if(Fan_Width_K > max_fan_k) Fan_Width_K = max_fan_k;
					if(Fan_Width_K < min_fan_k) Fan_Width_K = min_fan_k;
					Safe_Width_K = min_fan_k + ((Fan_Width_K - min_fan_k) * 0.5);  
					PATH_PLANNING_ANGLE[INV_i] = round(Candid_Hope[INV_i][PATH_SECT[INV_i]].x + Safe_Width_K * (SAFE_Num / 2.0));  //·ñÔò£¬ÓÃ×óµãŒÓÉÏ°²È«¿í¶ÈµÄ1/2

				}
			 }
		}
	}
	//ÓÉœüŒ°Ô¶ÔÙŒì²âÒ»ÏÂÉÈÇø£¬È·±£ËùÓÐµÄ¹æ»®µã¶ŒÔÚÍ¬Ò»žöÉÈÇø//
	int n2f_i = 0;         //ÓÉœüŒ°Ô¶×îÖÕÂ·µãµÄžöÊý
	float n2f_angle_l = 0;
	float n2f_angle_r = 0;
	for (n2f_i = 0;  n2f_i < (INV_Num - 1); n2f_i++)
	{
		scan_limited = SCAN_RANGE[n2f_i];   //Éè¶šÌœ²â·¶Î§
		n2f_angle_l = PATH_PLANNING_ANGLE[n2f_i + 1] - Candid_Hope[n2f_i][PATH_SECT[n2f_i]].x;
		n2f_angle_r = PATH_PLANNING_ANGLE[n2f_i + 1] - Candid_Hope[n2f_i][PATH_SECT[n2f_i]].y;
		if ((n2f_angle_l >= 0) && (n2f_angle_r <= 0))
		{
			float P3X = VO_X + scan_limited * sin((PATH_PLANNING_ANGLE[n2f_i] - 180.0) * PI / (2.0 * 180));
			float P3Y = VO_Y - scan_limited * cos((PATH_PLANNING_ANGLE[n2f_i] - 180.0) * PI / (2.0 * 180));
			PATH_PLANNING_POINT[n2f_i].x = P3X;
			PATH_PLANNING_POINT[n2f_i].y = P3Y;
		}
		else
		{
			float P3X = VO_X + scan_limited * sin((PATH_PLANNING_ANGLE[n2f_i] - 180.0) * PI / (2.0 * 180));
			float P3Y = VO_Y - scan_limited * cos((PATH_PLANNING_ANGLE[n2f_i] - 180.0) * PI / (2.0 * 180));
			PATH_PLANNING_POINT[n2f_i].x = P3X;
			PATH_PLANNING_POINT[n2f_i].y = P3Y;
			break;
		}
	}
	if (n2f_i==0)
		planning_level = 0;
	else
		planning_level = n2f_i - 1;

////////////////////////////////////////////////////////////////////////////////////////////////////

	///calculate road points from laser
	for(int i=0;i<301;i++)
	{
		roadpts_laser[i].x = 0;
		roadpts_laser[i].y = 0;
	}
	//axis convert(from planning axis to 3000 axis)
	int planning_pt_count = 0;
	CvPoint2D32f PATH_PLANNING_POINT_3000[301];
	for (int i=0;i<38;i++)
	{
		if (PATH_PLANNING_POINT[i].x !=0 || PATH_PLANNING_POINT[i].y != 0)
		{
			PATH_PLANNING_POINT_3000[i].x = 3000.0 - PATH_PLANNING_POINT[i].y*10;
			PATH_PLANNING_POINT_3000[i].y = PATH_PLANNING_POINT[i].x*10;
			if(i==37)//20121022
			{
				planning_pt_count = 38;//20121022
			}
		}
		else
		{
			planning_pt_count = i;
			break;
		}
	}
	// calculate road points from laser
	int pts_count = 0;
	if (planning_pt_count==0)//no planning points
	{
		for (int i=0;i<301;i++)
		{
			if(i<=88)
			{
				roadpts_laser[i].x = 1500;
				roadpts_laser[i].y = 10*i;
			}
			else
			{



				roadpts_laser[i].x = 1500;
				roadpts_laser[i].y = 880;
			}
		}
	}
	else
	{
		//calculate first line segment(from bottom to car center)
		for (int i=0; i<88;i++)
		{
			roadpts_laser[i].x = 1500;//(int)PATH_PLANNING_POINT_3000[0].x;
			roadpts_laser[i].y = 10*i;
			pts_count++;
		}
		//calculate rest line segments
		int fst_count = pts_count;
		int sec_count = 0;
		float x1;
		float y1;
		float x2;
		float y2;
		float norm;
		int lcount;
		for (int i=0;i<planning_pt_count;i++)
		{
			if(i==0)//from car center to first planning point
			{
				x1 = 1500;
				y1 = 880;
				x2 = PATH_PLANNING_POINT_3000[0].x;
				y2 = PATH_PLANNING_POINT_3000[0].y;

				norm = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
				lcount = (int)(norm/10.0);
				for (int j=0;j<=lcount;j++)
				{
					if (pts_count>=301)
					{
						break;
					}
					roadpts_laser[fst_count+j].x = (int)(x1+j*(x2-x1)/lcount);
					roadpts_laser[fst_count+j].y = (int)(y1+j*(y2-y1)/lcount);
					pts_count++;
				}
				sec_count = pts_count;
			}
			else//from i-1 point to i-th point 
			{
				x1 = PATH_PLANNING_POINT_3000[i-1].x;
				y1 = PATH_PLANNING_POINT_3000[i-1].y;
				x2 = PATH_PLANNING_POINT_3000[i].x;
				y2 = PATH_PLANNING_POINT_3000[i].y;
				norm = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
				lcount = (int)(norm/10.0);
				for (int j=0;j<=lcount;j++)
				{
					if (pts_count>=301)
					{
						break;
					}
					roadpts_laser[sec_count+j].x = (int)(x1+j*(x2-x1)/lcount);
					roadpts_laser[sec_count+j].y = (int)(y1+j*(y2-y1)/lcount);
					pts_count++;
				}
				sec_count = pts_count;
			}
		}
		int end_count = pts_count;
		if (pts_count < 301)
		{
			for (int i=0;i<301-end_count;i++)
			{
				roadpts_laser[end_count+i].x = roadpts_laser[end_count-1].x;
				roadpts_laser[end_count+i].y = roadpts_laser[end_count-1].y;
				pts_count++;
			}
		}
	}
	//speed_laser = (long)(5000 + planning_level*300);
	//if(speed_laser>5000)
	//	speed_laser = 5000;
	speed_laser = 5800;
}
