/*
 * C_LocalMap_MacroParam.h
 *
 *  Created on: 2013-10-11
 *      Author: greensky
 */

#ifndef C_LOCALMAP_MACROPARAM_H_
#define C_LOCALMAP_MACROPARAM_H_

#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>

// Parameter
#define DELAY_DEAD_NUM   20
#define HIS_EDGE_TS_THRE 500 // ms
#define HIS_GIS_ANG_THRE 8

// Remember To Modifry Here!!!!!!!!!!!!!!!
#define VERSION_SUB_URBAN 0
#define VERSION_URBAN 1
#define GIS_AND_HIS


#define USING_SCANINFOV2 1
#define USING_SCANINFOBG 0

#define INS_FRAME_HIS_LEN 50	// C_InsFrameHis records for (INS_FRAME_LIST_LEN / Ins_Sampling_Frequence) second(s)

#define DEFAULT_LANE_WIDTH 175

#define MAX_LINE_HIS_LEN 15
#define MAX_SCAN_HIS_LEN 15		// C_FrameHistory records for last MAX_SCAN_HIS_LEN ScanInfoFrame

#define MAX_LINE_NUM 20
#define MAX_CURVE_MODEL_NUM 5
#define MAX_LINEAR_MODEL_NUM 5
#define MAX_STOPLINE_MODEL_NUM 5

#define MODEL_BELIEFE_THRESHOLD 100

#define EAGE_THRESHOLD_1 8
#define EAGE_THRESHOLD_2 10
#define LIKELIEST_LINEAR_MODEL_THREASHOLD_1 5
#define LIKELIEST_LINEAR_MODEL_THREASHOLD_ANGLE 15.0
//Standard Lane Width varies from 3m to 3.5m
//Lane Width near Building #6 is about 4.5m
#define MIN_LANE_WIDTH 125
#define MAX_LANE_WIDTH 275

#define LIKELIEST_STOPLINE_MODEL_THREASHOLD_1 150
#define LIKELIEST_STOPLINE_MODEL_THREASHOLD_2 45
#define LIKELIEST_STOPLINE_MODEL_THREASHOLD_3 5
#define LIKELIEST_STOPLINE_MODEL_THREASHOLD_Angel 5.0

#define FITNESS_THRESHOLD 75

//#define MAX_GISERROR_NORM (20.0*50) //20.0 m
#define MAX_GISERROR_NORM (30.0*50) //30.0 m

#if VERSION_SUB_URBAN
	#define REGION_GROWING_MAX_DIST 0.4
#endif
#if VERSION_URBAN
	#define REGION_GROWING_MAX_DIST 0.35
#endif

#define GISERROR_ERRORVECTOR_WEIGHT_RADIO 5.0


#define PI 3.14159265359
#define LOW_LVL   8
#define HIGH_LVL  9
#define SEND_LOW_LVL   18
#define SEND_HIGH_LVL  19

#define PRINT_DEBUG_LANE 1
#define PRINT_DEBUG_REGIONGROWING 0
#define PRINT_DEBUG_GISERROR 0

const cv::Scalar ScanInfoColor(0,180,0);
const cv::Scalar GisPathColor(180,0,220);
const cv::Scalar LinearSetColor(0,0,250);
const cv::Scalar LinearSampleColor(180,0,0);
const cv::Scalar StoplineSetColor(0,200,250);
const cv::Scalar StoplineSampleColor(180,0,0);
const cv::Scalar InsInfoColor(20,20,20);


#endif /* C_LOCALMAP_MACROPARAM_H_ */
