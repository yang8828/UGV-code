/*
Ver 2.0
For Urban and SubUrban!!
Remember to Set the Define in Param!!
*/

#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <termios.h>
#include <in2_localmap/C_ServerTime.h>


#include "in2_localmap/C_LaneModel_002.h"
#include "in2_localmap/C_BasicOp.h"

using namespace std;
using namespace cv;
#define NODE "localmap_node"

/*
3000-system: 1 unit =0.02m, x = right, y = up

*/
C_ServerTime SystemTime;

int main(int argc, char** argv)
{
	cout << "// ****** in2_localmap ****** //" << endl;
	ros::init(argc, argv, NODE);
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");
  	
	C_LocalMap my_localmap(node, priv_nh);
	///ros::spin();
	cvStartWindowThread();
	SystemTime.C_Init();

	Mat image;
	image.create(3000,3000,CV_8UC3);
	namedWindow( "C_LocalMap", 0 );
	cout << "init complete!" << endl;
	ros::Rate loop_rate(5);
	while(ros::ok())
	{
		C_InsFrame now;
		my_localmap.InsFrameHis.GetLastInsFrame(now);
		image.setTo(Scalar(180,180,180));
		my_localmap.CostMap.DrawMap(image);
		my_localmap.ScanFrameHis.Draw(image,now,50);

		my_localmap.GisFrame.DrawRaw(image);
		my_localmap.GisFrame.DrawInitialAligned(image,Scalar(233,11,131));
		my_localmap.GisFrame.DrawAligned(image,Scalar::all(0));

		my_localmap.LaneModel.LinearSet.DrawLikeliestModel(image,now,my_localmap.GisFrame.laneNum);
		my_localmap.LaneModel.DrawLiearSample(image,now);
		my_localmap.LaneModel.StoplineSet.DrawLikeliestModel(image,now);
		my_localmap.LaneModel.DrawStoplineSample(image,now);

		my_localmap.InsFrameHis.DrawCompass(image);
		my_localmap.GisFrame.DrawAngle(image);
		my_localmap.GisError.DrawErrorVactor(image,now);
		rectangle(image,Point(1535,1910),Point(1460,2060),Scalar(0,0,0),4);
		my_localmap.DrawTimeStamp(image);
		my_localmap.tempDraw.Draw(image);

		my_localmap.RoadEdgeFrame.drawRoadEdge(image);
		int tmp_i = my_localmap.LaneModel.LinearSet.likiestModel_index;
		my_localmap.LaneModel.LinearSet.LinearModelSet[tmp_i].Draw(image, 0, 19, Scalar(0,255,0), false, 2);
		my_localmap.LaneModel.LinearSet.disGisOrEdgeOnImage(image);
		my_localmap.GisFrame.disInitGisErrorOnImage(image);
		if(my_localmap.GisFrame.GisWidthLegal)
		{
			my_localmap.GisFrame.disGisWidthOnImage(image);
		}

		//my_localmap.InitialAlignmeng(image);
		//my_localmap.Correcting(image);
		//std::cout<<atan2(0,-1.0)<<std::endl;
		//std::cout<<atan(-0.0)<<std::endl;
		imshow("C_LocalMap",image);
		waitKey(3);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}

