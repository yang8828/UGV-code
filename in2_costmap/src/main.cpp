#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <in2_msgs/LaneMarks.h>
#include <in2_msgs/RoadSurface.h>
#include "Coster.h"

using namespace cv;
using namespace std;

long int lane_dog = 0;

class CMain
{
	public:
		ros::Subscriber datapts_sub;
		ros::Subscriber road_surface_sub;
		ros::Subscriber lane_marks_sub;
		ros::Subscriber ins_sub;
		image_transport::Publisher obs_cost_map_pub;
		image_transport::Publisher lane_cost_map_pub;
		//in2_msgs::DebugInfo debuginfo_msg;
		
		bool isbusy;
		bool RecvIns;
		
		void periodInit()
		{
			RecvIns = false;
			oCoster.CurIns = oCoster.Ins;
		}
		
		void sendDebugInfo()
		{
			//debug_pub.publish(debuginfo_msg);
		}

		void sendObsCostMap()
		{
			//Mat obs_cost_map;
			//cv::resize(oCoster.LidarMaskFilt, obs_cost_map, cv::Size(3000,3000));
			
			std_msgs::Header header;
			header.frame_id = "obs_cost_map";
			Mat lidar_mask_filt_300;
			cv::resize(oCoster.LidarMaskFilt, lidar_mask_filt_300, cv::Size(300,300));
			//cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, "mono8", obs_cost_map);
			cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, "mono8", oCoster.LidarMaskFilt);
			obs_cost_map_pub.publish(cv_image.toImageMsg());
		}
		
		void sendLaneCostMap() {
		
			std_msgs::Header header;
			header.frame_id = "lane_cost_map";
			cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, "mono8", oCoster.LaneMaskFilt);
			lane_cost_map_pub.publish(cv_image.toImageMsg());
		}
		
		void displayObs()
		{
			// obs
			oCoster.LidarMaskFilt.copyTo(ObsCostMapShow);
			/*
			for(int i=0; i<oCoster.DataPts3000.size(); i++)
			{
				cv::circle(ObsCostMapShow, oCoster.DataPts3000[i], 5, 255, -1, CV_AA, 0);
			}
			*/
			cv::resize(ObsCostMapShow, ObsCostMapShowF, cv::Size(750,750));
		}
		
		void displayLane() {
			// lane
			oCoster.LaneMaskFilt.copyTo(LaneCostMapShow);
			cv::resize(LaneCostMapShow, LaneCostMapShowF, cv::Size(750,750));
			//cv::resize(LaneCostMapShow, LaneCostMapShowF, LaneCostMapShowF.size());
		}
		
		// Scan
		void transferDataPts(const in2_msgs::ScanInfoV2::ConstPtr &datapts_msg)
		{
			//if(!RecvIns) return;
		
			if(isbusy) return;
			isbusy = true;

			//cout << "I get a ScanInfoV2!" << endl;
			periodInit();
			oCoster.oScanInfo = *datapts_msg;
			oCoster.GenerateObsCostMap();
			sendDebugInfo();
			sendObsCostMap();
			//displayObs();
			
			isbusy = false;
		}
		
		void transferRoadSurface(const in2_msgs::RoadSurface::ConstPtr road_surface_ptr) {
			//if(!RecvIns) return;
		
			if(isbusy) return;
			isbusy = true;

			//cout << "I get a ScanInfoV2!" << endl;
			periodInit();
			//oCoster.oScanInfo = *datapts_msg;
			
			// 1.
			int pts_num = 720;
			for (int i = 0; i < pts_num; i++) {
				oCoster.oScanInfo.ScanInfoX[i] = 1500 + road_surface_ptr->points[i].x / 0.02;
				oCoster.oScanInfo.ScanInfoY[i] = 1000 + road_surface_ptr->points[i].y / 0.02;
			}
			
			oCoster.GenerateObsCostMap();
			sendDebugInfo();
			sendObsCostMap();
			//displayObs();
			
			isbusy = false;
		}
		
		// Lane
		void transferLaneMarks(const in2_msgs::LaneMarks::ConstPtr &lane_marks_ptr) {
		
			//cout << "I get a LaneMarks!" << endl;
			
			lane_dog = 0;
			periodInit();
			oCoster.oLaneMarks = *lane_marks_ptr;
			oCoster.GenerateLaneCostMap();
			sendDebugInfo();
			sendLaneCostMap();
			//displayLane();
		}
		
		/*void transferINS(const in2_ins::InsInfo::ConstPtr &ins_msg)
		{
			RecvIns = true;
			oCoster.Ins.x = ins_msg->position.x;
			oCoster.Ins.y = ins_msg->position.y;
			oCoster.Ins.z = ins_msg->attitude.z;
		}*/
	
		void init()
		{
			isbusy = false;
			RecvIns = false;
		}
	public:
		Mat ObsCostMapShow;
		Mat ObsCostMapShowF;
		Mat LaneCostMapShow;
		Mat LaneCostMapShowF;
		Coster oCoster;
		CMain(ros::NodeHandle node, ros::NodeHandle priv_nh)
		{
			//datapts_sub = node.subscribe("ScanInfoV2", 10, &CMain::transferDataPts, this, ros::TransportHints().tcpNoDelay(true));
			road_surface_sub = node.subscribe("RoadSurface", 10, &CMain::transferRoadSurface, this, ros::TransportHints().tcpNoDelay(true));
			lane_marks_sub = node.subscribe("LaneMarks", 10, &CMain::transferLaneMarks, this, ros::TransportHints().tcpNoDelay(true));
			//ins_sub = node.subscribe("ins", 10, &CMain::transferINS, this, ros::TransportHints().tcpNoDelay(true));
			
			image_transport::ImageTransport it_obs(node);
			obs_cost_map_pub = it_obs.advertise("obs_cost_map", 10);
			
			image_transport::ImageTransport it_lane(node);
			lane_cost_map_pub = it_lane.advertise("lane_cost_map", 10);
	
			//debug_pub = node.advertise<in2_msgs::DebugInfo>("DebugInfo", 10);
			
			init();
		}

		~CMain(){}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "costmap_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	CMain cm = CMain(node,priv_nh);

	ros::Rate r(10);
	while(ros::ok())
	{
		//if(!cm.ObsCostMapShowF.empty())
		//{
		//	cv::imshow("ObsCostMap", cm.ObsCostMapShowF);
		//	cv::waitKey(3);
		//}
		
		//if(!cm.LaneCostMapShowF.empty())
		//{
		//	namedWindow("LaneCostMap", 0);
		//	cv::imshow("LaneCostMap", cm.LaneCostMapShowF);
		//	cv::waitKey(3);
		//}
		
		lane_dog++;
		cout << "[costmap_node] lane_dog: " << lane_dog << endl;
		if (lane_dog >= 10) { // 1.0s
			cout << "lane is boom!" << endl;
			lane_dog = 0;
			cm.oCoster.LaneMaskFilt = Mat::zeros(L,L,CV_8UC1);
			cm.sendLaneCostMap();
		}
		
		ros::spinOnce();
		r.sleep();
	}

	ros::shutdown();       
	return 0;
}

