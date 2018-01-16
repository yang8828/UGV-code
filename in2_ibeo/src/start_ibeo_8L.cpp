/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


//#include <mrpt/hwdrivers/CIbeoLuxETH.h>
#include <sensor_msgs/PointCloud.h>
#include "CIbeoLuxETH.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <in2_msgs/IbeoObjects.h>


using namespace mrpt::hwdrivers;

sensor_msgs::PointCloud cloud_pc_upper;
sensor_msgs::PointCloud cloud_pc_lower;
sensor_msgs::PointCloud cloud_pc_whole;

in2_msgs::IbeoObjects Objects;

int numScanpoints_upper=0;
int numScanpoints_lower=0;

bool get_lower_flag		=	false;
bool get_upper_flag	=	false;

bool data_flag = false;
bool objects_flag = false;

int IBEO_TYPE;

void integrateCloud()
{
			int	integrate_counter=0;

			cloud_pc_whole.header.stamp = ros::Time::now();
			cloud_pc_whole.header.frame_id = "base_ibeo_8L";
			cloud_pc_whole.points.resize(numScanpoints_upper+numScanpoints_lower);
			cloud_pc_whole.channels.resize(4);
			cloud_pc_whole.channels[0].name = "Preprocessing Labels";
			cloud_pc_whole.channels[0].values.resize(numScanpoints_upper+numScanpoints_lower);
			cloud_pc_whole.channels[1].name = "Layers";
			cloud_pc_whole.channels[1].values.resize(numScanpoints_upper+numScanpoints_lower);
			cloud_pc_whole.channels[2].name = "Hangle";
			cloud_pc_whole.channels[2].values.resize(numScanpoints_upper+numScanpoints_lower);
			cloud_pc_whole.channels[3].name = "Distance";
			cloud_pc_whole.channels[3].values.resize(numScanpoints_upper+numScanpoints_lower);


			for( ;integrate_counter<numScanpoints_upper;integrate_counter++)
			{
					cloud_pc_whole.points[integrate_counter].x = cloud_pc_upper.points[integrate_counter].x;
					cloud_pc_whole.points[integrate_counter].y = cloud_pc_upper.points[integrate_counter].y;
					cloud_pc_whole.points[integrate_counter].z = cloud_pc_upper.points[integrate_counter].z;
					cloud_pc_whole.channels[0].values[integrate_counter] = cloud_pc_upper.channels[0].values[integrate_counter];
					cloud_pc_whole.channels[1].values[integrate_counter] = cloud_pc_upper.channels[1].values[integrate_counter];
					cloud_pc_whole.channels[2].values[integrate_counter] = cloud_pc_upper.channels[2].values[integrate_counter];
					cloud_pc_whole.channels[3].values[integrate_counter] = cloud_pc_upper.channels[3].values[integrate_counter];

			}
			for( ;integrate_counter<numScanpoints_upper+numScanpoints_lower; integrate_counter++)
			{
					cloud_pc_whole.points[integrate_counter].x = cloud_pc_lower.points[integrate_counter-numScanpoints_upper].x;
					cloud_pc_whole.points[integrate_counter].y = cloud_pc_lower.points[integrate_counter-numScanpoints_upper].y;
					cloud_pc_whole.points[integrate_counter].z = cloud_pc_lower.points[integrate_counter-numScanpoints_upper].z;
					cloud_pc_whole.channels[0].values[integrate_counter] = cloud_pc_lower.channels[0].values[integrate_counter-numScanpoints_upper] ;
					cloud_pc_whole.channels[1].values[integrate_counter] = cloud_pc_lower.channels[1].values[integrate_counter-numScanpoints_upper] ;
					cloud_pc_whole.channels[2].values[integrate_counter] = cloud_pc_lower.channels[2].values[integrate_counter-numScanpoints_upper] ;
					cloud_pc_whole.channels[3].values[integrate_counter] = cloud_pc_lower.channels[3].values[integrate_counter-numScanpoints_upper] ;
			}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ibeo_8L_node");
	ros::NodeHandle node;
	ros::NodeHandle n_tf;

 	ros::Publisher scan_pub_;
	ros::Publisher objects_pub_;
	tf::TransformBroadcaster broadcaster;

	scan_pub_ = node.advertise<sensor_msgs::PointCloud>("ibeo_8L_scan", 1000);
	objects_pub_ = node.advertise<in2_msgs::IbeoObjects>("ibeo_8L_objects", 1000);

	CIbeoLuxETH ibeo_lux_8L("192.168.0.21",12002);
	IBEO_TYPE = IBEO_8L_LUX;
	ibeo_lux_8L.initialize();

	while(ros::ok())
	{
		ibeo_lux_8L.doProcess();
		if(data_flag)
		{
			integrateCloud();
			scan_pub_.publish(cloud_pc_whole);
			data_flag = false;
			get_upper_flag=false;
			get_lower_flag=false;
			broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"base_vehicle", "base_ibeo_8L"));
		}

		if(objects_flag)
		{
			Objects.header.frame_id = "base_ibeo_4L";
			objects_pub_.publish(Objects);
			objects_flag = false;
		}

	}
	ibeo_lux_8L.stop();
	ros::shutdown();
	return 0;
}
