#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

//for pcl RANSAC
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_broadcaster.h>

#define PI 3.14159265357

using namespace std;

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
VPointCloud scan;

ros::Publisher rotate_16r;
double rotate_angle;

void processScan( const VPointCloud::ConstPtr &scan_msg )
{

	// point cloud data
	scan = *scan_msg;

	VPointCloud::Ptr new_scan(new pcl::PointCloud<pcl::PointXYZI>);
	new_scan->width=scan.width;
	new_scan->height=scan.height;
	new_scan->points.resize(new_scan->width*new_scan->height);

	for(size_t i=0; i<scan.points.size();i++)
	{
		double rot_x,rot_y,rot_z;
		rot_x =  scan.points[i].x * cos(rotate_angle*PI/180) + scan.points[i].z * sin(rotate_angle*PI/180) ;
		rot_y =  scan.points[i].y;
		rot_z = -sin(rotate_angle*PI/180) * scan.points[i].x + scan.points[i].z * cos(rotate_angle*PI/180) ;
		
		rot_x = rot_x;
		double tempy = rot_y * cos(-1.5*PI/180) + rot_z * ( -sin(-1.5*PI/180) );
		double tempz = rot_y * sin(-1.5*PI/180) + rot_z * cos(-1.5*PI/180);
		
		rot_y = tempy;
		rot_z = tempz;
		
		double tempx = rot_x * cos(-0.0*PI/180) - rot_y * sin(-0.0*PI/180);
		tempy = rot_x * sin(-0.0*PI/180) + rot_y * cos(-0.0*PI/180);//-0.8
		
		rot_x = tempx;
		rot_y = tempy;

		rot_x = rot_x + 0.435;//0.435
		rot_y = rot_y + 0.955 +0.35;
		rot_z = rot_z + 1.550;
		
		new_scan->points[i].x=rot_y;
		new_scan->points[i].y=-rot_x;
		new_scan->points[i].z=rot_z;

		/*new_scan->points[i].x=rot_x;
		new_scan->points[i].y=rot_y;
		new_scan->points[i].z=rot_z;*/
	}

	sensor_msgs::PointCloud2 after_rotate;
	pcl::toROSMsg(*new_scan,after_rotate);
	//after_rotate.header.frame_id="velodyne";
	after_rotate.header.frame_id="base_vehicle";
	rotate_16r.publish(after_rotate);

}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "rotate_16r_node");

	ros::NodeHandle node;

	node.param("/rotate_16r_node/rotate_angle_16r", rotate_angle, 19.0);

	ros::Subscriber scan;
	scan = node.subscribe("vlp_r_points",10, &processScan,  ros::TransportHints().tcpNoDelay(true));
	rotate_16r = node.advertise <sensor_msgs::PointCloud2>("rotate_16r", 1);

	ros::spin();

	ros::shutdown();
	return 0;
}
