
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <vector>
//#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>

using namespace cv;

class CHeightmap
{
public:
	void recieve_heightmap_16l(const sensor_msgs::ImageConstPtr &heightmap16l);
	void recieve_heightmap_16r(const sensor_msgs::ImageConstPtr &heightmap16r);
	void recieve_heightmap_ibeo(const sensor_msgs::ImageConstPtr &heightmapibeo);

	CHeightmap(ros::NodeHandle node, ros::NodeHandle priv_nh);
	//~CHeightmap();

	bool showmessages;
	bool showimages;
	
private:
	ros::Subscriber hml;
	ros::Subscriber hmr;
	ros::Subscriber hmibeo;

	Mat heightmap1;
	Mat heightmap2;
	Mat heightmap3;

	Mat heightmap1_old;
	Mat heightmap2_old;
	Mat heightmap3_old;

	bool flag16l,flag16r,flagibeo;
	int rcv_16l,rcv_16r,rcv_ibeo;

	image_transport::Publisher hm_publisher;
};
