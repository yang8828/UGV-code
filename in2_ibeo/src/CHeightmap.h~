
#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud.h>
#include "std_msgs/String.h"
#include "math.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//#include <in2_ins/InsInfo.h>

// shorter names for point cloud types
typedef sensor_msgs::PointCloud VPointCloud;
//typedef pcl::PointXYZI VPoint;
//typedef pcl::PointCloud<VPoint> VPointCloud;

using namespace cv;

class CHeightmap
{
public:
	CHeightmap(ros::NodeHandle node, ros::NodeHandle priv_nh);
	~CHeightmap();
	void processData(const VPointCloud::ConstPtr &scan);

	// Height Map Matrix
	Mat HeightMapMat;

private:
//	void transferINS(const in2_ins::InsInfo::ConstPtr &ins_msg);
	void constructGridClouds(const VPointCloud::ConstPtr &scan, unsigned npoints,
				size_t &obs_count, size_t &empty_count, cv::OutputArray heightmapmat);

	// Parameters that define the grids and the height threshold
	// Can be set via the parameter server
	int grid_dim_;
	double m_per_cell_;
	double height_diff_threshold_;

	ros::Subscriber ibeo_scan_;
	image_transport::Publisher hm_publisher_;
};
