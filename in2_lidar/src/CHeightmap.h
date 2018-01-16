
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

// shorter names for point cloud types
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
//typedef sensor_msgs::PointCloud2 VPointCloud;

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
	//void transferINS(const in2_ins::InsInfo::ConstPtr &ins_msg);
	void constructGridClouds(const VPointCloud::ConstPtr &scan, unsigned npoints,
				size_t &obs_count, size_t &empty_count, cv::OutputArray heightmapmat,
				double height_diff_threshold_);
	
	// Parameters that define the grids and the height threshold
	// Can be set via the parameter server
	int grid_dim_;
	double m_per_cell_;
	double height_diff_threshold_;
	bool showmessages,showimages;
		
	ros::Subscriber velodyne_scan_;
	image_transport::Publisher hm_publisher;
	
};
