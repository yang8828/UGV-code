
#include "CHeightmap.h"

CHeightmap::CHeightmap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
	// Initialize parameters
	priv_nh.param("cell_size", m_per_cell_, 0.20);
	priv_nh.param("grid_dimensions", grid_dim_, 300);
	//priv_nh.param("height_threshold", height_diff_threshold_, 0.05);
	// Set up publishers
	image_transport::ImageTransport it(node);
	hm_publisher_ = it.advertise("heightmap_ibeo",1);

	// subscribe to Velodyne data points
	ibeo_scan_ = node.subscribe("ibeoData_node", 10,
					&CHeightmap::processData, this,
					ros::TransportHints().tcpNoDelay(true));
}

CHeightmap::~CHeightmap() {}

/** point cloud input callback */
void CHeightmap::processData(const VPointCloud::ConstPtr &scan)
{
	// set the exact point cloud size -- the vectors should already have
	// enough space
	size_t npoints = scan->points.size();
	size_t obs_count=0;
	size_t empty_count=0;
	constructGridClouds(scan,npoints,obs_count, empty_count, HeightMapMat);

}

void CHeightmap::constructGridClouds(const VPointCloud::ConstPtr &scan,
					unsigned npoints, size_t &obs_count,
					size_t &empty_count, cv::OutputArray heightmapmat)
{
	//float min[grid_dim_][grid_dim_];
	//float max[grid_dim_][grid_dim_];
	float num_obs[grid_dim_][grid_dim_];
	float num_clear[grid_dim_][grid_dim_];
	bool init[grid_dim_][grid_dim_];

	for (int x = 0; x < grid_dim_; x++) {
		for (int y = 0; y < grid_dim_; y++) {
			init[x][y]=false;
			num_obs[x][y]=0;
			num_clear[x][y]=0;
		}
	}

	// build height map
	for (unsigned i = 0; i < npoints; ++i) {
		int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
		int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
			/*if (!init[x][y]) {
				min[x][y] = scan->points[i].z;
				max[x][y] = scan->points[i].z;
				num_obs[x][y] = 0;
				num_clear[x][y] = 0;
				init[x][y] = true;
			} else {
				min[x][y] = MIN(min[x][y], scan->points[i].z);
				max[x][y] = MAX(max[x][y], scan->points[i].z);
			}*/
			if (scan->channels[0].values[i]!=3) {
				num_obs[x][y]++;
			} else {
				num_clear[x][y]++;
			}
		}
	}

	// calculate number of obstacles in each cell
	/*for (unsigned i = 0; i < npoints; ++i) {
		int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
		int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
			if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {
				num_obs[x][y]++;
			} else {
				num_clear[x][y]++;
			}
		}
	}*/
	Mat heightmap = Mat(grid_dim_, grid_dim_, CV_8UC3, Scalar(100,100,100));
	// create clouds from grid
	double grid_offset=grid_dim_/2.0*m_per_cell_;
	for (int x = 0; x < grid_dim_; x++) {
		for (int y = 0; y < grid_dim_; y++) {
			if (num_obs[x][y]>0) {
				obs_count++;
				// update heightmap show
				heightmap.at<Vec3b>(grid_dim_-1-x,grid_dim_-1-y)[0]=(unsigned char)255;
				heightmap.at<Vec3b>(grid_dim_-1-x,grid_dim_-1-y)[1]=(unsigned char)0;
				heightmap.at<Vec3b>(grid_dim_-1-x,grid_dim_-1-y)[2]=(unsigned char)0;
			}
			else if (num_clear[x][y]>0) {
				empty_count++;
				// update heightmap show
				heightmap.at<Vec3b>(grid_dim_-1-x,grid_dim_-1-y)[0]=(unsigned char)0;
				heightmap.at<Vec3b>(grid_dim_-1-x,grid_dim_-1-y)[1]=(unsigned char)255;
				heightmap.at<Vec3b>(grid_dim_-1-x,grid_dim_-1-y)[2]=(unsigned char)0;
			}
		}
	}
	//ROS_INFO("obs_clouds_size=%d,clear_clouds_size=%d",obs_count,empty_count);
	heightmap.copyTo(heightmapmat);
	//imshow("heightmap",heightmap);
	//waitKey(5);

	//publish heightmap topic
	std_msgs::Header header;
	header.frame_id = "heigtmap";
	cv_bridge::CvImage cv_image = cv_bridge::CvImage(header,"bgr8",heightmap);
	hm_publisher_.publish(cv_image.toImageMsg());
}
