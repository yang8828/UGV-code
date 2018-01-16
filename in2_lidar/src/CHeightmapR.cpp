
#include "CHeightmap.h"

CHeightmap::CHeightmap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
	// Initialize parameters
	priv_nh.param("cell_size",         m_per_cell_,             0.20);
	priv_nh.param("grid_dimensions",   grid_dim_,                300);
	priv_nh.param("height_threshold_", height_diff_threshold_,  0.05);
	node.param("/heightmap16r_node/showmessages", showmessages, false);
    node.param("/heightmap16r_node/showimages",   showimages,   false);

	// Set up publishers  
	image_transport::ImageTransport it(node);
	hm_publisher = it.advertise("heightmap16r",1);
	
	// subscribe to Velodyne data points
	//velodyne_scan_ = node.subscribe("velodyne_points", 10,
	velodyne_scan_ = node.subscribe("rotate_16r", 10,
					&CHeightmap::processData, this,
					ros::TransportHints().tcpNoDelay(true));
}

CHeightmap::~CHeightmap() {}

/* point cloud input callback */
void CHeightmap::processData(const VPointCloud::ConstPtr &scan)
{
	// set the exact point cloud size -- the vectors should already have
	// enough space
	size_t npoints = scan->points.size();
	size_t obs_count;
	size_t empty_count;

	obs_count=0;
	empty_count=0;
	constructGridClouds(scan,npoints,obs_count, empty_count, HeightMapMat, height_diff_threshold_);
	if(showimages)
	{
		imshow("heightmap16r",HeightMapMat);
		waitKey(1);
	}

	

	//publish heightmap topic 
	std_msgs::Header header0;
	header0.frame_id = "heightmap16r";
	header0.stamp = ros::Time::now();
	cv_bridge::CvImage cv_image = cv_bridge::CvImage(header0,"bgr8",HeightMapMat);
	hm_publisher.publish(cv_image.toImageMsg());

}

void CHeightmap::constructGridClouds(const VPointCloud::ConstPtr &scan,
					                 unsigned npoints, size_t &obs_count,
					                 size_t &empty_count, cv::OutputArray heightmapmat, 
					                 double height_diff_threshold_)
{
	float min[grid_dim_][grid_dim_];
	float max[grid_dim_][grid_dim_];
	float num_obs[grid_dim_][grid_dim_];
	float num_clear[grid_dim_][grid_dim_];
	bool  init[grid_dim_][grid_dim_];

	for (int x = 0; x < grid_dim_; x++) {
		for (int y = 0; y < grid_dim_; y++) {
			init[x][y]=false;
			num_obs[x][y]=0;
			num_clear[x][y]=0;
		}
	}

	// build height map
	for (unsigned i = 0; i < npoints; ++i) 
	{
		int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
		int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);//+ -> -
		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) 
		{
			if (!init[x][y]) {
				min[x][y] = scan->points[i].z;
				max[x][y] = scan->points[i].z;
				num_obs[x][y] = 0;
				num_clear[x][y] = 0;
				init[x][y] = true;
			} else {
				min[x][y] = MIN(min[x][y], scan->points[i].z);
				max[x][y] = MAX(max[x][y], scan->points[i].z);
			}
		}
	}

	// calculate number of obstacles in each cell
	for (unsigned i = 0; i < npoints; ++i) 
	{
		int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
		int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
		if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
			if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {  
				num_obs[x][y]++;
			} else {
				num_clear[x][y]++;
			}
		}
	}
	Mat heightmap = Mat(grid_dim_, grid_dim_, CV_8UC3, Scalar(100,100,100));
	
	// create clouds from grid
	double grid_offset=grid_dim_/2.0*m_per_cell_;
	for (int x = 0; x < grid_dim_; x++) {
		for (int y = 0; y < grid_dim_; y++) {
			if (num_obs[x][y]>0) 
			{
				obs_count++;
				// update heightmap show
				//heightmap.at<unsigned char>(x,y)=(unsigned char)0;
				heightmap.at<Vec3b>(grid_dim_-x-1,grid_dim_-y-1)[0]=(unsigned char)255;
				heightmap.at<Vec3b>(grid_dim_-x-1,grid_dim_-y-1)[1]=(unsigned char)0;
				heightmap.at<Vec3b>(grid_dim_-x-1,grid_dim_-y-1)[2]=(unsigned char)0;
			}
			if (num_clear[x][y]>0) {
				empty_count++;
				// update heightmap show
				//heightmap.at<unsigned char>(x,y)=(unsigned char)255;
				heightmap.at<Vec3b>(grid_dim_-x-1,grid_dim_-y-1)[0]=(unsigned char)0;
				heightmap.at<Vec3b>(grid_dim_-x-1,grid_dim_-y-1)[1]=(unsigned char)255;
				heightmap.at<Vec3b>(grid_dim_-x-1,grid_dim_-y-1)[2]=(unsigned char)0;
			}
		}
	}

	heightmap.copyTo(heightmapmat);
}
