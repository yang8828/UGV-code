
#include <ros/ros.h>
#include "CHeightmap.h"


#define NODE "heightmap_node"
int main(int argc, char** argv)
{

	ros::init(argc, argv, NODE);
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	CHeightmap hm = CHeightmap(node, priv_nh);

	//cvStartWindowThread();
	ros::spin();
	/*namedWindow( "Heightmap", 0 );
	while(ros::ok())
	{
		if(!hm.HeightMapMat.empty())
		{
			cv::imshow("Heightmap",hm.HeightMapMat);
			resizeWindow("Heightmap",600,600);
			cv::imwrite("Heightmap.bmp",hm.HeightMapMat);
			cv::waitKey(10);
		}
		ros::spinOnce();
	}*/
	ros::shutdown();
	return 0;
}
