
#include <ros/ros.h>
#include "CHeightmap.h"


#define NODE "heightmap16r_node"
int main(int argc, char** argv)
{

	ros::init(argc, argv, NODE);
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	CHeightmap hm = CHeightmap(node, priv_nh);

	ros::spin();

	ros::shutdown();
	return 0;
}
