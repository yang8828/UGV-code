// system
#include <ros/ros.h>
#include <iostream>

// message
#include <std_msgs/Int8.h>
#include <in2_msgs/UdpGeneral.h>
#include <in2_msgs/RoadEdge.h>
#include <in2_msgs/LaneInfoV2.h>

#define NODE_NAME "plan_node"
#define GIS_PARA   0
#define EDGE_PARA  1
#define LANE_PARA  2
#define GIS_AVOID  3

// ****** Constant ****** //
const long int MY_INF = 0xffffffff;

// ****** Sensor Information ****** //
in2_msgs::UdpGeneral gis_udp;
in2_msgs::UdpGeneral gis_rect;
in2_msgs::RoadEdge   road_edge;
in2_msgs::LaneInfoV2 lane_model_info;
std_msgs::Int8       decision_result;
bool is_gis_udp_rcv = false;
bool is_gis_rect_rcv = false;
bool is_road_edge_rcv = false;
bool is_lane_model_info_rcv = false;
bool is_decision_result_rcv = false;

// ****** Subscirber ****** //
ros::Subscriber gis_udp_sub_;
ros::Subscriber gis_rect_sub_;
ros::Subscriber road_edge_sub_;
ros::Subscriber lane_model_info_sub_;

// ****** Subscirber Transfer ****** //
void transferGISUdp(const in2_msgs::UdpGeneral::ConstPtr &gis_udp_ptr) {
	gis_udp = *gis_udp_ptr;
	is_gis_udp_rcv = true;
}

void transferGISRect(const in2_msgs::UdpGeneral::ConstPtr &gis_rect_ptr) {
	gis_rect = *gis_rect_ptr;
	is_gis_rect_rcv = true;
}

void transferRoadEdge(const in2_msgs::RoadEdge::ConstPtr &road_edge_ptr) {
	road_edge = *road_edge_ptr;
	is_road_edge_rcv = true;
}

void transferLaneModelInfo(const in2_msgs::LaneInfoV2::ConstPtr &lane_model_info_ptr) {
	lane_model_info = *lane_model_info_ptr;
	is_lane_model_info_rcv = true;
}

void transferDecisionResult(const std_msgs::Int8::ConstPtr &decision_result_ptr) {
	decision_result = *decision_result_ptr;
	is_decision_result_rcv = true;
}

void plan();

int main(int argc, char *argv[]) {

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh_;
	
	// ****** Subscirber ****** //
	gis_udp_sub_ = nh_.subscribe("/GISudp", 10, &transferGISUdp, ros::TransportHints().tcpNoDelay(true));
	gis_rect_sub_ = nh_.subscribe("/GISRect", 10, &transferGISRect, ros::TransportHints().tcpNoDelay(true));
	road_edge_sub_ = nh_.subscribe("/RoadEdge", 10, &transferRoadEdge, ros::TransportHints().tcpNoDelay(true));
	lane_model_info_sub_ = nh_.subscribe("/LaneModelInfo", 10, &transferLaneModelInfo, ros::TransportHints().tcpNoDelay(true));
	decision_sub_ = nh_.subscribe("/decision_reuslt", 10, &transferDecisionResult, ros::TransportHints().tcpNoDelay(true));
	
	// ****** Publisher ****** //
	plan_pub_ = nh_.advertise<in2_tcp::TcpGeneral>("/FusDec", 10);
	
	int fast_hz = 20;
	ros::Rate loop_rate(fast_hz);
	while(ros::ok()) {
	
		plan();
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();

	return 0;
}

void plan() {

	char mode = decision_result.data;
	switch(mode) {
	
		case GIS_PARA:
		break;
		
		case EDGE_PARA:
		break;
		
		case LANE_PARA:
		break;
		
		case GIS_AVOID:
		break;
		
		default:
			ROS_WARN("An unknown mode! Please check the decision node!");
		break;
	}
}
