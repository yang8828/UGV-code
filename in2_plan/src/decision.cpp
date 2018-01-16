// system
#include <ros/ros.h>
#include <iostream>

// message
#include <std_msgs/Int8.h>
#include <in2_msgs/UdpGeneral.h>
#include <in2_msgs/RoadEdge.h>
#include <in2_msgs/LaneInfoV2.h>

#define NODE_NAME "decision_node"
#define GIS_PARA      0
#define EDGE_PARA     1
#define LANE_PARA     2
#define GIS_AVOID     3
#define GIS_NO_WAY    4
#define EDGE_NO_WAY   5
#define HOLD_NO_WAY   6

// ****** Flag ****** //
bool is_recover_from_dead = false;
bool is_in_cross = false;
bool is_in_pedestrian = false;

// ****** Constant ****** //
const long int MY_INF = 0xffffffff;

// ****** Parameter ****** //
double cross_area = 22.0; // 22m

// ****** Decision Result ****** //
std_msgs::Int8 decision_result;

// ****** Sensor Information ****** //
in2_msgs::UdpGeneral gis_udp;
in2_msgs::UdpGeneral gis_rect;
in2_msgs::RoadEdge   road_edge;
in2_msgs::LaneInfoV2 lane_model_info;
bool is_gis_udp_rcv = false;
bool is_gis_rect_rcv = false;
bool is_road_edge_rcv = false;
bool is_lane_model_info_rcv = false;
double dis_to_cross = (double)MY_INF;

// ****** Subscirber ****** //
ros::Subscriber gis_udp_sub_;
ros::Subscriber gis_rect_sub_;
ros::Subscriber road_edge_sub_;
ros::Subscriber lane_model_info_sub_;

// ****** Publisher ****** //
ros::Publisher decision_pub_;

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

void makeDecision();

int main(int argc, char *argv[]) {

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh_;
	
	// ****** Subscirber ****** //
	gis_udp_sub_ = nh_.subscribe("/GISudp", 10, &transferGISUdp, ros::TransportHints().tcpNoDelay(true));
	gis_rect_sub_ = nh_.subscribe("/GISRect", 10, &transferGISRect, ros::TransportHints().tcpNoDelay(true));
	road_edge_sub_ = nh_.subscribe("/RoadEdge", 10, &transferRoadEdge, ros::TransportHints().tcpNoDelay(true));
	lane_model_info_sub_ = nh_.subscribe("/LaneModelInfo", 10, &transferLaneModelInfo, ros::TransportHints().tcpNoDelay(true));
	
	// ****** Publisher ****** //
	decision_pub_ = nh_.advertise<std_msgs::Int8>("/decision_result", 10);
	
	int fast_hz = 20;
	int FQ_5HZ = fast_hz / 5;
	int divide_fq_cnt = 0;
	ros::Rate loop_rate(fast_hz);
	while(ros::ok()) {
	
		divide_fq_cnt++;
		if (divide_fq_cnt >= FQ_5HZ) { // enter here at 5Hz
		
			divide_fq_cnt = 0;
			// make decision
			makeDecision();
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();

	return 0;
}

void makeDecision() {

	char mode = GIS_PARA;
	if (dis_to_cross < cross_area) {
		if (!is_recover_from_dead)
			mode = GIS_NO_WAY;
		else
			mode = GIS_PARA;
	}
	
	decision_result.data = mode = EDGE_PARA;
	decision_pub_.publish(decision_result);
}
