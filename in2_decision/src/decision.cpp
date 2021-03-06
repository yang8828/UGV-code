// system
#include <ros/ros.h>
#include <iostream>

using namespace std;

// message
#include <std_msgs/Int8.h>
#include <in2_msgs/UdpGeneral.h>
#include <in2_msgs/RoadEdge.h>
#include <in2_msgs/LaneInfoV2.h>
#include <in2_msgs/GisInfo.h>
#include <in2_msgs/VehicleInfo.h>
#include <in2_msgs/TrafficLights.h>

#define NODE_NAME "decision_node"
#define GIS_PARA      0
#define EDGE_PARA     1
#define LANE_PARA     2
#define GIS_AVOID     3
#define GIS_NO_WAY    4
#define EDGE_NO_WAY   5
#define HOLD_NO_WAY   6
#define GIS_SOFT      7
#define RECOVER_FROM_DEAD_WAIT_TIME   100 // 10s

// ****** Counter ****** //
long int recover_from_dead_cnt = 0;
long int in_shi_gong_time = 0;

// ****** Flag ****** //
bool is_recover_from_dead = false;
bool is_in_cross = false;
bool is_in_pedestrian = false;
bool is_in_suidao = false;
bool is_in_fee = false;

// ****** Constant ****** //
const long int MY_INF = 0xffffffff;

// ****** Parameter ****** //
double cross_area = 40.0; // 40.0m

// ****** Decision Result ****** //
std_msgs::Int8 decision_result;
bool in_shi_gong = false;

// ****** Sensor Information ****** //
in2_msgs::UdpGeneral gis_udp;
in2_msgs::UdpGeneral gis_rect;
in2_msgs::RoadEdge   road_edge;
in2_msgs::LaneInfoV2 lane_model_info;
in2_msgs::GisInfo gis_info;
in2_msgs::VehicleInfo vehicle_info;
bool is_gis_udp_rcv = false;
bool is_gis_rect_rcv = false;
bool is_road_edge_rcv = false;
bool is_lane_model_info_rcv = false;
bool is_gis_info_rcv = false;
bool is_vehicle_info_rcv = false;
bool is_recover_timer_ok = false;
double dis_to_cross = (double)MY_INF;
double real_speed = 0.0; // km/h

// ****** Subscirber ****** //
ros::Subscriber gis_udp_sub_;
ros::Subscriber gis_rect_sub_;
ros::Subscriber road_edge_sub_;
ros::Subscriber lane_model_info_sub_;
ros::Subscriber gis_info_sub_;
ros::Subscriber vehicle_info_sub_;
ros::Subscriber traffic_lights_sub_;
in2_msgs::TrafficLights traffic_lights;
bool is_traffic_lights_rcv = false;

// ****** Publisher ****** //
ros::Publisher decision_pub_;

int forwd=1;
int lturn=1;
int rturn=1;
int islight = 0;
int Gis_TurnDirection = 0;

bool IsRedLight()
{	
	if(islight==1)
	{
		if((Gis_TurnDirection==0&&forwd==0)||(Gis_TurnDirection==1&&lturn==0)||(Gis_TurnDirection==2&&rturn==0))
			return(true);			
		else
			return(false);
	}
	else if(islight==0)
	{
		return(false);
	}
	else
		return(false);
}

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

void transferGisInfo(const in2_msgs::GisInfo::ConstPtr &gis_info_ptr) {
	gis_info = *gis_info_ptr;
	is_gis_info_rcv = true;
	dis_to_cross = gis_info.DistanceToNextCross;
	
	// cross
	if (dis_to_cross < cross_area) is_in_cross = true;
	else                           is_in_cross = false;
	
	// pedestrian
	if (gis_info.LaneChangeFlag == 2) is_in_pedestrian = true;
	else                              is_in_pedestrian = false;
	
	//if (!is_in_cross && !is_in_pedestrian) is_recover_from_dead = false;
	
	int cross_state = gis_info.CrossRoadType;
	if(cross_state<3)
	{
		islight = 0;
		if(cross_state==0)
			Gis_TurnDirection = 0;
		else if(cross_state==1)
			Gis_TurnDirection = 1;
		else
			Gis_TurnDirection = 2;
	}
	else
	{
		islight = 1;
		if(cross_state==3)
			Gis_TurnDirection = 0;
		else if(cross_state==4)
			Gis_TurnDirection = 1;
		else
			Gis_TurnDirection = 2;
	}
	
	if (gis_info.LaneChangeFlag == 3)  in_shi_gong = true;
	else                              in_shi_gong = false;
}

void transferVehicleInfo(const in2_msgs::VehicleInfo::ConstPtr &vehicle_info_ptr) {
	vehicle_info = *vehicle_info_ptr;
	is_vehicle_info_rcv = true;
	
	real_speed = vehicle_info.Speed; // km/h
}

void recoverFromDeadCallback(const ros::TimerEvent &event) {
	is_recover_timer_ok = true;

	if (in_shi_gong) {
		in_shi_gong_time++;
		if (in_shi_gong_time > 600) in_shi_gong = false;
	}
	
	if ((decision_result.data == GIS_NO_WAY ||
	    decision_result.data == EDGE_NO_WAY ||
	    decision_result.data == HOLD_NO_WAY ||
	    decision_result.data == GIS_SOFT) &&
	    fabs(real_speed) < 2.0 && !is_in_pedestrian && !IsRedLight() && !in_shi_gong && !is_in_suidao) { // km/h
	    	recover_from_dead_cnt++;
	    	if (recover_from_dead_cnt > RECOVER_FROM_DEAD_WAIT_TIME) {
	    		recover_from_dead_cnt = 0;
	    		is_recover_from_dead = true;
	    	}
	    }
}

void transferTrafficLightsInfo(const in2_msgs::TrafficLights::ConstPtr &traffic_lights_ptr) {

	traffic_lights = *traffic_lights_ptr;
	is_traffic_lights_rcv = true;
	
	lturn = (traffic_lights.TurnLeft.Color == traffic_lights.TurnLeft.COLOR_RED) ? 0 : 1;
	forwd = (traffic_lights.Forward.Color == traffic_lights.TurnLeft.COLOR_RED) ? 0 : 1;
	rturn = (traffic_lights.TurnRight.Color == traffic_lights.TurnLeft.COLOR_RED) ? 0 : 1;
}

void makeDecision();

int main(int argc, char *argv[]) {

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh_;
	
	// ****** Subscirber ****** //
	/*gis_udp_sub_ = nh_.subscribe("/GISudp", 10, &transferGISUdp, ros::TransportHints().tcpNoDelay(true));
	gis_rect_sub_ = nh_.subscribe("/GISRect", 10, &transferGISRect, ros::TransportHints().tcpNoDelay(true));
	road_edge_sub_ = nh_.subscribe("/RoadEdge", 10, &transferRoadEdge, ros::TransportHints().tcpNoDelay(true));
	lane_model_info_sub_ = nh_.subscribe("/LaneModelInfo", 10, &transferLaneModelInfo, ros::TransportHints().tcpNoDelay(true));*/
	gis_info_sub_ = nh_.subscribe("/gis_info", 10, &transferGisInfo, ros::TransportHints().tcpNoDelay(true));
	vehicle_info_sub_ = nh_.subscribe("vehicle_info", 10, &transferVehicleInfo, ros::TransportHints().tcpNoDelay(true));
	traffic_lights_sub_ = nh_.subscribe("TrafficLightsInfo", 10, &transferTrafficLightsInfo, ros::TransportHints().tcpNoDelay(true));
	
	// ****** Publisher ****** //
	decision_pub_ = nh_.advertise<std_msgs::Int8>("/decision_result", 10);
	
	// ****** Timer ****** //
	ros::Timer r_tmr = nh_.createTimer(ros::Duration(0.1), recoverFromDeadCallback);
	
	// ****** Parameter ****** //
	nh_.getParam("/decision_node/is_recover_from_dead", is_recover_from_dead);
	
	int fast_hz = 20;
	int FQ_5HZ = fast_hz / 5;
	int divide_fq_cnt = 0;
	ros::Rate loop_rate(fast_hz);
	
	while(!is_gis_info_rcv || !is_recover_timer_ok) {
		cout << "Wait message!" << endl;
		ros::spinOnce();
		loop_rate.sleep();
		
		if (!ros::ok())
			break;
	}
	
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
	if (is_in_pedestrian) {
		if (!is_recover_from_dead)
			mode = GIS_NO_WAY;
		else
			mode = GIS_PARA;
	} else if (is_in_cross) {
		if (!is_recover_from_dead)
			mode = GIS_NO_WAY;
		else
			mode = GIS_PARA;
	} else {
		mode = GIS_PARA;
		is_recover_from_dead = false;
	}
	
	decision_result.data = mode;
	decision_pub_.publish(decision_result);
}
