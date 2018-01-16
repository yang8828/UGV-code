uint8 LANE_CHANGE_FLAG_FORBID = 0
uint8 LANE_CHANGE_FLAG_FREE = 1

uint8 MISSION_STATE_BEFORE_MISSION = 0
uint8 MISSION_STATE_BEFORE_LAUNCH = 1
uint8 MISSION_STATE_DRIVING = 2
uint8 MISSION_STATE_END = 3

uint8 CROSS_ROAD_STRAIGHT_NO_LIGHT = 0
uint8 CROSS_ROAD_LEFT_NO_LIGHT = 1
uint8 CROSS_ROAD_RIGHT_NO_LIGHT = 2
uint8 CROSS_ROAD_STRAIGHT_WITH_LIGHT = 3
uint8 CROSS_ROAD_LEFT_WITH_LIGHT = 4
uint8 CROSS_ROAD_RIGHT_WITH_LIGHT = 5

uint8 TRAFFIC_SIGN_FALSE = 0
uint8 TRAFFIC_SIGN_TRUE = 1

uint8 ALIGN_TO_LANE_CENTER_TRUE = 0
uint8 ALIGN_TO_LANE_CENTER_FALSE = 1

uint8 CURVE_ID_NONE = 0
uint8 CURVE_ID_CROSS = 1
uint8 CURVE_ID_UTURN = 2


Header header

int64 SysTime

uint8 LaneNum
uint8 RecommendLane
float64 LaneAngle

uint8 LaneChangeFlag
uint8 MissionState
uint8 CrossRoadType
float64 DistanceToNextCross
uint8 TrafficSignFlag
uint8 AlignToLaneCenterFlag

float64 SpeedLimit

uint8 CurveEnterIDFlag
int32 CurveEnterID
uint8 CurveExitIDFlag
int32 CurveExitID

Point2D[] RoadPoints
