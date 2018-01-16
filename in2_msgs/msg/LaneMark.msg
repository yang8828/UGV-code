uint8 TYPE_NONE = 0
uint8 TYPE_UNKNOWN = 1
uint8 TYPE_DASHED_WHITE = 2
uint8 TYPE_SOLID_WHITE = 3
uint8 TYPE_DASHED_YELLOW = 4
uint8 TYPE_SOLID_YELLOW = 5
uint8 TYPE_STOP = 6

uint8 IS_CURVE_FALSE = 0
uint8 IS_CURVE_TRUE = 1

float64 r
float64 theta
uint8 type

# For double lanemark. The right side lanemark is the secondary lanemark. For single lanemark, set secondary_type = TYPE_NONE.
uint8 secondary_type

float64 confidence
uint8 is_curve
geometry_msgs/Point32[] points
