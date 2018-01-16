uint8 MANUAL_CLOSED_LOOP_MODE = 0
uint8 AUTO_DRIVE_MODE = 1
uint8 MANUAL_OPEN_LOOP_MODE = 2
uint8 STOP_MODE = 3

uint8 GEAR_P = 0
uint8 GEAR_R = 1
uint8 GEAR_N = 2
uint8 GEAR_D = 3
uint8 GEAR_S = 4
uint8 GEAR_1 = 5
uint8 GEAR_2 = 6
uint8 GEAR_3 = 7
uint8 GEAR_4 = 8
uint8 GEAR_5 = 9
uint8 GEAR_6 = 10
uint8 GEAR_7 = 11
uint8 GEAR_8 = 12
uint8 GEAR_9 = 13

uint8 TURN_LIGHT_NONE = 0
uint8 TURN_LIGHT_LEFT = 1
uint8 TURN_LIGHT_RIGHT = 2

uint8 HORN_OFF = 0
uint8 HORN_ON = 1

Header header

uint8 Mode

float64 GivenSpeed
float64 SteerAngle

float64 Deceleration
float64 Torque
uint8 Gear

float64 SpeedLimit
Point2D[] RoadPoints

uint8 TurnLight
uint8 Horn
