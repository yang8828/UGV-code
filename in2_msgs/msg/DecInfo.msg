uint8 MANUAL_CLOSED_LOOP_MODE = 0
uint8 AUTO_DRIVE_MODE = 1
uint8 MANUAL_OPEN_LOOP_MODE = 2
uint8 STOP_MODE = 3

uint8 TURN_LIGHT_NONE = 0
uint8 TURN_LIGHT_LEFT = 1
uint8 TURN_LIGHT_RIGHT = 2

uint8 HORN_OFF = 0
uint8 HORN_ON = 1

Header header

int64 SysTime

float64 SpeedLimit

uint8 Mode

Point2D[] RoadPoints

uint8 TurnLight
uint8 Horn
