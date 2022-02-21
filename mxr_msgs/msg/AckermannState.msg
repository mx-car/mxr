std_msgs/Header header

uint8 FRONT_LEFT  = 0
uint8 FRONT_RIGHT = 1
uint8 BACK_LEFT   = 2
uint8 BACK_RIGHT  = 3

string NA="na"
string PWM="pwm"
string VELOCITY="velocity"

bool[4]    coubled  [false, false, false, false]
float64[4] anlge    [0, 0, 0, 0]
float64[4] velocity [0, 0, 0, 0]
float64[4] steering [0, 0, 0, 0]
string     mode     NA
