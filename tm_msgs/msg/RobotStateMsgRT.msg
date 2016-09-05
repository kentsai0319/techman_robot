# Data structure for the realtime communications interface
# used by the Techman Robot controller
# 
# This data structure is send at 100 Hz on TCP port 6188

Header header
float64 time
float64 controller_time
float64[] q_command
float64[] qd_command
float64[] qt_command
float64[] q_actual
float64[] qd_actual
float64[] qt_actual
float64[] tool0_pos_command
float64[] tool0_pos_actual
float64[] tool_pos_command
float64[] tool_pos_actual
float64[] tcp_speed
float64[] tcp_force
int32[] kine_config
float64 common_speed_ratio
int32 robot_mode
int32 safety_mode
int32 teach_mode
int32 control_mode
int32 error_code_0
int32 error_code_1