# AGV Status message. This contains information such as power status, actual wheel and Turret position.
# V0.0 - 10/08/2015 - @ESTG - IPLEIRIA
# V1.0 - 23/09/2015 - @ESTG - IPLEIRIA
# V1.1 - 18/11/2015 - @ESTG - IPLEIRIA
# V2.0 - 18/12/2015 - @ESTG - IPLEIRIA

Header header
uint8 Motor_power			# bool use: Motors Controllers On/Off
uint8 Motor_ready			# Motors Controllers Ready
uint8 Collision				# bool use: Info on collision event
float32 Battery 			# Wheel Rotation angle

#nav_msgs/Odometry Base
#geometry_msgs/Quaternion Turret


#twist  - reduced version
float32 linear
float32 angular

#pose  - reduced version
float32 pose_x
float32 pose_y
float32 turret_angle
float32 base_angle