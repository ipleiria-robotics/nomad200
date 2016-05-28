# AGV command message. This contains information such as desired linear and angular velocity and Turret position.
# V0.0 - 10/08/2015 - @ESTG - IPLEIRIA
# V1.0 - 22/09/2015 - @ESTG - IPLEIRIA
# V1.1 - 18/11/2015 - @ESTG - IPLEIRIA
# V1.2 - 25/11/2015 - @ESTG - IPLEIRIA
# V2.0 - 18/12/2015 - @ESTG - IPLEIRIA


Header header
#geometry_msgs/Twist Base
#geometry_msgs/Quaternion Turret

#twist  - reduced version
float32 linear
float32 angular

#pose  - reduced version
float32 turret_angle

 
