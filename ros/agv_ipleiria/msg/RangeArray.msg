# Range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is  appropriate for Bumper, Sonar and Infrared devices sensors.

# This message was created based on the sensor_msgs/Range.msg
# AGV sensor message
# V0.0 - 13/06/2015 - @ESTG - IPLEIRIA
# V1.0 - 18/11/2015 - @ESTG - IPLEIRIA




# Radiation type enums
uint8 ULTRASOUND = 0
uint8 INFRARED = 1
uint8 BUMPER = 2


int8 INACTIVE = 0

Header header



uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]
                        
                        
                        
float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range. 
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        
uint8 ir_res  		# ir resolution value   
                     
                        
float32[16] ranges        # range data [m]
                        # (Note: values < range_min or > range_max
                        # should be discarded)
                        
                    






