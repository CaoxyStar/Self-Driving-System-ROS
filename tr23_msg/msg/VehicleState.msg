# vehicleSpeed
# the vehicle speed value estimated from the four wheel speeds.
# unit : km/h
float32 vehicleSpeed

# steeringWheelAngle
# the angle of the steering wheel measured from the angle sensor
# a positive value indicates that the wheel is turning counter-clockwise.
# unit : deg
float32 steeringWheelAngle

# gear
# actual number of the gear
# the gear value coincides with the enumeration value in M1 GPR,
# in which value 0 indicates that the gearbox is in neutral, 
# value -1 indicates that the gear is currently unknown due to sensor failure.
int16 GEAR_DEFAULT = -1
int16 GEAR_NEUTRAL = 0
int16 GEAR_FIRST = 1
int16 GEAR_SECOND = 2
int16 GEAR_THIRD = 3
int16 GEAR_FOURTH = 4
int16 GEAR_FIFTH = 5
int16 GEAR_SIXTH = 6
int16 GEAR_SENVENTH = 7
int16 gear
