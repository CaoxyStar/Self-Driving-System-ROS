# throttleAim
# the throttle servo aim value
# 1.0f indicates wide open throttle and 0.0f indicates an idle position.
float32 throttleAim

# clutchAim
# the clucth servo aim value
# 0.0f indicates a fully engaged clutch position and 1.0f represents a fully disengaged clutch position. 
float32 clutchAim   

# gearShiftRequest
# Enumerated value, the value coincide with the Gear Shift Enumeration in M1 GPR Package.
int8 GEAR_SHIFT_IDLE = 0
int8 GEAR_SHIFT_UP = 1
int8 GEAR_SHIFT_DOWN = 2
int8 gearShiftRequest

float32 brakingAim
