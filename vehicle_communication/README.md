### vehicle_communication节点说明

### 功能：
1. 通过can-bus收ECU的消息转化为rosmsg发送给相应节点
2. 通过rosmsg收取无人系统部分状态发送给Datalogger

### 收can：
1. vehicle_speed_ecu 0x640
2. ebs_target 0x739
3. engine_speed 0x64a
4. ebs_monitor 0x740
### 转rosmsg：
1. vehicle_speed
2. braking_target
3. engine_speed
4. ebs_state

### 收rosmag：
# 0x500
1. braking_result
2. steering_angle
3. throttle_aim
4. speed
# 0x502
1. as_state
2. ebs_state
3. event
4. steering_state
5. braking_state
6. lap_counter
7. cones_actual
8. cones_all
### 转can：
can_msg 0x500
can_msg 0x502
