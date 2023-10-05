### motor_control节点说明 ###

### 功能：
该节点控制转向电机、离合电机、制动电机和节气门以及换挡

### 收：
1. steering_angle
2. braking_aim 需要对从主节点和通过通信节点从ECU收到的target作比较，取更大的值控制转向电机
3. throttle_aim 从主节点收
4. clutch_aim
5. inner_state
6. event

### 发：
1. 通过can_msg控制各电机和节气门
2. 发送最终braking_target给通信节点继而发送给Datalogger