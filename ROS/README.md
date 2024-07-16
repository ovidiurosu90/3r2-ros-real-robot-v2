# **ROS Setup**

## TODO [Tune ROS navigation parameters](https://kaiyuzheng.me/documents/navguide.pdf)

## Simulations

### Simple test in empty world
```bash
roslaunch ros_real_robot_v2_gazebo empty_world.launch
roslaunch ros_real_robot_v2_bringup model.launch

# rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosrun ros_real_robot_v2_teleop ros_real_robot_v2_teleop_key
```

### Creating a map with gmapping
```bash
roslaunch ros_real_robot_v2_gazebo turtlebot3_house.launch
roslaunch ros_real_robot_v2_slam slam.launch

rosrun ros_real_robot_v2_teleop ros_real_robot_v2_teleop_key

roscd ros_real_robot_v2_navigation/maps/
rosrun map_server map_saver -f turtlebot3_house
```

### Navigating on a known map
```bash
roslaunch ros_real_robot_v2_gazebo turtlebot3_house.launch
roslaunch ros_real_robot_v2_navigation navigation.launch map_file:=$(rospack find ros_real_robot_v2_navigation)/maps/turtlebot3_house.yaml

# Rotate until the number of particles is small and they are under or close to the robot
# After the pose estimate is okay, close this node, otherwise it will conflict with the nevigation stack
rosrun ros_real_robot_v2_teleop ros_real_robot_v2_teleop_key
```

#### Note known error with roswtf

While runing the navigation stack, roswtf shows the following error:

```bash
roswtf

ERROR The following nodes should be connected but aren't:
 * /move_base->/move_base (/move_base/global_costmap/footprint)
 * /move_base->/move_base (/move_base/local_costmap/footprint)
```

You can ignore it as it is being shown while running the navigation stack on turtlebot3 as well.

`rqt_graph` && `rosrun rqt_tf_tree rqt_tf_tree` are showing similar results with turtlebot3 in simulation.



## Real World

### Known IPs
- XavierNX:    192.168.50.108
- Desktop:     192.168.50.136
- Leptop:      192.168.50.198

### Network Test

```bash
# On Ubuntu (0)
export ROS_HOSTNAME=192.168.50.136
export ROS_MASTER_URI=http://192.168.50.136:11311
export ROS_IP=192.168.50.136
roscore

# On XavierNX (0)
export ROS_MASTER_URI=http://192.168.50.136:11311
export ROS_IP=192.168.50.108
rosrun rospy_tutorials listener.py

# On Ubuntu (1)
export ROS_HOSTNAME=192.168.50.136
export ROS_MASTER_URI=http://192.168.50.136:11311
export ROS_IP=192.168.50.136
rosrun rospy_tutorials talker.py

# You should see data being sent between two nodes
```

### Network Setup
```bash
# On Ubuntu (0)
vim ~/.bashrc
#########################
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=192.168.50.136
export ROS_MASTER_URI=http://192.168.50.136:11311
export ROS_IP=192.168.50.136
#########################
source ~/.bashrc

# On XavierNX (0)
vim ~/.bashrc
#########################
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.50.136:11311
export ROS_IP=192.168.50.108
#########################
source ~/.bashrc
```

### Building the map
```bash
# On Ubuntu (0)
roscore

# On XavierNX (0)
roslaunch ros_real_robot_v2_bringup robot.launch

# On Ubuntu (1)
roslaunch ros_real_robot_v2_slam slam.launch

# On Ubuntu (2)
rosrun ros_real_robot_v2_teleop ros_real_robot_v2_teleop_key
# Move around until the map is complete

# On Ubuntu (3)
roscd ros_real_robot_v2_navigation/maps/
rosrun map_server map_saver -f apartment1
```

## Navigate the known map
```bash
# On Ubuntu (0)
roscore

# On XavierNX (0)
roslaunch ros_real_robot_v2_bringup robot.launch

# On Ubuntu (1)
roslaunch ros_real_robot_v2_navigation navigation.launch
# roslaunch ros_real_robot_v2_navigation navigation.launch map_file:=/home/[USERNAME]/catkin_ws/src/ros_real_robot_v2/ros_real_robot_v2_navigation/maps/apartment1.yaml

# On Ubuntu (2): Check if everything is fine
cd ~/Desktop/
rosrun tf view_frames && evince frames.pdf
```


# Adafruit BNO055 IMU

## View IMU

```bash
# On Ubuntu (0)
roscore

# On XavierNX (0)
# roslaunch ros_real_robot_v2_bringup imu.launch # IMU operation_mode
# roslaunch ros_real_robot_v2_bringup imu.launch operation_mode:=NDOF_FMC_OFF
roslaunch ros_real_robot_v2_bringup imu.launch operation_mode:=NDOF

# On Ubuntu (1)
roslaunch ros_real_robot_v2_utils view_imu.launch

# On XavierNX (1)
rosservice call /imu/reset_device "{}" # Reset orientation of the axis perpendicular to the ground plane
rosservice call /imu/calibration_status "{}"
```


## Calibration IMU

```bash
# On Ubuntu (0)
roscore

# On XavierNX (0)
roscd ros_imu_bno055/src # this is where the calibration files will be stored
# rm IMU_calibration NDOF_FMC_OFF_calibration
roslaunch ros_imu_bno055 imu_calibration.launch serial_port:=/dev/O_ttyUSB1 operation_mode:=IMU
roslaunch ros_imu_bno055 imu_calibration.launch serial_port:=/dev/O_ttyUSB1 operation_mode:=NDOF_FMC_OFF
roslaunch ros_imu_bno055 imu_calibration.launch serial_port:=/dev/O_ttyUSB1 operation_mode:=NDOF
```


## Sensor Fusion

```bash
# On Ubuntu (0)
roscore

# On XavierNX (0)
roslaunch ros_real_robot_v2_bringup robot.launch

# On Ubuntu (1)
roslaunch ros_real_robot_v2_bringup model.launch
# Add rviz_imu_plugin (Create visualization by display type), select topic /imu/data, enable box

# On Ubuntu (2)
rosrun ros_real_robot_v2_teleop ros_real_robot_v2_teleop_key
```


# Run tests

## cmd_vel_odom_test0 in simulation (Feb 05, 2022)
```bash
roslaunch ros_real_robot_v2_gazebo empty_world.launch
roslaunch ros_real_robot_v2_bringup model.launch
rosrun ros_real_robot_v2_utils cmd_vel_odom_test0.py

# Results (3 tries, ~200 odom msgs): linearError% between(-0.23, 1.55) and angularError% between(1.30, 3.13)
```

## cmd_vel_odom_test0 in the real world (Feb 05, 2022)
```bash
# On Ubuntu (0)
roscore

# On XavierNX (0)
roslaunch ros_real_robot_v2_bringup robot.launch

# On Ubuntu (1)
roslaunch ros_real_robot_v2_bringup model.launch

# On Ubuntu (2)
rosrun ros_real_robot_v2_utils cmd_vel_odom_test0.py

# Results (4 tries, ~360 odom msgs): linearError% between(-9.55, -10.12) and angularError% between(-9.93, -10.23)
```

## cmd_vel_odom_test0 in the real world (Feb 12, 2022)
```bash
# The commands are the same as above

# Results (3 tries, ~200 odom msgs): linearError% between(-0.19, 0.26) and angularError% between(-0.09, 0.11)
# The inconsistency is now fixed.
# The error was that the code was trying to maintain a transfer rate of 100Hz but it wasnâ€™t able to.
# The calculations were implying that, which was wrong. Using the correct cycle time (instead of the inferred one) fixed the issue.
```

