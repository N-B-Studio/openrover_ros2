====================== Rock 5c: =====================
## terminal 1:
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch ros_car_navigation explore.launch.py

## terminal 2:
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run tf2_ros static_transform_publisher \
  --x -0.007 \
  --y 0.0 \
  --z 0.125 \
  --roll 0 \
  --pitch 0 \
  --yaw 0 \
  --frame-id base_link \
  --child-frame-id laser_frame

## terminal 3:
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch ldlidar ldlidar.launch.py \
  serial_port:=/dev/ttyS6 \
  lidar_frame:=laser_frame \
  topic_name:=scan
## terminal 4:
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 node list
ros2 topic hz /scan

=========================== PC: ===========================
## terminal 1:
# Srouce
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO

# set network
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# restart daemon
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2

# check scan hz, should be 10 hz
ros2 topic hz /scan

# check node list
ros2 node list

# install SLAM Toolbox
ros2 pkg list | grep slam_toolbox

# install if no output:
sudo apt update
sudo apt install -y ros-humble-slam-toolbox

# create SLAM setting:
mkdir -p ~/workspace/ros_car_demo/config
# create ymal
nano ~/workspace/ros_car_demo/config/slam_params.yaml
# add:
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    mode: mapping
    use_sim_time: false

    use_map_saver: true
    debug_logging: false
    throttle_scans: 1

    transform_publish_period: 0.05
    map_update_interval: 1.0
    resolution: 0.05

    min_laser_range: 0.05
    max_laser_range: 12.0

    minimum_time_interval: 0.10
    transform_timeout: 0.5
    tf_buffer_duration: 30.0

    stack_size_to_use: 40000000
    enable_interactive_mode: true

    use_scan_matching: true
    use_scan_barycenter: true

    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05

    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0

    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
## terminal 2:
# Start SLAM
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/workspace/ros_car_demo/config/slam_params.yaml

# should include:
Using solver plugin solver_plugins::CeresSolver

## terminal 3:
# confirm SLAM
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
# check
ros2 topic list | grep -E "map|scan"
# should show:
/map
/map_metadata
/scan

# check lidar topic:
ros2 topic info /scan -v

# should be:
Publisher count: 1
Subscription count: 1

# check TF:
ros2 run tf2_ros tf2_echo map odom

* should be:
* map -> odom

## terminal 4:

# start RViz2
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

rviz2

# rviz2 setting:
* Fixed Frame = map

add:

* Map
* LaserScan
* RobotModel
* TF
* Odometry
* Grid

Topics：
* Map        = /map
* LaserScan  = /scan
* Odometry   = /odom

## Terminal 5: Keyboard controll

# install: 
sudo apt install -y ros-humble-teleop-twist-keyboard

# start:
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 run teleop_twist_keyboard teleop_twist_keyboard


## Rebuild map:
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/workspace/ros_car_demo/config/slam_params.yaml

## save map:
地图满意后，在 laptop 执行：

mkdir -p ~/workspace/ros_car_demo/maps

保存：

ros2 run nav2_map_server map_saver_cli \
  -f ~/workspace/ros_car_demo/maps/room_map

如果没有 nav2_map_server：

sudo apt install -y ros-humble-nav2-map-server

生成：

~/workspace/ros_car_demo/maps/room_map.yaml
~/workspace/ros_car_demo/maps/room_map.pgm