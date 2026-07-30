# RoboNavLab DLC — LD06 2D SLAM & Autonomous Exploration

This DLC extends the frozen Stage 1 mobile base with an LD06 2D LiDAR, `slam_toolbox`, Nav2, and a frontier-exploration node.

## Final ROS 2 Architecture

```text
LD06
  |
  v
/scan ----------------------+
                            |
Stage 1 base                v
/cmd_vel <------------- Nav2 / Explorer
/odom ------------------+   |
/tf                     |   |
                        v   v
                    slam_toolbox
                        |
                        v
                      /map
                        |
                        +----> map_saver_cli
```

Target TF tree:

```text
map
└── odom
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        └── laser_frame
```

## Environment

Rock 5C:

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

Ubuntu workstation:

```bash
source /opt/ros/<your-distro>/setup.bash
export ROS_DOMAIN_ID=42
```

## Start the Robot Base

On the Rock 5C:

```bash
ros2 launch ros_car_bringup car.launch.py
```

Verify:

```bash
ros2 topic hz /odom
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo odom base_link
```

## Start the LD06 Driver

Run:
```bash
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
```

The final DLC will expose the LiDAR as:

```text
/scan
```

Verify:

```bash
ros2 topic list | grep scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

The `LaserScan.header.frame_id` must match the LiDAR TF frame, for example:

```text
laser_frame
```

## Start 2D SLAM

After the DLC packages and configuration are built:

```bash
ros2 launch ros_car_navigation slam.launch.py
```

Verify:

```bash
ros2 topic list | grep map
ros2 run tf2_ros tf2_echo map odom
```

Expected important topics:

```text
/scan
/odom
/map
/map_metadata
/tf
/tf_static
```

## Manual Mapping Test

Use keyboard teleoperation from the workstation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly around the room and verify that the occupancy grid grows in RViz.

Recommended RViz displays:

```text
Map        -> /map
LaserScan  -> /scan
RobotModel
TF
Odometry
```

Set:

```text
Fixed Frame = map
```

## Save the Map Locally

Create a map directory:

```bash
mkdir -p ~/workspace/ros2_ws/maps
```

Save the current occupancy map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/workspace/ros2_ws/maps/room
```

Expected files:

```text
~/workspace/ros2_ws/maps/room.yaml
~/workspace/ros2_ws/maps/room.pgm
```

## Start Autonomous Exploration

After the exploration lesson is complete:

```bash
ros2 launch ros_car_navigation explore.launch.py
```

The exploration node will:

1. Read the current occupancy grid.
2. Find frontiers between known free space and unknown space.
3. Select a reachable frontier.
4. Send a Nav2 `NavigateToPose` goal.
5. Wait for the robot to reach or fail the goal.
6. Select the next frontier.
7. Stop when no useful frontiers remain.

Keep an emergency-stop terminal ready:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

For first autonomous tests, keep the robot in a clear room and use conservative speed limits.

## Save the Automatically Built Map

When exploration is finished:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/workspace/ros2_ws/maps/room_auto
```

## Basic Health Checks

```bash
ros2 node list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo map odom
```

## Expected Final Packages

```text
ros_car_base/          # Stage 1 base driver
ros_car_description/   # URDF / TF
ros_car_bringup/       # Stage 1 bringup
ros_car_lidar/         # LD06 launch/config
ros_car_navigation/    # slam_toolbox + Nav2 + map save
ros_car_exploration/   # frontier explorer
```

## Final Demo

```text
Power on robot
    |
    v
Stage 1 bringup
    |
    v
LD06 /scan
    |
    v
slam_toolbox
    |
    v
Nav2 + frontier exploration
    |
    v
Robot autonomously explores room
    |
    v
room_auto.yaml + room_auto.pgm
```