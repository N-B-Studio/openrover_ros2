This is the README I would put in your GitHub repository. It assumes a user has already cloned your ROS2 car project and has successfully built both the Rock 5C and PC workspaces.

---

# ROS 2 Differential Drive Robot Navigation (LD06 + Nav2)

This project demonstrates a complete ROS 2 navigation pipeline using:

* Rock 5C onboard computer
* STM32 differential-drive controller
* LD06 2D LiDAR
* ROS 2 Humble
* SLAM Toolbox
* Nav2
* RViz2

The robot publishes all sensor data over WiFi while all heavy computation runs on a desktop PC.

## System Architecture

```
                 WiFi DDS

        Rock 5C
+----------------------------+
| Base Driver                |
| Robot State Publisher      |
| LD06 Driver                |
| Static TF                  |
+-------------+--------------+
              |
        /scan /odom /tf
              |
=============================
              |
        Desktop PC
+----------------------------+
| SLAM Toolbox               |
| RViz2                      |
| Map Server                 |
| AMCL                       |
| Nav2                       |
| Teleop                     |
+----------------------------+
```

The robot only performs low-level hardware control.

The desktop PC performs:

* Mapping
* Localization
* Global planning
* Local planning
* Navigation
* Visualization

---

# Requirements

## Robot

* Rock 5C
* ROS2 Humble
* LD06 LiDAR
* STM32 base controller
* WiFi connection

## PC

Ubuntu 22.04

ROS2 Humble

Packages

```
slam_toolbox
navigation2
nav2_bringup
rviz2
teleop_twist_keyboard
```

---

# Coordinate Frames

```
map
 └── odom
      └── base_link
            ├── laser_frame
            ├── camera_link
            ├── left_wheel
            └── right_wheel
```

Robot origin:

```
base_link
```

LD06 position:

```
X = -0.007 m
Y = 0
Z = 0.125 m
```

---

# Step 1 — Start Robot

Terminal 1

Launch base driver.

```bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch base_driver bringup.launch.py
```

---

Terminal 2

Launch LD06.

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch ldlidar ldlidar.launch.py \
serial_port:=/dev/ttyS6 \
lidar_frame:=laser_frame \
topic_name:=scan
```

---

Terminal 3

Publish the static transform.

```bash
source ~/ros2_humble/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
--x -0.007 \
--y 0 \
--z 0.125 \
--roll 0 \
--pitch 0 \
--yaw 0 \
--frame-id base_link \
--child-frame-id laser_frame
```

Verify

```
/scan
/odom
/tf
```

---

# Step 2 — Build a Map

Desktop PC

Terminal 1

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=false \
slam_params_file:=~/workspace/ros_car_demo/config/slam_params.yaml
```

---

Terminal 2

```bash
rviz2
```

Set

```
Fixed Frame = map
```

Add

```
Map
LaserScan
RobotModel
TF
Odometry
```

---

Terminal 3

Drive robot.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive around slowly until the map is complete.

---

# Step 3 — Save the Map

Stop driving.

Open another terminal.

```bash
source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_saver_cli \
-f ~/workspace/ros_car_demo/maps/room_map
```

Generated files

```
room_map.yaml
room_map.pgm
```

Verify

```
ls maps
```

---

# Step 4 — Stop SLAM

Stop

```
SLAM Toolbox
RViz
Teleop
```

The saved map will now be used for localization.

---

# Step 5 — Load the Map

Terminal 1

```bash
source /opt/ros/humble/setup.bash

ros2 launch nav2_bringup localization_launch.py \
map:=~/workspace/ros_car_demo/maps/room_map.yaml \
params_file:=~/workspace/ros_car_demo/config/localization_params.yaml \
use_sim_time:=false \
autostart:=true
```

Verify

```
ros2 node list
```

Should contain

```
map_server
amcl
```

---

# Step 6 — Start Navigation

Open another terminal.

```bash
source /opt/ros/humble/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=false \
autostart:=true \
params_file:=~/workspace/ros_car_demo/config/nav2_params.yaml
```

Verify

```
planner_server
controller_server
bt_navigator
behavior_server
```

are running.

---

# Step 7 — Open RViz

```bash
rviz2 \
-d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Set

```
Fixed Frame = map
```

Add if necessary

```
Map
LaserScan
TF
RobotModel
Global Costmap
Local Costmap
```

---

# Step 8 — Initial Localization

Use

```
2D Pose Estimate
```

Click the robot location on the map.

Drag to match the robot heading.

After a few seconds,

LaserScan should align with walls.

AMCL particles should converge.

---

# Step 9 — Navigate

Click

```
Nav2 Goal
```

(or **2D Goal Pose** depending on the RViz configuration.)

Choose a destination.

Nav2 will automatically

* plan a path
* avoid obstacles
* publish `/cmd_vel`
* drive the robot to the goal

---

# Verify Navigation

Robot

```
ros2 topic hz /scan
```

should be approximately

```
10 Hz
```

Robot odometry

```
ros2 topic hz /odom
```

should be approximately

```
44 Hz
```

Navigation

```
ros2 topic echo /cmd_vel
```

should publish velocity commands while navigating.

TF Tree

```
map
 └── odom
      └── base_link
            └── laser_frame
```

---

# Project Structure

```
ros_car_demo/

├── config/
│   ├── slam_params.yaml
│   ├── localization_params.yaml
│   └── nav2_params.yaml
│
├── maps/
│   ├── room_map.yaml
│   └── room_map.pgm
│
└── README.md
```

---

# Current Features

* ROS2 Differential Drive Robot
* WiFi DDS Communication
* LD06 LiDAR Driver
* Static TF Configuration
* Robot URDF
* Odometry
* SLAM Toolbox Mapping
* Map Saving
* AMCL Localization
* Nav2 Autonomous Navigation
* RViz2 Visualization

---

## Future Roadmap

* Camera streaming over WiFi (Rock 5C → PC)
* YOLO object detection on desktop GPU
* Automatic object following
* Loop closure optimization
* Multi-floor mapping
* Fast-LIO2 with Livox Mid-360
* D455 RGB point cloud colorization
* Autonomous exploration
* GPS waypoint navigation (outdoor)
* Migration to PX4 autonomous mobile robot/UAV platform

This README reflects the final architecture you built during the course: **the Rock 5C is responsible only for real-time sensing and control, while the desktop PC performs SLAM, localization, planning, AI, and visualization.** This is also the architecture commonly used on larger research robots where compute-intensive workloads are offloaded from the embedded computer.
