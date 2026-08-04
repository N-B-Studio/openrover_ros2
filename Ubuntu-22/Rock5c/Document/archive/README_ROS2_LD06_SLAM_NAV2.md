# ROS 2 Distributed Mobile Robot: LD06 SLAM and Nav2

This project runs a differential-drive robot as a distributed ROS 2 system.

- **Rock 5C / ROS 2 Humble** runs the hardware-facing robot stack.
- **Ubuntu PC / ROS 2 Jazzy** runs SLAM Toolbox, Nav2, and RViz2.
- The LD06 publishes `/scan`.
- The robot base publishes `/odom` and `odom -> base_link`.
- The PC builds the map and sends velocity commands back through `/cmd_vel`.

> This README is written so that a new user can clone the repository, install dependencies, build the workspace, launch the robot, create a map, and test navigation.

---

## 1. System Architecture

```text
┌───────────────────────────────┐
│ Rock 5C — ROS 2 Humble        │
│                               │
│ ros_car_base                  │
│   └── base_driver             │
│                               │
│ ros_car_description           │
│   └── robot_state_publisher   │
│                               │
│ LD06 driver                   │
│   └── publishes /scan         │
│                               │
│ TF publishers                 │
│   ├── odom -> base_link       │
│   └── base_link -> laser_frame│
│                               │
│ subscribes /cmd_vel           │
└───────────────┬───────────────┘
                │
                │ ROS 2 DDS over LAN
                │ ROS_DOMAIN_ID=42
                │
┌───────────────▼───────────────┐
│ Ubuntu PC — ROS 2 Jazzy       │
│                               │
│ SLAM Toolbox                  │
│   └── map -> odom             │
│                               │
│ Nav2                          │
│   └── publishes /cmd_vel      │
│                               │
│ RViz2                         │
│   └── visualization           │
└───────────────────────────────┘
```

Final TF tree:

```text
map
└── odom
    └── base_link
        ├── laser_frame
        ├── left_wheel_link
        └── right_wheel_link
```

Frame ownership:

| Transform | Publisher |
|---|---|
| `map -> odom` | SLAM Toolbox on the PC |
| `odom -> base_link` | `base_driver` on the Rock 5C |
| `base_link -> laser_frame` | robot description or static TF publisher |
| wheel transforms | `robot_state_publisher` |

---

## 2. Hardware and Software

### Robot computer

- Radxa Rock 5C
- Debian 12
- ROS 2 Humble workspace built from source
- STM32 motor controller
- Two CAN FOC motors
- LD06 2D LiDAR

### Workstation

- Ubuntu 24.04
- ROS 2 Jazzy
- RViz2
- SLAM Toolbox
- Navigation2

### Current robot geometry

```text
Wheel diameter:   0.062 m
Wheel separation: 0.122 m
```

LD06 transform:

```text
Parent frame: base_link
Child frame:  laser_frame

x:    -0.007 m
y:     0.000 m
z:     0.125 m
roll:  0.000 rad
pitch: 0.000 rad
yaw:   0.000 rad
```

---

## 3. Repository Structure

The repository should contain the following ROS 2 packages:

```text
ros2_ws/
├── src/
│   ├── ros_car_base/
│   ├── ros_car_description/
│   ├── ros_car_bringup/
│   ├── ros_car_slam/
│   └── ldlidar/
├── build/
├── install/
└── log/
```

Recommended package responsibilities:

```text
ros_car_base
└── serial/CAN base driver, odometry, /cmd_vel handling

ros_car_description
└── URDF/Xacro, robot geometry, wheel links, sensor links

ros_car_bringup
└── Rock-side launch files for base, description, TF, and LD06

ros_car_slam
└── PC-side SLAM parameters, Nav2 parameters, RViz config, maps

ldlidar
└── LD06 hardware driver
```

The final Rock-side launch command in this README assumes this launch file exists:

```text
ros_car_bringup/launch/robot.launch.py
```

It should start:

```text
base_driver
robot_state_publisher
LD06 driver
base_link -> laser_frame transform
```

---

# Part A — Clone and Build the Rock 5C Workspace

## 4. Install build tools on the Rock 5C

```bash
sudo apt update

sudo apt install -y \
  git \
  python3-pip \
  python3-rosdep \
  python3-colcon-common-extensions
```

Initialize `rosdep` if needed:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` reports that the sources already exist, continue with:

```bash
rosdep update
```

## 5. Clone the repository

```bash
mkdir -p ~/workspace
cd ~/workspace

git clone <YOUR_GITHUB_REPOSITORY_URL> ros2_ws
cd ~/workspace/ros2_ws
```

Expected repository root:

```text
src
README.md
```

## 6. Source ROS 2 Humble on Rock 5C

```bash
source ~/ros2_humble/install/setup.bash
```

Verify:

```bash
printenv ROS_DISTRO
```

Expected:

```text
humble
```

## 7. Install dependencies

```bash
cd ~/workspace/ros2_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r \
  -y
```

## 8. Build the Rock workspace

```bash
cd ~/workspace/ros2_ws

source ~/ros2_humble/install/setup.bash

colcon build --symlink-install
```

Source the overlay:

```bash
source ~/workspace/ros2_ws/install/setup.bash
```

Verify packages:

```bash
ros2 pkg list | grep -E \
'ros_car_base|ros_car_description|ros_car_bringup|ldlidar'
```

## 9. Configure hardware permissions

Current project examples:

```text
Base controller: /dev/ttyS4
LD06:            /dev/ttyS6
```

Check devices:

```bash
ls -l /dev/ttyS*
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
```

Add the user to `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in.

Temporary permission test only:

```bash
sudo chmod 666 /dev/ttyS4
sudo chmod 666 /dev/ttyS6
```

Use a `udev` rule for a permanent deployment.

---

# Part B — Configure ROS 2 Networking

## 10. Put both machines on the same network

Rock 5C:

```bash
hostname -I
```

PC:

```bash
ping <ROCK_5C_IP>
```

## 11. Use the same ROS domain

Run on both machines:

```bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Optional persistent configuration:

```bash
cat >> ~/.bashrc <<'EOF2'

# ROS 2 distributed robot network
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
EOF2
```

Reload:

```bash
source ~/.bashrc
```

Verify:

```bash
echo "$ROS_DOMAIN_ID"
echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"
```

Expected:

```text
42
SUBNET
```

---

# Part C — Start the Robot on Rock 5C

## 12. Start the complete Rock bringup

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_bringup robot.launch.py
```

This bringup should start:

```text
/base_driver
/robot_state_publisher
/ldlidar
/static_transform_publisher
```

Expected topics:

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
/robot_description
/joint_states
```

## 13. Manual fallback

Use this only until `robot.launch.py` is complete.

### Terminal R1 — base driver

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run ros_car_base base_driver
```

Find the real executable if needed:

```bash
ros2 pkg executables ros_car_base
```

### Terminal R2 — robot description

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_description display.launch.py
```

If the launch name differs:

```bash
find ~/workspace/ros2_ws/src/ros_car_description -maxdepth 3 -type f
```

### Terminal R3 — LD06

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ldlidar ldlidar.launch.py \
  serial_port:=/dev/ttyS6 \
  lidar_frame:=laser_frame \
  topic_name:=scan
```

### Terminal R4 — temporary LD06 TF

Use only if the transform is not already in the URDF or bringup launch:

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run tf2_ros static_transform_publisher \
  --x -0.007 \
  --y 0.0 \
  --z 0.125 \
  --roll 0.0 \
  --pitch 0.0 \
  --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id laser_frame
```

Do not publish the same transform twice.

## 14. Verify the Rock-side stack

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Nodes:

```bash
ros2 node list
```

LiDAR rate:

```bash
ros2 topic hz /scan
```

Expected: approximately `10 Hz`.

Odometry rate:

```bash
ros2 topic hz /odom
```

Expected: approximately `44 Hz`.

Complete TF:

```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```

Expected chain:

```text
odom -> base_link -> laser_frame
```

An initial missing-frame warning can occur during discovery. Valid transforms must appear shortly afterwards.

---

# Part D — Prepare the Ubuntu PC

## 15. Install PC dependencies

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup
```

## 16. Clone and build the PC workspace

```bash
mkdir -p ~/workspace
cd ~/workspace

git clone <YOUR_GITHUB_REPOSITORY_URL> ros2_ws
cd ~/workspace/ros2_ws
```

Install dependencies:

```bash
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r \
  -y
```

Build workstation packages:

```bash
colcon build \
  --symlink-install \
  --packages-select \
  ros_car_description \
  ros_car_slam
```

Source:

```bash
source ~/workspace/ros2_ws/install/setup.bash
```

If every package supports Jazzy:

```bash
colcon build --symlink-install
```

## 17. Verify Rock data on the PC

```bash
source /opt/ros/jazzy/setup.bash

if [ -f ~/workspace/ros2_ws/install/setup.bash ]; then
  source ~/workspace/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Restart the daemon under the correct domain:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2
```

Check:

```bash
ros2 node list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom laser_frame
```

Do not start SLAM until these pass:

```text
/scan: approximately 10 Hz
/odom: approximately 44 Hz
odom -> laser_frame: available
```

---

# Part E — Configure and Run SLAM Toolbox

## 18. SLAM parameters

Expected path:

```text
ros_car_slam/config/slam_params.yaml
```

Recommended baseline:

```yaml
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

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349

    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5

    use_response_expansion: true
```

## 19. Start SLAM Toolbox

Preferred repository launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_slam slam.launch.py
```

Direct fallback:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/workspace/ros2_ws/src/ros_car_slam/config/slam_params.yaml
```

## 20. Verify SLAM

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Check subscription:

```bash
ros2 topic info /scan -v
```

Check map:

```bash
ros2 topic hz /map
```

Check SLAM TF:

```bash
ros2 run tf2_ros tf2_echo map odom
```

SLAM is operational only when:

```text
map -> odom exists
/map publishes
```

---

# Part F — Run RViz2

## 21. Start RViz2

Preferred project config:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

rviz2 \
  -d ~/workspace/ros2_ws/src/ros_car_slam/rviz/mapping.rviz
```

Generic startup:

```bash
rviz2
```

Set:

```text
Fixed Frame: map
```

Add:

| Display | Topic |
|---|---|
| Map | `/map` |
| LaserScan | `/scan` |
| Odometry | `/odom` |
| RobotModel | `/robot_description` |
| TF | no topic selection required |

If `map` does not exist yet, temporarily set:

```text
Fixed Frame: odom
```

Switch back to `map` after `map -> odom` appears.

---

# Part G — Build a Map

## 22. Recommended speeds

```text
Linear speed:  0.10–0.15 m/s
Angular speed: 0.20–0.30 rad/s
```

## 23. Recommended mapping route

1. Start all nodes.
2. Keep the robot stationary for 5–10 seconds.
3. Drive forward slowly.
4. Avoid sudden acceleration.
5. Rotate slowly.
6. Follow room boundaries.
7. Avoid repeatedly spinning in place.
8. Return close to the starting position.
9. Continue slowly until loop closure corrects drift.
10. Stop before saving.

Avoid:

```text
fast rotation
wheel slip
collisions
lifting the robot
driving over cables
rapid forward/reverse changes
moving people near the LiDAR
```

---

# Part H — Rebuild an Unsatisfactory Map

## 24. Start a clean mapping session

Stop SLAM Toolbox:

```text
Ctrl+C
```

Restart it:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_slam slam.launch.py
```

Or use the direct command:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/workspace/ros2_ws/src/ros_car_slam/config/slam_params.yaml
```

A fresh process starts a new empty map. RViz may display the old map briefly until new `/map` messages arrive.

## 25. Delete an old saved map

```bash
rm -f ~/workspace/ros2_ws/src/ros_car_slam/maps/room_map.*
```

## 26. Keep multiple attempts

```text
room_map_01
room_map_02
room_map_03
```

This prevents a new test from overwriting a good map.

---

# Part I — Save the Final Map

## 27. Save the map

```bash
mkdir -p ~/workspace/ros2_ws/src/ros_car_slam/maps
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run nav2_map_server map_saver_cli \
  -f ~/workspace/ros2_ws/src/ros_car_slam/maps/room_map
```

Expected files:

```text
room_map.yaml
room_map.pgm
```

Commit:

```bash
cd ~/workspace/ros2_ws

git add src/ros_car_slam/maps/
git commit -m "Add LD06 indoor occupancy map"
git push
```

---

# Part J — Start Nav2 While Mapping

## 28. Safety before Nav2

1. Confirm the base watchdog works.
2. Confirm stale `/cmd_vel` commands stop the motors.
3. Raise the wheels off the ground.
4. Test linear and angular directions.
5. Keep an emergency stop available.
6. Start with a target less than 0.5 m away.

## 29. Start Nav2

Preferred project launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_slam navigation.launch.py
```

Direct fallback:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true
```

The fallback uses Nav2 defaults. A physical robot should use a tuned project-specific parameter file with the correct footprint, speed limits, controller, planner, and costmap settings.

## 30. Verify `/cmd_vel`

```bash
ros2 topic info /cmd_vel -v
```

Expected:

```text
Publisher: Nav2 controller server
Subscriber: Rock 5C base_driver
```

Observe commands:

```bash
ros2 topic echo /cmd_vel
```

Check nodes:

```bash
ros2 node list | grep -E \
'planner|controller|behavior|bt_navigator|costmap|lifecycle'
```

## 31. Send a goal

In RViz:

1. Confirm `Fixed Frame = map`.
2. Select **Nav2 Goal**.
3. Click close to the robot.
4. Drag to set heading.
5. Observe global and local paths.
6. Be ready to stop the robot.

Command path:

```text
RViz Nav2 Goal
    ↓
BT Navigator
    ↓
Planner Server
    ↓
Controller Server
    ↓
/cmd_vel
    ↓
Rock 5C base_driver
    ↓
STM32
    ↓
motor controllers
```

---

# Part K — Final Startup Order

## Rock 5C

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_bringup robot.launch.py
```

Verify:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom laser_frame
```

## PC terminal P1 — SLAM

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_slam slam.launch.py
```

## PC terminal P2 — RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

rviz2 \
  -d ~/workspace/ros2_ws/src/ros_car_slam/rviz/mapping.rviz
```

## PC terminal P3 — Nav2

Start only after SLAM is healthy:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_slam navigation.launch.py
```

---

# Part L — Troubleshooting

## PC receives no Rock topics

```bash
echo "$ROS_DOMAIN_ID"
echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"
```

Expected:

```text
42
SUBNET
```

Restart daemon:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

Check connectivity:

```bash
ping <OTHER_MACHINE_IP>
```

Multicast test:

Machine A:

```bash
ros2 multicast receive
```

Machine B:

```bash
ros2 multicast send
```

## `/scan` exists but RViz shows no laser

```bash
ros2 topic echo /scan --once
```

Confirm:

```text
header.frame_id: laser_frame
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```

RViz:

```text
Fixed Frame: odom
LaserScan topic: /scan
```

## `/odom` exists but TF does not

An odometry topic does not automatically create TF. The base driver must publish:

```text
odom -> base_link
```

Check:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

## `map` frame does not exist

Check:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom laser_frame
ros2 topic info /scan -v
```

Inspect SLAM logs for:

```text
Message Filter dropping message
Failed to compute laser pose
Extrapolation Error
```

Verify:

```yaml
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan
```

## Walls appear doubled

Common causes:

```text
wheel slip
incorrect wheel separation
incorrect wheel diameter
fast rotation
rapid acceleration
incorrect LiDAR yaw
moving objects
poor loop closure
```

Action:

1. Restart SLAM.
2. Move more slowly.
3. Reduce angular speed.
4. Verify wheel geometry.
5. Verify `base_link -> laser_frame`.
6. Repeat the route.

## Map is slightly rotated

This can be normal. The map axes are initialized from the robot pose when SLAM starts. A rotated room outline does not necessarily indicate an error.

A progressively bending or duplicated map is not normal and usually indicates odometry, TF, motion, or scan-matching problems.

## Robot moves in the wrong direction

Stop immediately.

```bash
ros2 topic echo /cmd_vel
```

For positive `linear.x`, the chassis must move forward.

For positive `angular.z`, the chassis must rotate counterclockwise under the ROS coordinate convention.

Correct motor signs before using Nav2.

---

# Part M — Demonstration Checklist

```text
[ ] Rock bringup starts with one command
[ ] /scan is stable at approximately 10 Hz
[ ] /odom is stable at approximately 44 Hz
[ ] odom -> laser_frame is available
[ ] PC receives Rock topics
[ ] SLAM produces /map
[ ] map -> odom is available
[ ] RViz shows robot, laser, map, and TF
[ ] robot can be driven slowly for mapping
[ ] loop closure is visible
[ ] final map is saved
[ ] Nav2 publishes /cmd_vel
[ ] short Nav2 goal succeeds safely
[ ] emergency stop is available
```

Suggested video sequence:

1. Show the physical robot.
2. Start Rock bringup.
3. Show `/scan` and `/odom` rates.
4. Show the TF tree.
5. Start SLAM on the PC.
6. Show live LD06 scans in RViz.
7. Drive around the room.
8. Show loop closure.
9. Save the map.
10. Send a short Nav2 goal.
11. Show the architecture diagram.

---

# References

- ROS 2 workspace and `colcon`:  
  https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

- ROS 2 dependency management with `rosdep`:  
  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html

- Nav2 navigating while mapping:  
  https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

- Nav2 mapping and localization:  
  https://docs.nav2.org/setup_guides/sensors/mapping_localization.html

- Nav2 footprint configuration:  
  https://docs.nav2.org/setup_guides/footprint/setup_footprint.html

---

## Current Project Status

Validated:

```text
Rock /scan: approximately 10 Hz
Rock /odom: approximately 44 Hz
Rock odom -> laser_frame: working

PC /scan: received
PC /odom: received
PC odom -> laser_frame: received

SLAM Toolbox: mapping operational
RViz2: map visualization operational
```

Still required before calling the repository complete:

```text
Create and validate ros_car_bringup/launch/robot.launch.py
Create and validate ros_car_slam/launch/slam.launch.py
Create and tune project-specific Nav2 parameters
Create the correct robot footprint
Store the RViz configuration in the repository
Document the emergency-stop procedure
```
