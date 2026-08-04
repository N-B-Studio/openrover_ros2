# ROS 2 Keyboard Teleoperation and RViz Demo

This is the minimal version of the project.

It allows a user to:

- start the robot on the Rock 5C;
- control the car from the PC keyboard;
- view the robot model, odometry, TF, and grid in RViz2;
- run without the LD06 LiDAR, SLAM Toolbox, or Nav2.

---

## 1. Architecture

```text
Rock 5C — ROS 2 Humble
├── base_driver
│   ├── subscribes: /cmd_vel
│   ├── publishes:  /odom
│   └── publishes:  odom -> base_link
│
└── robot_state_publisher
    ├── publishes: /robot_description
    ├── publishes: /tf
    └── publishes: /tf_static

             ROS 2 DDS over LAN

Ubuntu PC — ROS 2 Jazzy
├── teleop_twist_keyboard
│   └── publishes: /cmd_vel
│
└── RViz2
    ├── Fixed Frame: odom
    ├── Grid
    ├── RobotModel
    ├── TF
    └── Odometry
```

No LiDAR is required.

No SLAM is required.

No map frame is required.

---

## 2. Requirements

### Rock 5C

- Debian 12
- ROS 2 Humble
- built robot workspace
- STM32 base controller connected
- working motor driver
- working odometry publisher

### Ubuntu PC

- Ubuntu 24.04
- ROS 2 Jazzy
- RViz2
- `teleop_twist_keyboard`

Install PC packages:

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard
```

---

## 3. Clone and Build the Repository

Run this on both the Rock 5C and the PC.

```bash
mkdir -p ~/workspace
cd ~/workspace

git clone <YOUR_GITHUB_REPOSITORY_URL> ros2_ws
cd ~/workspace/ros2_ws
```

### Rock 5C build

```bash
source ~/ros2_humble/install/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r \
  -y

colcon build --symlink-install

source ~/workspace/ros2_ws/install/setup.bash
```

### PC build

If the PC only needs the robot description package:

```bash
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r \
  -y

colcon build \
  --symlink-install \
  --packages-select ros_car_description

source ~/workspace/ros2_ws/install/setup.bash
```

---

## 4. Configure ROS 2 Networking

Use the same settings on both machines.

```bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Optional: add them to `~/.bashrc`.

```bash
cat >> ~/.bashrc <<'EOF'

# ROS 2 distributed robot network
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
EOF
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

Confirm the PC can reach the Rock 5C:

```bash
ping <ROCK_5C_IP>
```

---

# 5. Start the Robot on the Rock 5C

## Preferred command

The preferred bringup launch should start:

```text
base_driver
robot_state_publisher
```

Run:

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_bringup base.launch.py
```

The launch file should not start:

```text
LD06
SLAM Toolbox
Nav2
RViz2
```

---

## Manual fallback

Use separate terminals if the bringup launch file does not exist yet.

### Rock terminal R1 — base driver

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run ros_car_base base_driver
```

Check the available executable name if required:

```bash
ros2 pkg executables ros_car_base
```

### Rock terminal R2 — robot description

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_description display.launch.py
```

If the repository uses a different launch filename:

```bash
find ~/workspace/ros2_ws/src/ros_car_description \
  -maxdepth 3 \
  -type f
```

---

## Verify the Rock 5C

```bash
ros2 node list
```

Expected nodes:

```text
/base_driver
/robot_state_publisher
```

Check odometry:

```bash
ros2 topic hz /odom
```

Expected:

```text
approximately 44 Hz
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Check command subscription:

```bash
ros2 topic info /cmd_vel -v
```

Expected:

```text
Subscriber:
  /base_driver
```

---

# 6. Verify Robot Data on the PC

Open a terminal on the PC:

```bash
source /opt/ros/jazzy/setup.bash

if [ -f ~/workspace/ros2_ws/install/setup.bash ]; then
  source ~/workspace/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Restart the ROS daemon:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2
```

Check nodes:

```bash
ros2 node list
```

Check odometry:

```bash
ros2 topic hz /odom
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Do not continue until the PC receives `/odom` and `odom -> base_link`.

---

# 7. Start RViz2 on the PC

```bash
source /opt/ros/jazzy/setup.bash

if [ -f ~/workspace/ros2_ws/install/setup.bash ]; then
  source ~/workspace/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

rviz2
```

Set:

```text
Fixed Frame: odom
```

Add these displays:

```text
Grid
RobotModel
TF
Odometry
```

Configure:

```text
Odometry topic: /odom
```

Recommended grid settings:

```text
Plane: XY
Cell Size: 1.0
Plane Cell Count: 20
```

The robot should appear on the grid and move according to odometry.

---

## RobotModel troubleshooting

If RobotModel reports that the robot description is missing:

```bash
ros2 topic echo /robot_description --once
```

If the topic is unavailable, run `robot_state_publisher` locally on the PC using the same URDF package, or verify that it is publishing from the Rock 5C.

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

The RobotModel cannot be displayed correctly without the required link transforms.

---

# 8. Control the Robot from the PC Keyboard

Open a new PC terminal:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The standard controls are:

```text
u    i    o
j    k    l
m    ,    .
```

Common commands:

```text
i = forward
, = backward
j = rotate left
l = rotate right
k = stop
```

Speed controls:

```text
q/z = increase/decrease all speeds
w/x = increase/decrease linear speed
e/c = increase/decrease angular speed
```

Start with low speeds.

Recommended initial values:

```text
Linear speed:  0.10 m/s
Angular speed: 0.25 rad/s
```

The terminal running `teleop_twist_keyboard` must remain focused for keyboard input.

---

# 9. Safety Check Before Driving

Before placing the robot on the floor:

1. Lift the drive wheels off the ground.
2. Press `i` and confirm both wheels drive forward.
3. Press `,` and confirm both wheels reverse.
4. Press `j` and confirm the robot would rotate left.
5. Press `l` and confirm the robot would rotate right.
6. Press `k` and confirm the motors stop.
7. Confirm the STM32 watchdog stops the robot if `/cmd_vel` disappears.

Stop immediately if the wheel direction is incorrect.

---

# 10. Final Run Commands

## Rock 5C

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch ros_car_bringup base.launch.py
```

## PC terminal P1 — RViz2

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

rviz2
```

RViz settings:

```text
Fixed Frame: odom

Displays:
Grid
RobotModel
TF
Odometry (/odom)
```

## PC terminal P2 — keyboard control

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

# 11. Quick Verification Commands

On either machine:

```bash
ros2 topic hz /odom
```

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

```bash
ros2 topic info /cmd_vel -v
```

```bash
ros2 topic echo /cmd_vel
```

Expected data flow:

```text
PC keyboard
    ↓
teleop_twist_keyboard
    ↓
/cmd_vel
    ↓ DDS
Rock 5C base_driver
    ↓
STM32
    ↓
motors

wheel feedback
    ↓
STM32
    ↓
Rock 5C base_driver
    ↓
/odom and odom -> base_link
    ↓ DDS
PC RViz2
```

---

# 12. Common Problems

## PC cannot see Rock nodes

Check both machines:

```bash
echo "$ROS_DOMAIN_ID"
echo "$ROS_AUTOMATIC_DISCOVERY_RANGE"
```

Expected:

```text
42
SUBNET
```

Restart the PC daemon:

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
```

Check network access:

```bash
ping <ROCK_5C_IP>
```

---

## Keyboard command publishes but robot does not move

Check:

```bash
ros2 topic info /cmd_vel -v
```

The topic must have:

```text
Publisher:
  teleop_twist_keyboard

Subscriber:
  base_driver
```

Observe commands:

```bash
ros2 topic echo /cmd_vel
```

Then check:

```text
serial port
STM32 connection
watchdog state
motor enable state
CAN connection
motor IDs
```

---

## Robot moves but RViz does not move

Check odometry:

```bash
ros2 topic hz /odom
ros2 topic echo /odom
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Publishing `/odom` alone is not enough. The base driver must also publish:

```text
odom -> base_link
```

---

## RobotModel is missing

Check:

```bash
ros2 topic echo /robot_description --once
```

Check required transforms:

```bash
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

Verify that `robot_state_publisher` is running.

---

## RViz says `map` does not exist

This demo does not use a map.

Use:

```text
Fixed Frame: odom
```

Do not use:

```text
Fixed Frame: map
```

---

# 13. Demo Checklist

```text
[ ] Rock bringup starts
[ ] base_driver is running
[ ] robot_state_publisher is running
[ ] PC receives /odom
[ ] PC receives odom -> base_link
[ ] RViz Fixed Frame is odom
[ ] Grid is visible
[ ] RobotModel is visible
[ ] Odometry display is visible
[ ] keyboard terminal publishes /cmd_vel
[ ] robot moves forward and backward
[ ] robot rotates left and right
[ ] k stops the robot
[ ] RViz movement matches physical movement
```

Suggested video sequence:

1. Show the physical robot.
2. Start the Rock 5C bringup.
3. Start RViz2 on the PC.
4. Show the robot model on the grid.
5. Start keyboard teleoperation.
6. Drive forward and backward.
7. Rotate left and right.
8. Show the physical car and RViz moving together.
9. Stop the robot with `k`.
