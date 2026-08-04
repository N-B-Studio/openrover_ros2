下面是一套从零开始、按终端编号执行的完整流程。照这个顺序启动，就不会再丢步骤。

# A. Rock 5C 端

## Terminal R1 — 启动小车基础节点

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

启动你现有的 Stage 1 bringup：

```bash
ros2 launch ros_car_bringup car.launch.py
```

如果你的 launch 文件名不是这个，使用你之前已经能启动：

```text
/base_driver
/robot_state_publisher
```

的那个命令。

验证：

```bash
ros2 node list
```

至少应看到：

```text
/base_driver
/robot_state_publisher
```

检查里程计：

```bash
ros2 topic hz /odom
```

目标：

```text
约 44 Hz
```

---

## Terminal R2 — 启动 LD06

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

启动：

```bash
ros2 launch ldlidar ldlidar.launch.py \
  serial_port:=/dev/ttyS6 \
  lidar_frame:=laser_frame \
  topic_name:=scan
```

正常日志应包含：

```text
Using port /dev/ttyS6
LiDAR_LD06 started successfully
```

保持该终端运行。

---

## Terminal R3 — 发布 LiDAR 静态 TF

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

运行：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x -0.007 \
  --y 0.0 \
  --z 0.125 \
  --roll 0 \
  --pitch 0 \
  --yaw 1.57079632679 \
  --frame-id base_link \
  --child-frame-id laser_frame
```

保持该终端运行。

---

## Terminal R4 — Rock 端总检查

```bash
source ~/ros2_humble/install/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

检查 `/scan`：

```bash
ros2 topic hz /scan
```

应约为：

```text
10 Hz
```

检查 `/odom`：

```bash
ros2 topic hz /odom
```

应约为：

```text
44 Hz
```

检查完整 TF：

```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```

应持续输出类似：

```text
Translation: [-0.007, 0.000, 0.125]
```

如果这三项通过，Rock 端完成。

不要在 Rock 上启动：

```bash
ros2 launch ros_car_navigation slam.launch.py
```

因为 SLAM 由 laptop 运行。

---

# B. Laptop 端

## Terminal P1 — 检查 Rock 数据

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

建议重启 daemon：

```bash
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2
```

检查雷达：

```bash
ros2 topic hz /scan
```

应约为：

```text
10 Hz
```

检查里程计：

```bash
ros2 topic hz /odom
```

应约为：

```text
44 Hz
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```

必须持续输出。

只有这三项都正常，才继续 SLAM。

---

## Terminal P2 — 启动 SLAM Toolbox

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

启动：

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/workspace/ros_car_demo/config/slam_params.yaml
```

保持运行。

正常日志应包含：

```text
Using solver plugin solver_plugins::CeresSolver
```

---

## Terminal P3 — 检查 SLAM

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

检查地图 topic：

```bash
ros2 topic list | grep -E "map|scan"
```

应看到：

```text
/map
/map_metadata
/scan
```

检查 SLAM 是否订阅 `/scan`：

```bash
ros2 topic info /scan -v
```

应至少有：

```text
Publisher count: 1
Subscription count: 1
```

检查 `map -> odom`：

```bash
ros2 run tf2_ros tf2_echo map odom
```

一旦这个开始持续输出，说明 SLAM 正常工作。

---

## Terminal P4 — 启动 RViz2

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

rviz2
```

RViz 设置：

```text
Fixed Frame = map
```

添加：

```text
Map
LaserScan
RobotModel
TF
Odometry
Grid
```

对应 Topic：

```text
Map        = /map
LaserScan  = /scan
Odometry   = /odom
```

如果刚启动时提示：

```text
Frame [map] does not exist
```

先等待几秒。

如果仍然不存在，回 Terminal P3 检查：

```bash
ros2 run tf2_ros tf2_echo map odom
```

---

## Terminal P5 — 键盘控制建图

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

启动：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

按键：

```text
i = 前进
, = 后退
j = 左转
l = 右转
k = 停止
```

建议速度：

```text
Linear  = 0.10–0.15 m/s
Angular = 0.20–0.30 rad/s
```

推荐建图路线：

```text
1. 静止 5–10 秒
2. 缓慢直行
3. 缓慢转弯
4. 沿房间边缘走
5. 尽量回到起点
6. 等待 loop closure
```

避免：

```text
快速旋转
急加速
轮胎打滑
抬起机器人
手推机器人
```

---

# C. 地图不好时重新开始

只停止 laptop 上的 SLAM Toolbox。

在 Terminal P2 按：

```text
Ctrl+C
```

然后重新运行：

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/workspace/ros_car_demo/config/slam_params.yaml
```

这会创建一张新的空地图。

以下节点不需要重启：

```text
Rock base_driver
Rock robot_state_publisher
Rock LD06
Rock laser TF
Laptop RViz
Laptop keyboard teleop
```

RViz 可以保持打开。

---

# D. 保存地图

## Terminal P6 — 保存最终地图

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

创建目录：

```bash
mkdir -p ~/workspace/ros_car_demo/maps
```

保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/workspace/ros_car_demo/maps/room_map
```

生成：

```text
~/workspace/ros_car_demo/maps/room_map.yaml
~/workspace/ros_car_demo/maps/room_map.pgm
```

检查：

```bash
ls -lh ~/workspace/ros_car_demo/maps
```

---

# 最终启动顺序速查

## Rock 5C

```text
R1: base bringup
R2: LD06
R3: base_link -> laser_frame
R4: diagnostics
```

## Laptop

```text
P1: verify /scan, /odom, TF
P2: slam_toolbox
P3: verify /map and map -> odom
P4: RViz2
P5: keyboard teleop
P6: save map
```

最关键的启动顺序是：

```text
1. Rock base
2. Rock LD06
3. Rock laser TF
4. Laptop verify data
5. Laptop SLAM
6. Laptop verify map -> odom
7. Laptop RViz
8. Laptop keyboard control
9. Save map
```
