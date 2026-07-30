Robot: Rock 5C
────────────────────────────
STM32 base controller
LD06 driver
base_driver
robot_state_publisher
static/dynamic TF publishers
hardware watchdog
sensor diagnostics

Publishes:
  /scan
  /odom
  /tf
  /tf_static
  /joint_states
  /diagnostics

Subscribes:
  /cmd_vel

             DDS network

Workstation: Ubuntu PC
────────────────────────────
RViz
slam_toolbox
map saver
Nav2
rosbag2
diagnostic tools
later perception workloads