# G431 Four-Wheel Rover Base — micro-ROS Runbook

Status: **frozen baseline (`g431-base-v1.0`)**  
Target: STM32G431 + Jetson, ROS 2 Humble, micro-ROS serial transport

## 1. Frozen ROS 2 interface

The MCU intentionally exposes only the minimum base interface:

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Jetson → G431 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity command |
| G431 → Jetson | `/wheel_states` | `sensor_msgs/msg/JointState` | Four-wheel position and velocity feedback |

Expected node name:

```text
/g431_rover_base
```

`/wheel_states` joint order:

```text
front_left_wheel_joint
rear_left_wheel_joint
front_right_wheel_joint
rear_right_wheel_joint
```

## 2. Safety and timing contract

- The MCU stops the motors if no `/cmd_vel` arrives for **200 ms**.
- The command source must continuously publish at **10–20 Hz**; **20 Hz is recommended**.
- Do not use 2 Hz: one message arrives every 500 ms, so the 200 ms watchdog repeatedly stops the motors for about 300 ms. This causes the observed start-stop motion and is expected behavior.
- The firmware retains the **300 ms direction-change protection**, 30 RPM limit, and calibrated PWM–RPM mapping.
- Stopping the command publisher must stop the rover within approximately 200 ms.
- This software watchdog is not a substitute for a physical emergency stop.

## 3. One-time Jetson prerequisites

Confirm the serial device:

```bash
ls -l /dev/ttyACM0
```

Confirm the user can access it:

```bash
groups
```

If the device changes after reconnecting, identify it with:

```bash
ls -l /dev/serial/by-id/
```

Prefer the stable `/dev/serial/by-id/...` path in scripts when available.

## 4. Start the native micro-ROS Agent

Terminal 1:

```bash
source /opt/ros/humble/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v4
```

Keep this terminal running while the G431 communicates with ROS 2. Only one Agent may open the serial port at a time.

For detailed diagnosis, temporarily replace `-v4` with `-v6`.

## 5. Optional Docker Agent

Use this only if the native Agent is unavailable. Do not run it at the same time as the native Agent.

```bash
docker run -it --rm \
  --net=host \
  --privileged \
  -v /dev:/dev \
  microros/micro-ros-agent:humble \
  serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v4
```

## 6. Verify the frozen interface

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_ws/install/setup.bash

ros2 daemon stop
ros2 daemon start
ros2 node info /g431_rover_base
ros2 topic list -t
```

Expected node interface:

```text
Subscribers:
  /cmd_vel: geometry_msgs/msg/Twist
Publishers:
  /wheel_states: sensor_msgs/msg/JointState
```

Verify feedback:

```bash
ros2 topic hz /wheel_states
```

Expected rate: approximately **10 Hz** (measured baseline: **9.92 Hz**).

```bash
ros2 topic echo /wheel_states --once
```

Confirm:

- `header.stamp` is non-zero;
- `frame_id` is `base_link`;
- all four joint names are present;
- positions accumulate when wheels rotate;
- velocities update with the correct sign.

## 7. Safe bench motion test

Raise all four wheels off the ground before the first test.

Forward at 20 Hz:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Stop the publisher with `Ctrl+C`. The wheels must stop within approximately 200 ms.

Pure rotation test:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Publish an explicit zero command if required:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 8. Check the command source

While teleoperation or autonomy is active:

```bash
ros2 topic hz /cmd_vel
```

Healthy baseline:

```text
average rate: approximately 20 Hz
min/max interval: approximately 0.050 s
```

Long gaps greater than 0.200 s trigger the watchdog. A low average caused by stopping and restarting the publisher is not a transport fault; inspect the `max` interval and observe the rate while commands are actively being sent.

## 9. Common faults

### `/wheel_states` is listed but no samples arrive

- Confirm the Agent is still running.
- Confirm the G431 is powered and connected.
- Ensure only one Agent owns `/dev/ttyACM0`.
- Restart the Agent, power-cycle the G431, and repeat the interface check.
- Use Agent verbosity `-v6` for entity-creation and transport logs.

### Two nodes have the same `/g431_rover_base` name

Stop duplicate Agents, then refresh discovery:

```bash
ros2 daemon stop
ros2 daemon start
```

Power-cycle the G431 and start exactly one Agent.

### Motors pulse or repeatedly start and stop

Measure `/cmd_vel`. If it is below 5 Hz or has gaps over 200 ms, the watchdog is operating correctly. Run the command source continuously at 10–20 Hz; 20 Hz is recommended.

### Agent reports that the serial port is busy

Find the process holding it:

```bash
lsof /dev/ttyACM0
```

Stop the older Agent process before starting another one.

## 10. Freeze checklist

Before tagging the repository:

- Release firmware builds and flashes successfully.
- `/g431_rover_base` has exactly one subscriber and one publisher.
- `/wheel_states` is stable near 10 Hz.
- Joint names, timestamp, frame, positions, and velocities are valid.
- A fixed command at 20 Hz produces smooth motion.
- Ending `/cmd_vel` publication stops the base within about 200 ms.
- Forward, reverse, and pure rotation directions are correct with the rover lifted.
- Firmware source, CubeMX configuration, linker/micro-ROS files, and this runbook are committed together.

Suggested Git tag:

```bash
git tag -a g431-base-v1.0 -m "Freeze G431 micro-ROS rover base interface"
git push origin g431-base-v1.0
```

Do not change the frozen MCU interface during the next stage. Implement odometry and TF on the Jetson using `/wheel_states` as the input.
