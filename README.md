# OpenRover ROS 2

A modular ROS 2 mobile robotics platform for hardware integration, odometry, LiDAR SLAM, sensor fusion, and autonomous navigation.

## Overview

This repository contains firmware and software components for an open-source rover platform built around STM32 microcontrollers and ROS 2. It includes:

- STM32CubeIDE firmware projects for rover motor control and embedded interfaces
- ROS 2 workspace sources for perception, navigation, and control
- Support for simulation, debugging, and hardware bring-up workflows

## Project Structure

- G431CBU6-Rover/: rover-related firmware and project assets
- STM32F103C8T6_ros2_car/: STM32F103-based ROS 2 car firmware project
- Ubuntu-22/workspace/ros2_ws/: ROS 2 workspace for higher-level robotics software

## Getting Started

1. Clone this repository.
2. Open the STM32CubeIDE project you want to build.
3. Build and flash the firmware to the target MCU.
4. Source your ROS 2 environment and build the workspace.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
