# ROS2 driver for Bosch BNO055 IMU via I2C
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Galactic](https://img.shields.io/badge/ROS2-Galactic-blue)
[![CI](https://github.com/jarkenau/ros2-imu-bno055/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/jarkenau/ros2-imu-bno055/actions/workflows/ros2-ci.yml)

**Currently WIP!!!**

## Table of Contents
  - [General](#general)
  - [Topics](#topics)
  - [Parametes](#parametes)
  - [Devices](#devices)
    - [Raspberry Pi](#raspberry-pi)

## General 

A ROS2 node for the [Bosch BNO055 IMU](https://www.bosch-sensortec.com/products/smart-sensors/bno055/) via I2C. 

This repository is a port of [Dheera Venkatraman's  ros-imu-bno055](https://github.com/dheera/ros-imu-bno055) to ROS2.

## Topics

- TODO implement all topics

## Parametes

The following parameters can be set in the [launch file](https://github.com/jarkenau/ros2-imu-bno055/blob/main/launch/ros2_bno055_imu.py) of the node. 

| Paramter | Description | Default |
| --- | --- | --- |
| device |  Path to the bus file |  `/dev/i2c-1` |
| adress  |  I2C address of the BNO055 IMU | `0x28` |
| frame  | Coordinate frame of the IMU | `imu`|

## Devices

If your device is not listed, consider adding it with a PR.

For the next steps we assume that you have already set up ROS2 [galactic](https://docs.ros.org/en/galactic/index.html) or [humble](https://docs.ros.org/en/humble/Installation.html).

### Raspberry Pi 

1. Clone the repository into your colcon workspace.

    ```bash
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    git clone https://github.com/jarkenau/ros2-imu-bno055.git
    ```
2. Install dependencies with [rosdep](http://wiki.ros.org/rosdep)

    ```bash 
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the package with [colcon](https://colcon.readthedocs.io/en/released/index.html)
    ```bash
    colcon build
    source install/setup.bash
    ```
4. Launch the node with the settings from the [parameters](#parametes) section
    ```bash
    ros2 launch ros2_bno055_imu ros2_bno055_imu.py
    ```