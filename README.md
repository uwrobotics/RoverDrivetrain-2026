# RoverDrivetrain-2026

## Introduction

6-wheel differential drive controller using ROS2 Control and ODrive motor controllers.

## 3D Model Pre-processing

> Lessons Learned:
> - URDF Exporter from Solidworks are not reliable. (Assembly link feature does not the actual stl file origin)
> - STL File too Dense for quick Rendering

**Solutions**

- MeshLab for stl file down resolution

## ODrive Integration

This project uses ODrive motor controllers for wheel control. The `odrive_base`, `odrive_node`, and `odrive_ros2_control` packages are based on the official ODrive ROS2 repository:

**https://github.com/odriverobotics/ros_odrive**

See `odrive_example/` for the original BotWheel Explorer example implementation.

## Project Structure

```
drivetrain-2026/
├── diff_drive/              # Main 6-wheel differential drive package
├── odrive_base/             # ODrive CAN communication library
├── odrive_node/             # Standalone ODrive ROS2 node
├── odrive_ros2_control/     # ODrive ros2_control hardware interface
├── odrive_example/          # ODrive BotWheel Explorer example
├── config/                  # FastDDS network configuration
└── scripts/                 # CAN setup utilities
```

## Architecture

[Software Drivetrain Architecture Planning](https://lucid.app/lucidspark/89862bef-7110-44f3-9bf3-ac842ae2c49e/edit?invitationId=inv_481e83f3-17d8-42c1-9446-1c6eecdb2b79)

## Setup Instructions

### Build and Run

```bash
# Set ROS domain for multi-robot communication
export ROS_DOMAIN_ID=42

# Build the Docker image
docker-compose build

# Run with real hardware
docker-compose up

# Run with mock hardware (for testing without ODrive)
docker-compose run diff_drive ros2 launch diff_drive diff_drive.launch.py use_mock_hardware:=true

# Enter container shell
docker-compose run diff_drive bash
```

### Verify System

```bash
# List ROS2 topics
ros2 topic list

# Check controller status
ros2 control list_controllers

# Send velocity command (unstamped since use_stamped_vel: false)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

### Velocity Convention (ROS right-hand rule)

| Command | Direction |
|---------|-----------|
| `linear.x > 0` | Forward |
| `linear.x < 0` | Backward |
| `angular.z > 0` | Turn LEFT (counter-clockwise) |
| `angular.z < 0` | Turn RIGHT (clockwise) |
