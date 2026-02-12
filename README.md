# UWRT Drivetrain 2026

<Add a Full System Design Document and how are we going to approach it. It need to include research and all the actionable item - Fully understand the scope and the requirement>



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

## Acknowledgement

This project uses ODrive motor controllers for wheel control. The `odrive_base`, `odrive_node`, and `odrive_ros2_control` packages are based on the official ODrive ROS2 repository:

**https://github.com/odriverobotics/ros_odrive**

See `odrive_example/` for the original BotWheel Explorer example implementation.