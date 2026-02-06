# Differential Drive Controller

A ROS2 Control-based differential drive controller package for robots using ODrive motor controllers.

## Package Structure

```
diff_drive/
├── config/
│   └── diff_drive_controllers.yaml   # Controller configuration
├── description/
│   └── urdf/
│       ├── diff_drive.urdf.xacro           # Main robot description
│       ├── diff_drive_description.urdf.xacro  # Physical robot model
│       └── diff_drive.ros2_control.xacro   # ros2_control hardware interface
├── launch/
│   └── diff_drive.launch.py          # Launch file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

- ROS2 (Humble/Iron/Jazzy/Kilted)
- ros2_control
- ros2_controllers
- diff_drive_controller
- xacro
- robot_state_publisher
- odrive_ros2_control (for real hardware)

## Usage

### With Real Hardware (ODrive)

1. Setup CAN interface:
```bash
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

2. Launch the controller:
```bash
ros2 launch diff_drive diff_drive.launch.py
```

### With Mock Hardware (Testing)

```bash
ros2 launch diff_drive diff_drive.launch.py use_mock_hardware:=true
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_mock_hardware` | `false` | Use mock hardware for testing |
| `mock_sensor_commands` | `false` | Enable mock sensor commands |
| `can_interface` | `can0` | CAN interface name |
| `left_wheel_node_id` | `0` | ODrive node ID for left wheel |
| `right_wheel_node_id` | `1` | ODrive node ID for right wheel |

## Controlling the Robot

Send velocity commands to the `/diff_drive_controller/cmd_vel` topic:

```bash
# Move forward
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Rotate
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

## Topics

### Subscribed
- `/diff_drive_controller/cmd_vel` (geometry_msgs/Twist) - Velocity commands

### Published
- `/diff_drive_controller/odom` (nav_msgs/Odometry) - Odometry
- `/joint_states` (sensor_msgs/JointState) - Joint states
- `/tf` - Transform from odom to base_link

## Configuration

Edit `config/diff_drive_controllers.yaml` to adjust:
- Wheel separation and radius
- Velocity and acceleration limits
- Odometry covariance
- Update rates

## Robot Parameters

Default robot dimensions (configurable in URDF):
- Wheel radius: 0.1 m
- Wheel separation: 0.35 m
- Base dimensions: 0.4 x 0.3 x 0.15 m
