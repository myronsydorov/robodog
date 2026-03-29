# Unitree Go2 ROS2 Starter Pack

> **Documentation status:** This README is currently incomplete and lags behind the actual state of the codebase. Some implemented features, integrations, topics, frames, and utilities are not yet documented here. Treat this as a starter guide rather than full documentation.

ROS2 Foxy

---

## 📦 Package Overview

```text
src/
├── go2_bringup/        # Launch files and sensor configs
├── go2_control/        # C++ velocity control node (SportClient API)
├── go2_interfaces/     # Python utilities and message types
├── go2_examples/       # Keyboard teleop for testing
├── go2_nav/            # SLAM integration available, Nav2 integration in progress
├── unitree_api/        # Unitree ROS2 message definitions
└── unitree_go/         # Unitree Go2 message definitions
```

---

## 🚀 Quick Start

### 1. Build the Workspace

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Robot Control

```bash
ros2 launch go2_bringup go2_control.launch.py
```

### 3. Launch Sensors

```bash
# Launch both camera and LiDAR
ros2 launch go2_bringup sensors.launch.py

# Camera only
ros2 launch go2_bringup sensors.launch.py enable_lidar:=false

# LiDAR only
ros2 launch go2_bringup sensors.launch.py enable_camera:=false

# Launch with RViz
ros2 launch go2_bringup sensors.launch.py enable_rviz:=true
```

### 4. Launch SLAM

```bash
ros2 launch go2_nav slam.launch.py
```

### 5. Test with Keyboard Control

```bash
ros2 run go2_examples keyboard_teleop
```

---

## ⌨️ Keyboard Controls

| Key | Action |
|------|---------|
| `W / S` | Forward / Backward |
| `A / D` | Strafe Left / Right |
| `Q / E` | Rotate Left / Right |
| `1` | Stand Up |
| `2` | Sit Down |
| `3` | Balance Stand |
| `4` | Recovery Stand |
| `5` | Hello / Wave |
| `+ / -` | Increase / Decrease speed |
| `SPACE` | Stop |
| `X` | Emergency Stop |
| `R` | Release Emergency Stop |
| `ESC` | Quit |

---

## 🎮 Robot Control

### Velocity Commands

Publish to `/cmd_vel`:

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 0.5
cmd.linear.y = 0.0
cmd.angular.z = 0.3

publisher.publish(cmd)
```

Recommended limits:

- Linear velocity: `±1.0 m/s`
- Angular velocity: `±1.0 rad/s`
- Recommended for first tests: `0.2 - 0.3 m/s`

---

### Posture Commands

Publish to `/cmd_posture`:

```bash
# Stand up
ros2 topic pub /cmd_posture std_msgs/String "data: 'up'" --once

# Sit down
ros2 topic pub /cmd_posture std_msgs/String "data: 'down'" --once

# Balance stand
ros2 topic pub /cmd_posture std_msgs/String "data: 'balance'" --once

# Recovery stand
ros2 topic pub /cmd_posture std_msgs/String "data: 'recovery'" --once

# Wave / Hello
ros2 topic pub /cmd_posture std_msgs/String "data: 'hello'" --once
```

Available posture commands:

```text
up, down, balance, recovery, sit, hello, stretch, stop
```

---

### Emergency Stop

```bash
# Activate emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: false" --once
```

---

## 🗺️ SLAM

SLAM is currently integrated and working. Nav2 integration is still in progress.

```bash
ros2 launch go2_nav slam.launch.py
```

Typical SLAM output topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Generated 2D map |
| `/map_metadata` | `nav_msgs/MapMetaData` | Map metadata |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan used by SLAM |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms |
| `/odom` | `nav_msgs/Odometry` | Odometry source if available |

SLAM Notes:

- Requires a correct TF tree
- Sensor frames must be connected to `base_link`
- If using `/scan`, point cloud to laser conversion must be running
- `map -> odom` is usually produced by SLAM
- `odom -> base_link` is usually produced by odometry / localization

---

## 📡 ROS2 Topics

> The following list contains the main documented topics, but it may not be exhaustive.

### Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command input |
| `/cmd_posture` | `std_msgs/String` | Posture command input |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop trigger |
| `/api/sport/request` | `unitree_api/Request` | Raw Unitree Sport API command |

---

### Camera Topics (RealSense D435i)

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB image |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth image |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | Colored point cloud |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/camera/imu` | `sensor_msgs/Imu` | Camera IMU if enabled |

---

### LiDAR Topics (Hesai XT16)

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_points` | `sensor_msgs/PointCloud2` | LiDAR point cloud |
| `/lidar_imu` | `sensor_msgs/Imu` | LiDAR internal IMU |

---

### SLAM / Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 2D scan for SLAM / Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | Current map |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms |
| `/odom` | `nav_msgs/Odometry` | Odometry |

---

## 🧭 TF Frames

The exact TF tree may vary depending on your current SLAM / localization setup.

Minimal sensor tree:

```text
base_link
├── camera_link
│   └── camera_color_optical_frame
└── hesai_lidar
```

SLAM / navigation tree:

```text
map
└── odom
    └── base_link
        ├── camera_link
        │   └── camera_color_optical_frame
        └── hesai_lidar
```

Frame Notes:

- `map -> odom` is generally provided by SLAM or localization
- `odom -> base_link` is provided by odometry or state estimation
- Sensor transforms should usually be static
- Frame names must match the configuration used in SLAM and Nav2

---

## 🏗️ Architecture

```text
┌─────────────────────┐     ┌─────────────────────┐
│   Your ROS2 Node    │     │ Keyboard Teleop    │
└─────────┬───────────┘     └─────────┬───────────┘
          │                           │
          │ /cmd_vel                  │ /cmd_vel
          │ /cmd_posture              │
          ▼                           ▼
┌───────────────────────────────────────────────────────┐
│                  go2_control_node                    │
│      Converts ROS2 commands to Unitree Sport API    │
└─────────────────────────┬────────────────────────────┘
                          │
                          │ /api/sport/request
                          ▼
┌───────────────────────────────────────────────────────┐
│                  Unitree Go2 Robot                   │
│             On-board Unitree sport_mode             │
└───────────────────────────────────────────────────────┘
```

---

## ⚠️ Safety

1. Always have someone nearby ready to intervene
2. Start with low speeds (`0.2 - 0.3 m/s`)
3. Test in open areas first
4. Be ready to emergency stop immediately

```bash
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
```

---

## 🔍 Useful Debug Commands

```bash
# List all ROS2 topics
ros2 topic list

# Show active TF tree
ros2 run tf2_tools view_frames

# Inspect current transforms
ros2 run tf2_ros tf2_echo base_link hesai_lidar

# Show current topic rates
ros2 topic hz /lidar_points
ros2 topic hz /camera/color/image_raw

# Inspect SLAM map output
ros2 topic echo /map

# Visualize active nodes
ros2 node list
```

---

## 📚 Resources

- ROS2 Foxy Documentation: https://docs.ros.org/en/foxy/
- Intel RealSense ROS2: https://github.com/IntelRealSense/realsense-ros
- Hesai ROS2 Driver: https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- Nav2: https://navigation.ros.org/