# Line Tracking & Obstacle Avoidance Robot — ROS 2

An autonomous robot that follows a white line using camera vision and navigates around obstacles using LiDAR, built on ROS 2 Jazzy with Gazebo Harmonic simulation.

## Demo

The TurtleBot3 Waffle follows a white line on a grey floor. When it encounters an obstacle (red/orange box), it autonomously detours around it and rejoins the line on the other side.

## Architecture

```
Camera ──→ line_detector ──→ /line_error ──────→ controller ──→ /cmd_vel ──→ Robot
                          ──→ /line_detected ──→
                          ──→ /line_side ──────→
LiDAR  ──→ obstacle_det  ──→ /obstacle_detected →
                          ──→ /front_distance ──→
                          ──→ /left_distance ───→
                          ──→ /right_distance ──→
```

| Node | Sensor | Purpose |
|------|--------|---------|
| `line_detector` | Camera | Detects white line via HSV thresholding, publishes normalised error [-1, 1] |
| `obstacle_detector` | LiDAR | Splits 360° scan into front/left/right sectors, detects obstacles within 0.5m |
| `controller` | — | PD line following + 7-state obstacle avoidance state machine |

## How It Works

### Line Following
- Camera image → crop bottom 45% → BGR→HSV → threshold white → morphology → contour detection → centroid
- **PD Controller**: `angular = -(Kp × error + Kd × d_error)` with normalised error [-1, 1]
- Kp=1.0, Kd=0.3, angular clamped to ±0.5 rad/s

### Obstacle Avoidance (7-State Machine)
```
FOLLOW_LINE → BACKUP → TURN1 (90°) → DRIVE1 (sideways) → TURN2 (90° back) → DRIVE2 (past) → RETURN (arc to line) → FOLLOW_LINE
```
- Symmetric timed turns: same speed & duration, opposite direction → heading errors cancel (no odometry needed)
- Camera-guided return: arcs toward line, PD steers to centre when line detected

## Requirements

### System
- **Ubuntu 24.04** (Noble Numbat)
- **ROS 2 Jazzy** ([install guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Gazebo Harmonic** ([install guide](https://gazebosim.org/docs/harmonic/install))

### ROS 2 Packages
```bash
sudo apt install \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-cv-bridge \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-opencv \
  python3-numpy
```

### Environment
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select line_tracking_avoidance --symlink-install
source install/setup.bash
```

## Run

```bash
# Kill any stale Gazebo processes
pkill -f gz

# Launch everything (Gazebo + all nodes)
ros2 launch line_tracking_avoidance line_tracking.launch.py
```

Wait ~10 seconds for all components to start. The robot will begin following the line automatically.

**Stop**: Press `Ctrl+C`

## File Structure

```
line_tracking_avoidance/
├── launch/
│   └── line_tracking.launch.py     # Starts Gazebo + bridge + robot + all nodes
├── line_tracking_avoidance/
│   ├── controller.py               # PD line following + obstacle avoidance
│   ├── line_detector.py            # Camera-based line detection (OpenCV)
│   └── obstacle_detector.py        # LiDAR-based obstacle detection
├── worlds/
│   └── line_track.sdf              # Gazebo world (ground, line, obstacles)
├── package.xml                     # ROS 2 package manifest
├── setup.py                        # Python build configuration
└── setup.cfg                       # Executable install paths
```

## Parameters

All parameters are configurable in the launch file:

### Line Following
| Parameter | Default | Description |
|-----------|---------|-------------|
| `Kp` | 1.0 | Proportional gain |
| `Kd` | 0.3 | Derivative gain |
| `base_speed` | 0.18 m/s | Forward speed |
| `max_angular` | 0.5 rad/s | Max turning speed |

### Obstacle Avoidance
| Parameter | Default | Description |
|-----------|---------|-------------|
| `turn_speed` | 0.5 rad/s | Turn-in-place speed |
| `turn_duration` | 3.0 s | Duration of each 90° turn |
| `strafe_time` | 2.5 s | Lateral drive duration |
| `pass_time` | 5.5 s | Forward drive past obstacle |
| `safe_distance` | 0.5 m | Obstacle detection threshold |

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Gazebo won't start | `pkill -9 -f gz && pkill -9 -f ruby` then relaunch |
| Robot doesn't move | Check `ros2 topic echo /line_detected` — should show `True` |
| Robot wobbles | Reduce `Kp` or `max_angular` in launch file |
| Robot doesn't avoid obstacle | Check `ros2 topic echo /obstacle_detected` |

## Tech Stack

- **ROS 2 Jazzy** — robotics middleware (pub/sub communication)
- **Gazebo Harmonic** — physics simulation (camera, LiDAR, differential drive)
- **OpenCV** — computer vision (HSV thresholding, contour detection)
- **Python 3.12** — all node logic
- **TurtleBot3 Waffle** — differential drive robot with camera + 360° LiDAR

## License

MIT
