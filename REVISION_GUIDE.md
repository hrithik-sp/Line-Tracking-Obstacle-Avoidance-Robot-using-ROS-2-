# Quick Revision Guide — Line Tracking & Obstacle Avoidance Robot

> Read time: ~15 mins. Covers every file, every concept, every "why".

---

## THE BIG PICTURE (30-second summary)

**Robot**: TurtleBot3 Waffle (differential drive, has camera + LiDAR)  
**Simulator**: Gazebo Harmonic  
**Framework**: ROS 2 Jazzy (Python)  
**Task**: Follow white line → detect obstacle → go around it → rejoin line

**3 nodes** do everything:

| Node | Sensor | Job |
|------|--------|-----|
| `line_detector` | Camera | Find white line, publish error |
| `obstacle_detector` | LiDAR | Detect obstacles, publish distances |
| `controller` | None (subscribes to both) | PD steering + 7-state avoidance |

**Why separate nodes?** ROS 2 pub/sub — each node is independent, communicates via topics. If one crashes, others keep running. Can be tested individually.

---

## FILE 1: `worlds/line_track.sdf`

**What**: XML file defining the Gazebo simulation world  
**Format**: SDF (Simulation Description Format) — Gazebo's native format

**Contents**:
- **Ground plane**: 40x40m, dark grey RGB(0.3, 0.3, 0.3) — gives contrast for white line detection
- **White line**: 16m x 0.15m x 0.001m box, pure white, raised 0.001m above ground (avoids z-fighting)
- **Obstacle 1**: Red box at x=2.5, size 0.2x0.18x0.3m
- **Obstacle 2**: Orange box at x=6.0, same size
- **4 plugins**: Physics (gravity/collisions), Sensors (camera/LiDAR rendering), SceneBroadcaster (GUI), UserCommands (spawn robots)

**Key detail**: World name = `"line_track_world"` — must match spawn command in launch file exactly.

---

## FILE 2: `line_detector.py`

**Input**: Camera image (1920x1080 BGR)  
**Output**: 3 topics — `/line_error`, `/line_detected`, `/line_side`

### Pipeline (6 steps):

```
Camera frame -> Crop bottom 45% (ROI) -> BGR to HSV -> Threshold white
-> Morphology (clean noise) -> Find contours -> Centroid -> Normalised error
```

1. **Crop ROI**: Only bottom 45% — ground ahead, ignore sky/walls
2. **BGR to HSV**: HSV separates brightness from colour. White = any hue, low saturation, high value
3. **Threshold**: H:0-180, S:0-40, V:200-255 — only bright unsaturated pixels (white)
4. **Morphology**: OPEN removes noise dots, CLOSE fills gaps. 5x5 kernel
5. **Contours**: findContours -> filter area > 180px -> take largest
6. **Centroid**: cx = M['m10'] / M['m00'] (centre of mass)

### Error normalisation (THE critical formula):

```
error = (cx - w/2) / (w/2)    # Range: [-1.0, +1.0]
```

**Why normalise?** Raw pixel error depends on resolution. 1920px camera gives +-960, but PD gains tuned for 640px would cause violent oscillation. Normalised = resolution-independent.

### Side detection:
- cx < w/3 = `"left"` | cx > 2w/3 = `"right"` | else = `"center"`

### Key tools:
- **cv_bridge**: Converts ROS Image msg to/from OpenCV numpy array
- **OpenCV**: cvtColor, inRange, morphologyEx, findContours, moments
- **Why HSV not RGB?** HSV separates brightness (V) from colour (H). White = high V, low S. In RGB, "white" changes with lighting.

---

## FILE 3: `obstacle_detector.py`

**Input**: LiDAR scan (/scan — 360 range values)  
**Output**: /obstacle_detected, /front_distance, /left_distance, /right_distance

### How it works:

1. **Clean data**: Replace inf/NaN with 10.0m
2. **Compute angles** per reading, normalise with arctan2(sin,cos) to [-pi, +pi]
3. **3 sectors**: Front (+-30deg), Left (30-100deg), Right (-30 to -100deg)
4. **Min per sector**: Closest object in each direction
5. **Blocked**: front < 0.5m = obstacle detected

**Why 0.5m safe distance?** Robot at 0.18 m/s needs ~1s to react. Covers 0.18m. 0.5m gives 0.32m margin.

**Why arctan2(sin,cos)?** Normalises angles regardless of whether LiDAR starts at 0deg or -180deg.

---

## FILE 4: `controller.py` — THE BRAIN

**Loop**: 20 Hz timer. Subscribes to all 7 topics, publishes /cmd_vel.

### LINE FOLLOWING — PD Control

```
ang = -(Kp * error + Kd * d_error)      clamped to +-0.5 rad/s
speed = 0.18 * max(0.5, 1.0 - |err|)    slows when off-centre
```

| Gain | Value | Role |
|------|-------|------|
| Kp | 1.0 | Corrects current error (proportional) |
| Kd | 0.3 | Dampens oscillation (derivative) |

**Negative sign**: error > 0 (line right) needs angular < 0 (turn right). Negate.

**Line lost < 0.8s**: creep forward. **Lost > 0.8s**: spin toward last-known side.

### OBSTACLE AVOIDANCE — 7-State Machine

```
FOLLOW_LINE -> BACKUP -> TURN1 -> DRIVE1 -> TURN2 -> DRIVE2 -> RETURN -> FOLLOW_LINE
```

| # | State | Action | Duration | Speed |
|---|-------|--------|----------|-------|
| 1 | BACKUP | Reverse | front > 0.6m or 2.5s | -0.10 m/s |
| 2 | TURN1 | Turn away | 3.0s (~86deg) | 0.5 rad/s |
| 3 | DRIVE1 | Drive sideways | 2.5s (~0.45m) | 0.18 m/s |
| 4 | TURN2 | Turn back | 3.0s (symmetric!) | -0.5 rad/s |
| 5 | DRIVE2 | Drive past | 5.5s (~1.0m) | 0.18 m/s |
| 6 | RETURN | Arc to line | Camera sees line centred | 0.10 + 0.35 rad/s |

**3 KEY CONCEPTS:**

1. **Debouncing**: 3 consecutive obstacle detections needed before reacting. Prevents false triggers.
2. **Turn direction**: Compare left_dist vs right_dist. Go toward the clearer side.
3. **Symmetric turns**: TURN1 and TURN2 = same speed, same time, OPPOSITE direction. Rotation errors cancel. No odometry needed.

**RETURN sub-phases**: Blind arc -> Camera detects line -> amplified PD steer -> line centred -> FOLLOW_LINE

---

## FILE 5: `launch/line_tracking.launch.py`

Starts EVERYTHING with one command. Staggered timing:

| Time | Starts | Why wait? |
|------|--------|-----------|
| 0s | Gazebo + robot_state_publisher | — |
| 3s | Parameter bridge | Gazebo must be loaded |
| 5s | Robot spawn | Bridge must be connected |
| 8s | 3 nodes (detector, detector, controller) | Robot must exist |

**Bridge syntax**: `[` = Gazebo->ROS (sensors IN), `]` = ROS->Gazebo (commands OUT)

**Robot spawn**: Position x=-1.5 y=0 (on the line), quaternion (0,0,0,1) = facing +x.

---

## FILES 6-8: Package Config

**package.xml**: Lists dependencies (rclpy, std_msgs, sensor_msgs, geometry_msgs, cv_bridge). Build type = ament_python.

**setup.py**: `data_files` copies launch + worlds to install dir. `entry_points` maps executable names to Python main() functions.

**setup.cfg**: Puts executables in lib/line_tracking_avoidance/ where ROS 2 expects them.

---

## TOPIC MAP

```
Camera --> line_detector --> /line_error ---------> controller --> /cmd_vel --> Motors
                         --> /line_detected ------>
                         --> /line_side ---------->
LiDAR  --> obstacle_det  --> /obstacle_detected -->
                         --> /front_distance ----->
                         --> /left_distance ------>
                         --> /right_distance ----->
```

---

## 10 RAPID-FIRE Q&As

**Q1: Why camera for line + LiDAR for obstacles?**  
Line has zero height, LiDAR can't see it. Camera can't measure obstacle distance reliably.

**Q2: What does use_sim_time: True do?**  
Nodes use Gazebo's clock, not wall clock. Timing stays consistent regardless of PC speed.

**Q3: What are morphological operations?**  
OPEN (erode then dilate) = removes noise. CLOSE (dilate then erode) = fills gaps.

**Q4: Why normalise error to [-1,1]?**  
Resolution-independent PD gains. Same Kp works for any camera.

**Q5: What does the D term do?**  
Dampens oscillation. Slows correction when error is already improving.

**Q6: Why symmetric turns?**  
Same speed x same time x opposite direction = errors cancel. No odometry needed.

**Q7: What is obstacle debouncing?**  
3 consecutive detections required. Filters LiDAR noise.

**Q8: What does the parameter bridge do?**  
Converts Gazebo messages to ROS 2 messages and vice versa.

**Q9: Why stagger launch timing?**  
Each component depends on the previous: Gazebo -> bridge -> spawn -> nodes.

**Q10: SDF vs URDF?**  
SDF = Gazebo worlds + models + sensors. URDF = ROS robot kinematics only.

---

## BUILD and RUN

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select line_tracking_avoidance --symlink-install
source install/setup.bash
ros2 launch line_tracking_avoidance line_tracking.launch.py
# Ctrl+C to stop
```
