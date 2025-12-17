# follow_nao

A ROS 2 package for object tracking and following behavior on the NAO humanoid robot. This package integrates YOLO-based object detection with robot motion control using **BehaviorTree.CPP**, utilizing NAO's sonar sensors for obstacle avoidance.

## Overview

The `follow_nao` package enables the NAO robot to detect and follow objects (e.g., persons) while autonomously avoiding obstacles using its built-in sonar sensors. The package uses **BehaviorTree.CPP** to implement intelligent search and follow behaviors, providing both 2D (bounding box-based) and 3D (depth-aware) tracking modes.

### Key Features

- **BehaviorTree.CPP Integration**: Modular behavior tree architecture for search and follow behaviors
- **Autonomous Search**: Robot automatically spins to search for targets when none are detected
- **Object Detection Integration**: Works with YOLO object detectors via the `yolo_ros` package
- **2D and 3D Tracking**: Support for both monocular (2D) and depth-based (3D) tracking
- **Obstacle Avoidance**: Leverages NAO's left and right sonar sensors to detect and avoid obstacles
- **TF Broadcasting**: Publishes target positions as TF frames for easy integration with navigation stacks
- **Touch Sensor Control**: Start/stop following behavior by touching NAO's head sensors
- **XML-Configurable**: Behavior trees can be modified via XML without recompiling

### Current Limitations

⚠️ **Depth Information**: At the moment, real depth information has not been fully integrated. The `entity_tracker_fake_3d` node works around this limitation by always placing detected objects at a **fixed distance of 1 meter** from the robot, using camera intrinsics to compute the angular direction. This is a temporary solution until proper depth estimation (e.g., from depth cameras or monocular depth estimation networks) is integrated.

## Architecture

The package now uses **BehaviorTree.CPP** for intelligent behavior management:

### BehaviorTree Architecture

```
Root: ReactiveFallback
├── ReactiveSequence (Follow if detected)
│   ├── IsTargetDetected? (Condition)
│   └── Follow (Action with obstacle avoidance)
└── SpinSearch (Action - returns RUNNING until target found)
```

**Behavior Logic:**
1. **IsTargetDetected**: Checks if target TF frame exists
2. If **TRUE** → **Follow**: Robot follows target with integrated obstacle avoidance
3. If **FALSE** → **SpinSearch**: Robot spins in place searching for target (returns RUNNING continuously)

### System Architecture

The complete system workflow:

```
┌──────────────┐
│  YOLO ROS    │ ──> DetectionArray (custom YOLO format)
└──────────────┘
       │
       v
┌──────────────────┐
│ yolo_to_standard │ ──> Detection2DArray / Detection3DArray
└──────────────────┘
       │
       v
┌───────────────────┐
│ Entity Tracker    │ ──> TF: base_link -> target
└───────────────────┘
       │
       v
┌──────────────────────┐
│  BehaviorTree Node   │
│  ┌────────────────┐  │
│  │ IsTargetDetected│ │ ──> Checks TF
│  └────────────────┘  │
│  ┌────────────────┐  │       ┌──────────────┐
│  │ SpinSearch     │ │ <──── │ Sonar Sensors│
│  └────────────────┘  │       └──────────────┘
│  ┌────────────────┐  │              │
│  │ Follow         │ │ <─────────────┘
│  └────────────────┘  │
└──────────────────────┘
       │
       v
   Twist (cmd_vel)
```

## Nodes

### BehaviorTree Nodes (C++)

#### bt_follow_node

Main behavior tree executor that loads and runs the XML-defined behavior tree.

**Parameters:**
- `bt_xml` (string, **required**) - Path to the behavior tree XML file
- `tick_rate` (double, default: `10.0`) - Behavior tree tick rate in Hz

**Behavior Tree Components:**

##### IsTargetDetected (Condition)
Checks if the target TF frame is available.

**Ports:**
- `target_frame` (string, default: `"target"`) - Target TF frame to check
- `base_frame` (string, default: `"base_link"`) - Base TF frame
- `timeout` (double, default: `0.5`) - Time to wait for transform (seconds)

**Returns:**
- `SUCCESS` if target TF is available
- `FAILURE` if target TF is not available

##### SpinSearch (Action)
Rotates the robot in place while searching for the target. Returns RUNNING until target is detected.

**Ports:**
- `target_frame` (string, default: `"target"`) - Target TF frame to search for
- `base_frame` (string, default: `"base_link"`) - Base TF frame
- `angular_speed` (double, default: `0.5`) - Rotation speed (rad/s)
- `cmd_vel_topic` (string, default: `"/cmd_vel"`) - Velocity command topic

**Returns:**
- `RUNNING` while searching (spinning)
- `SUCCESS` when target is detected

##### Follow (Action)
Follows the target TF frame with integrated obstacle avoidance using sonar sensors.

**Ports:**
- `target_frame` (string, default: `"target"`) - Target TF frame to follow
- `base_frame` (string, default: `"base_link"`) - Base TF frame
- `min_distance` (double, default: `1.0`) - Desired distance to target (m)
- `avoidance_distance` (double, default: `0.5`) - Minimum distance to obstacles (m)
- `max_linear_speed` (double, default: `0.5`) - Maximum forward speed (m/s)
- `max_angular_speed` (double, default: `1.0`) - Maximum rotation speed (rad/s)
- `cmd_vel_topic` (string, default: `"/cmd_vel"`) - Velocity command topic
- `sonar_topic` (string, default: `"/sensors/sonar"`) - Sonar sensor topic
- `touch_topic` (string, default: `"/sensors/touch"`) - Touch sensor topic

**Returns:**
- `RUNNING` while following
- `FAILURE` if target is lost

### Python Nodes (Legacy)

#### 1. yolo_to_standard

Converts YOLO's custom `DetectionArray` messages to standard ROS `Detection2DArray` and `Detection3DArray` messages.

**Subscribed Topics:**
- `input_detection_2d` (`yolo_msgs/DetectionArray`) - YOLO 2D detections
- `input_detection_3d` (`yolo_msgs/DetectionArray`) - YOLO 3D detections

**Published Topics:**
- `output_detection_2d` (`vision_msgs/Detection2DArray`) - Standard 2D detections
- `output_detection_3d` (`vision_msgs/Detection3DArray`) - Standard 3D detections

#### 2. entity_tracker_fake_3d

Tracks a target object from 2D detections and publishes its position as a TF frame. Uses camera intrinsics to compute the angular direction and places the target at a fixed 1m distance.

**Subscribed Topics:**
- `input_detection_2d` (`vision_msgs/Detection2DArray`) - 2D object detections
- `camera_info` (`sensor_msgs/CameraInfo`) - Camera intrinsics

**Published TF:**
- `source_frame` → `target_frame` (default: `base_link` → `target`)

**Parameters:**
- `target_class` (string, default: `"person"`) - Class name to track
- `source_frame` (string, default: `"base_link"`) - Parent frame for the TF
- `target_frame` (string, default: `"target"`) - Child frame name for the TF
- `optical_frame` (string, default: `"CameraTop_optical_frame"`) - Camera optical frame

#### 3. entity_tracker_3d

Tracks a target object from 3D detections (with real depth information) and publishes its position as a TF frame.

**Subscribed Topics:**
- `input_detection_3d` (`vision_msgs/Detection3DArray`) - 3D object detections

**Published TF:**
- `source_frame` → `target_frame`

**Parameters:**
- `target_class` (string, default: `"person"`)
- `source_frame` (string, default: `"base_link"`)
- `target_frame` (string, default: `"target"`)

#### 4. motion_control_2d (Legacy)

Generates velocity commands to follow a detected object while avoiding obstacles using sonar sensors. Works directly with 2D detections.

**Subscribed Topics:**
- `input_detection_2d` (`vision_msgs/Detection2DArray`) - 2D object detections
- `camera_info` (`sensor_msgs/CameraInfo`) - Camera intrinsics
- `sonar` (`nao_lola_sensor_msgs/Sonar`) - Sonar distance measurements
- `touch` (`nao_lola_sensor_msgs/Touch`) - Touch sensor states

**Published Topics:**
- `vel` (`geometry_msgs/Twist`) - Velocity commands

**Parameters:**
- `target_class` (string, default: `"person"`) - Class to follow
- `avoidance_distance` (double, default: `0.5`) - Minimum distance to obstacles (meters)
- `max_linear_speed` (double, default: `0.5`) - Maximum forward speed (m/s)
- `max_angular_speed` (double, default: `1.0`) - Maximum rotation speed (rad/s)
- `base_frame` (string, default: `"base_footprint"`) - Robot's base frame
- `optical_frame` (string, default: `"CameraTop_optical_frame"`) - Camera optical frame

#### 5. motion_control_3d (Legacy)

Generates velocity commands to follow a target TF frame while avoiding obstacles using sonar sensors.

**Subscribed Topics:**
- `sonar` (`nao_lola_sensor_msgs/Sonar`) - Sonar distance measurements
- `touch` (`nao_lola_sensor_msgs/Touch`) - Touch sensor states
- **TF Lookup:** `base_frame` → `target_frame`

**Published Topics:**
- `vel` (`geometry_msgs/Twist`) - Velocity commands

**Parameters:**
- `min_distance` (double, default: `1.0`) - Desired distance to target (meters)
- `avoidance_distance` (double, default: `0.5`) - Minimum distance to obstacles (meters)
- `max_linear_speed` (double, default: `0.5`) - Maximum forward speed (m/s)
- `max_angular_speed` (double, default: `1.0`) - Maximum rotation speed (rad/s)
- `base_frame` (string, default: `"base_footprint"`) - Robot's base frame
- `target_frame` (string, default: `"target"`) - Target frame to follow

## Installation

### Prerequisites

This package requires the following dependencies:
- ROS 2 Jazzy (or Humble)
- **BehaviorTree.CPP** v4.x
- `yolo_ros` package ([mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros))
- `nao_lola_sensor_msgs` package (NAO robot interfaces)
- Standard ROS 2 packages: `rclcpp`, `rclpy`, `geometry_msgs`, `vision_msgs`, `tf2_ros`

### Build

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> follow_nao
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
source /opt/ros/jazzy/setup.bash  # or humble
colcon build --packages-select follow_nao
source install/setup.bash
```

## Usage

### Launch Files

#### **Recommended: BehaviorTree Follow (Complete System)**

Launch the complete system with BehaviorTree.CPP for autonomous search and follow:

```bash
ros2 launch follow_nao bt_follow_nao.launch.py
```

This launches:
- YOLO detector (`yolo_bringup`)
- `yolo_to_standard` converter
- `entity_tracker_fake_3d` TF publisher
- `bt_follow_node` BehaviorTree executor

**Behavior:**
- Robot automatically **spins to search** for target when none is detected
- When target is found, robot **follows it** with obstacle avoidance
- Touch head sensors to pause/resume

#### Legacy Launch Files

##### 1. 2D Follow (Direct Detection Following)

Launch the 2D following behavior without depth information:

```bash
ros2 launch follow_nao 2d_follow_nao.launch.py
```

This launches:
- `motion_control_2d` node

Default remappings:
- `/sonar` → `/sensors/sonar`
- `/touch` → `/sensors/touch`
- `/camera_info` → `/camera_rgb_info`
- `/input_detection_2d` → `/detections_2d`
- `/vel` → `/target`

##### 2. 3D Follow (TF-based Following)

Launch the 3D following behavior with TF frame tracking:

```bash
ros2 launch follow_nao 3d_follow_nao.launch.py
```

This launches:
- `motion_control_3d` node

Default remappings:
- `/sonar` → `/sensors/sonar`
- `/vel` → `/target`

##### 3. 2D Track (Detection + TF Publishing)

Launch YOLO detection with fake 3D tracking (1m fixed distance):

```bash
ros2 launch follow_nao 2d_track.launch.py
```

This launches:
- YOLO detector (`yolo_bringup`)
- `yolo_to_standard` converter
- `entity_tracker_fake_3d` TF publisher

##### 4. 3D Track (YOLO + Depth)

Launch YOLO detection with 3D depth information:

```bash
ros2 launch follow_nao 3d_track.launch.py
```

This launches:
- YOLO detector with depth (`yolo_bringup`)
- `yolo_to_standard` converter
- `entity_tracker_3d` TF publisher

### Example: Complete Following System

To run a complete person-following system with fake depth:

```bash
# Terminal 1: Launch YOLO detection and entity tracking
ros2 launch follow_nao 2d_track.launch.py

# Terminal 2: Launch motion control
ros2 launch follow_nao 3d_follow_nao.launch.py
```

Or use a custom launch file combining both.

## Obstacle Avoidance

The **Follow** action in the behavior tree integrates NAO's sonar sensors to detect and avoid obstacles:

- **Left Sonar**: Detects obstacles on the left; robot turns right to avoid
- **Right Sonar**: Detects obstacles on the right; robot turns left to avoid
- **Both Sonars**: When both detect obstacles, the robot turns away from the closer one
- **Avoidance Distance**: Configurable via the `avoidance_distance` parameter (default: 0.5m)

The avoidance behavior is seamlessly integrated into the Follow action, ensuring the robot doesn't collide with obstacles while pursuing its target.

## Touch Sensor Control

Touch any of NAO's head sensors (front, middle, or rear) to toggle between:
- **Active**: Robot follows the target and avoids obstacles
- **Stopped**: Robot stops all motion (publishes zero velocities)

This works within the Follow action node.

## Configuration

### Customizing the Behavior Tree

The behavior tree is defined in XML and can be modified without recompiling:

**File:** `config/follow_behavior.xml`

You can customize:
- Tree structure (add decorators, conditions, etc.)
- Node parameters (speeds, distances, frame names)
- Control flow (sequence, fallback, parallel)

Example - Change search speed:
```xml
<SpinSearch 
  name="SearchForTarget"
  target_frame="target"
  base_frame="base_link"
  angular_speed="0.8"
  cmd_vel_topic="/cmd_vel"/>
```

### Changing Target Class

Modify in the launch file to track different objects:

```python
parameters=[{
    'target_class': 'cup',  # or 'chair', 'bottle', etc.
    # ... other parameters
}]
```

### Using a Custom Behavior Tree

Create your own XML file and specify it:

```bash
ros2 run follow_nao bt_follow_node --ros-args \
    -p bt_xml:=/path/to/your_custom_tree.xml \
    -p tick_rate:=20.0
```

## Visualization

To visualize the target TF frame in RViz:

```bash
ros2 run rviz2 rviz2
```

Add a TF display and you should see the `target` frame being published relative to `base_link` when an object is detected.

## Troubleshooting

### No detections received
- Check that YOLO is running: `ros2 topic echo /yolo/detections`
- Verify topic remappings match your system
- Ensure the target class exists in YOLO's class list

### Robot not moving
- Check touch sensors: Touch NAO's head to toggle start/stop
- Verify velocity commands: `ros2 topic echo /target`
- Check sonar values: `ros2 topic echo /sensors/sonar`

### TF errors
- Ensure camera frames exist: `ros2 run tf2_ros tf2_echo base_link CameraTop_optical_frame`
- Check frame names match your robot's TF tree
- Verify camera_info is being published: `ros2 topic echo /camera_rgb_info`

### Fake depth inaccuracies
- The 1m fixed distance is a workaround; angular direction is accurate
- For precise distance control, integrate real depth information
- Consider using monocular depth estimation (e.g., Depth Anything) for better results

## Future Improvements

- [ ] Integration of real depth information from depth cameras
- [ ] Monocular depth estimation using neural networks (e.g., Depth Anything V2)
- [ ] PID controller tuning for smoother motion
- [ ] Multiple target tracking and selection
- [ ] Dynamic obstacle avoidance using additional sensors
- [ ] Path planning integration for complex environments

## License

This package is licensed under the Apache License 2.0.

## Authors and Contributors

- Rodrigo Pérez-Rodríguez (rodrigo.perez@urjc.es)

## Acknowledgments

- Built for the NAO humanoid robot by SoftBank Robotics
- Uses [yolo_ros](https://github.com/mgonzs13/yolo_ros) for object detection
- Part of the social robotics research at Universidad Rey Juan Carlos
