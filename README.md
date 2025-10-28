# smart_robot_ai_perception (ROS2)

**ROS 2-based AI Perception & Control for a mobile robot (e.g., TurtleBot3).**
- YOLOv8 (if available) or a **fake detector** fallback
- LIDAR processing + camera-LIDAR **fusion**
- Simple **PID controller** to follow the selected target
- Launch file to bring up the full pipeline

> Tested conceptually with ROS 2 Humble on Ubuntu 22.04. Simulation can be done using TurtleBot3 Gazebo/rviz.


## Features
- `/camera/image_raw` ➜ **Detector** ➜ `/detections` (JSON in `std_msgs/String`) + `/detections_markers` (RViz)
- `/scan` ➜ **Lidar Node** ➜ `/scan_cache` (internal) for fusion
- **Fusion** picks the most relevant vision detection, estimates angle + distance (via LIDAR) ➜ `/fusion/target` (`geometry_msgs/PointStamped` with x=distance [m], y=angle [rad])
- **Controller** uses a PID to generate `/cmd_vel` (`geometry_msgs/Twist`) to follow target

## Quickstart

```bash
# 0) Prereqs (Ubuntu 22.04, ROS 2 Humble)
#    Install ROS 2 Humble per the official guide.

# 1) Create workspace and clone this package
mkdir -p ~/ai_perception_ws/src
# copy this folder into ~/ai_perception_ws/src (or unzip then move)

# 2) Install Python deps (in your system / venv)
pip install -r src/smart_robot_ai_perception/requirements.txt

# 3) Build
cd ~/ai_perception_ws
colcon build --symlink-install
source install/setup.bash

# 4) Run (camera+lidar topics must be available; use TurtleBot3 sim or a bag)
ros2 launch smart_robot_ai_perception sim_perception.launch.py
```

### Simulation tips (TurtleBot3)
- Set `export TURTLEBOT3_MODEL=burger`
- Launch a TB3 Gazebo world that publishes `/camera/image_raw` and `/scan` (e.g., add a camera plugin or use a depth camera node).
- Use RViz to view `/detections_markers` and `/fusion/target`.

## Parameters
Configured in `config/params.yaml`:
- `detector.backend`: `yolo` | `fake` | `auto`
- `detector.model_path`: path to YOLO weights (`.pt`)
- `camera.fov_deg`: Horizontal FOV to convert pixel → angle
- `controller` gains, speed limits

## Package Layout

```
smart_robot_ai_perception/
├── package.xml
├── setup.py
├── requirements.txt
├── resource/smart_robot_ai_perception
├── smart_robot_ai_perception/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── detector_node.py
│   │   ├── lidar_node.py
│   │   ├── fusion_node.py
│   │   └── controller_node.py
│   ├── utils/
│   │   ├── yolov8_detector.py
│   │   ├── fake_detector.py
│   │   └── pid.py
│   └── launch/sim_perception.launch.py
└── config/params.yaml
```

## Roadmap
- [ ] Depth camera support for direct distance without LIDAR
- [ ] Multi-object tracking (SORT/OC-SORT)
- [ ] Re-ID and class-specific following
- [ ] Nav2 integration & obstacle avoidance blend
- [ ] Docker and devcontainer
