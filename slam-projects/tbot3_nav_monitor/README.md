# TurtleBot3 Navigation Performance Monitor

This repository contains a ROS2 Humble package named `tbot3_nav_monitor` for monitoring TurtleBot3 Burger navigation performance in Gazebo simulation.

The package collects navigation metrics, logs data to CSV, publishes live status messages, estimates goal-based navigation performance, and provides an adaptive safe velocity output. It also includes a terminal-based live dashboard for headless Gazebo testing.

---

## Project Overview

The goal of this project is to monitor TurtleBot3 navigation performance in real time and provide useful metrics for evaluating navigation behavior.

The package monitors:

* Robot position
* Linear velocity
* Travelled distance
* Simulated battery consumption
* Minimum obstacle distance
* Front obstacle distance
* Environment complexity
* Command velocity activity
* Stuck/recovery behavior indicators
* Goal execution time
* Final goal error
* Actual path length
* Straight-line optimal path length
* Path efficiency

The package also publishes an adaptive safe velocity command on:

```bash
/tbot3_nav_monitor/safe_cmd_vel
```

This topic demonstrates a safety-aware velocity reduction layer based on obstacle proximity, environment complexity, and recovery/stuck conditions.

---

## Repository Structure

```text
tbot3_nav_monitor/
├── config/
│   └── monitor_params.yaml
├── launch/
│   └── monitor.launch.py
├── resource/
│   └── tbot3_nav_monitor
├── scripts/
│   └── analyze_logs.py
├── tbot3_nav_monitor/
│   ├── __init__.py
│   ├── nav_monitor_node.py
│   ├── nav_goal_metrics_node.py
│   └── live_dashboard_node.py
├── test/
├── Dockerfile
├── docker-compose.yml
├── package.xml
├── README.md
├── setup.cfg
└── setup.py
```

---

## Main ROS2 Nodes

### 1. `nav_monitor_node`

This is the main navigation monitoring node.

It subscribes to:

```bash
/odom
/scan
/cmd_vel
```

It publishes:

```bash
/tbot3_nav_monitor/status
/tbot3_nav_monitor/safe_cmd_vel
```

It computes and logs:

* Robot position
* Linear velocity
* Commanded velocity
* Safe adaptive velocity
* Incremental distance
* Total travelled distance
* Simulated battery level
* Minimum obstacle distance
* Front obstacle distance
* Environment complexity
* Close obstacle detection
* Stuck/recovery indicators
* System status

---

### 2. `nav_goal_metrics_node`

This node monitors goal-based navigation performance.

It subscribes to:

```bash
/goal_pose
/odom
```

It publishes:

```bash
/tbot3_nav_monitor/goal_metrics
```

It computes:

* Goal count
* Completed goals
* Goal active status
* Current goal error
* Final goal error
* Goal execution time
* Actual path length
* Straight-line optimal path distance
* Path efficiency

Path efficiency is estimated as:

```text
path_efficiency = straight_line_distance / actual_path_length
```

A value closer to `1.0` means the robot path is closer to the ideal straight-line path.

---

### 3. `live_dashboard_node`

This is an optional terminal-based live dashboard.

It subscribes to:

```bash
/tbot3_nav_monitor/status
/tbot3_nav_monitor/goal_metrics
```

It displays live metrics such as:

* Distance travelled
* Battery level
* Front obstacle distance
* Environment complexity
* Adaptation status
* Recovery count
* Goal error
* Execution time
* Actual path
* Optimal path
* Path efficiency

Run it with:

```bash
ros2 run tbot3_nav_monitor live_dashboard_node
```

This dashboard is useful for headless Gazebo testing and demo videos when Gazebo GUI is unstable.

---

## Requirements

Tested environment:

```text
Ubuntu 22.04
ROS2 Humble
Gazebo 11
TurtleBot3 Burger
Python 3.10
```

Required ROS2/TurtleBot3 packages include:

```bash
ros-humble-turtlebot3
ros-humble-turtlebot3-gazebo
ros-humble-navigation2
ros-humble-nav2-bringup
```

Python dependencies:

```bash
python3-pandas
python3-matplotlib
python3-rich
```

Install Python dependencies:

```bash
sudo apt update
sudo apt install -y python3-pandas python3-matplotlib python3-rich
```

---

## Build Instructions

Go to the ROS2 workspace:

```bash
cd ~/tbot3_nav_monitor_ws
```

Build the package:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Check available executables:

```bash
ros2 pkg executables tbot3_nav_monitor
```

Expected output:

```text
tbot3_nav_monitor live_dashboard_node
tbot3_nav_monitor nav_goal_metrics_node
tbot3_nav_monitor nav_monitor_node
```

---

## Running TurtleBot3 Gazebo

Set TurtleBot3 model:

```bash
export TURTLEBOT3_MODEL=burger
```

Run Gazebo in headless mode:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
```

Gazebo GUI was unstable on the development machine, so the package was validated using headless Gazebo. Since the monitor uses ROS2 topics such as `/odom`, `/scan`, `/cmd_vel`, and `/goal_pose`, the GUI is not required for validating the monitoring logic.

---

## Running the Monitor

In a new terminal:

```bash
cd ~/tbot3_nav_monitor_ws
source install/setup.bash
ros2 launch tbot3_nav_monitor monitor.launch.py
```

This launch file starts:

```text
nav_monitor_node
nav_goal_metrics_node
```

---

## Running the Live Dashboard

In another terminal:

```bash
source ~/tbot3_nav_monitor_ws/install/setup.bash
ros2 run tbot3_nav_monitor live_dashboard_node
```

The dashboard displays live navigation and goal metrics in the terminal.

---

## Checking ROS2 Topics

List package topics:

```bash
ros2 topic list | grep tbot3_nav_monitor
```

Expected topics:

```text
/tbot3_nav_monitor/status
/tbot3_nav_monitor/safe_cmd_vel
/tbot3_nav_monitor/goal_metrics
```

Check monitor status:

```bash
ros2 topic echo /tbot3_nav_monitor/status --once
```

Check goal metrics:

```bash
ros2 topic echo /tbot3_nav_monitor/goal_metrics --once
```

---

## Testing Goal Metrics

Publish a nearby goal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
header: {frame_id: 'map'},
pose: {
  position: {x: -1.9, y: -0.4, z: 0.0},
  orientation: {w: 1.0}
}
}"
```

Then check goal metrics:

```bash
ros2 topic echo /tbot3_nav_monitor/goal_metrics --once
```

Example output:

```text
goal_count=1
completed_goals=1
goal_active=0
goal_completed=1
goal_error=0.141
final_goal_error=0.141
execution_time=0.03
```

---

## Adaptive Safe Velocity Output

The topic:

```bash
/tbot3_nav_monitor/safe_cmd_vel
```

is currently an adaptive safety output topic.

It demonstrates velocity reduction logic based on:

* Obstacle proximity
* Environment complexity
* Recovery/stuck conditions

It is not directly remapped to replace the robot's default `/cmd_vel` command in the standard TurtleBot3 pipeline. This design was chosen to avoid interfering with the default Gazebo/TurtleBot3 controller during testing.

In a future closed-loop deployment, `/tbot3_nav_monitor/safe_cmd_vel` can be remapped to `/cmd_vel` or inserted as a safety layer between Nav2 and the robot controller.

---

## Running an Adaptive Test

The adaptive test can be performed by increasing the obstacle warning threshold:

```bash
ros2 run tbot3_nav_monitor nav_monitor_node --ros-args -p obstacle_warning_distance:=2.0
```

This forces the monitor to treat more obstacle situations as risky and activate the adaptive velocity reduction more frequently.

---

## Results and Analysis

The package includes a log analysis script:

```bash
python3 scripts/analyze_logs.py
```

The script reads the latest CSV log from the `logs/` folder and generates summary metrics and result plots, including:

```text
results/summary_metrics.csv
results/distance_battery_plot.png
results/obstacle_complexity_plot.png
results/velocity_adaptation_plot.png
```

Note: Generated result plots and the full submission package are also available in the backup submission zip file when provided.

---

## Example Adaptive Test Result

The adaptive test was performed by increasing the obstacle warning threshold.

Result summary:

```text
Total time: 77.321 s
Total distance: 7.507 m
Final simulated battery: 84.99 %
Average velocity: 0.098 m/s
Maximum velocity: 0.228 m/s
Average front obstacle distance: 0.756 m
Minimum front obstacle distance: 0.283 m
Average environment complexity: 0.427
Maximum environment complexity: 0.495
Recovery events: 0
Adaptation active samples: 1848
Adaptation active percentage: 81.481 %
```

This confirms that the adaptive speed reduction layer was activated under risky obstacle conditions and published reduced velocity commands on:

```bash
/tbot3_nav_monitor/safe_cmd_vel
```

---

## Docker Support

The repository includes:

```text
Dockerfile
docker-compose.yml
```

The Docker setup is intended to support reproducible deployment of the ROS2 package. It includes the ROS2 Humble environment, TurtleBot3 dependencies, and the local package build.

Example command:

```bash
docker-compose up --build
```

Then enter the container if needed:

```bash
docker exec -it tbot3_nav_monitor_container bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/tbot3_nav_monitor_ws/install/setup.bash
ros2 launch tbot3_nav_monitor monitor.launch.py
```

---

## Demo Video Status

A short demo video is currently being prepared.

The project can already be demonstrated using headless Gazebo and the terminal-based live dashboard.

Recommended demo sequence:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
```

```bash
cd ~/tbot3_nav_monitor_ws
source install/setup.bash
ros2 launch tbot3_nav_monitor monitor.launch.py
```

```bash
ros2 run tbot3_nav_monitor live_dashboard_node
```

```bash
ros2 topic echo /tbot3_nav_monitor/status --once
```

```bash
ros2 topic echo /tbot3_nav_monitor/goal_metrics --once
```

---

## Current Testing Status

The current implementation was tested using TurtleBot3 Gazebo in headless mode and adaptive forced-risk conditions.

Implemented features:

* ROS2 monitoring node
* Goal-based metrics node
* Terminal live dashboard
* CSV logging
* Simulated battery consumption
* Adaptive safe velocity output
* Dockerfile and docker-compose support
* Result analysis script

---

## Future Work

Future work includes:

* Testing in additional Gazebo worlds such as TurtleBot3 house and custom narrow-passage worlds
* Direct remapping of `/safe_cmd_vel` into the robot command pipeline
* Docker Hub publishing
* Web-based dashboard support
* Extended Nav2 integration with full autonomous goal execution
* More systematic comparison across multiple navigation environments

---

## Notes

The package is designed to monitor TurtleBot3 navigation without interfering with the default robot controller. The adaptive safe velocity output is published separately for safety-layer demonstration and future integration.

The complete project source is available in this repository, and a backup zip package is also available when required for submission.
