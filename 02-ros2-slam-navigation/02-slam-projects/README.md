

---

## Results and Analysis

The package includes a log analysis script:

    python3 scripts/analyze_logs.py

The script reads the latest CSV log from the logs/ folder and generates:

    results/summary_metrics.csv
    results/distance_battery_plot.png
    results/obstacle_complexity_plot.png
    results/velocity_adaptation_plot.png

### Example Adaptive Test Result

The adaptive test was performed by increasing the obstacle warning threshold:

    ros2 run tbot3_nav_monitor nav_monitor_node --ros-args -p obstacle_warning_distance:=2.0

Result summary:

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

This confirms that the adaptive speed reduction layer was activated under risky obstacle conditions and published reduced velocity commands on:

    /tbot3_nav_monitor/safe_cmd_vel

---

## Adaptive Safe Velocity Output

The topic `/tbot3_nav_monitor/safe_cmd_vel` is currently an adaptive safety output topic.

It demonstrates velocity reduction logic based on obstacle proximity, environment complexity, and recovery/stuck conditions.

It is not directly remapped to replace the robot's default `/cmd_vel` command in the standard TurtleBot3 pipeline. This design was chosen to avoid interfering with the default Gazebo/TurtleBot3 controller during testing. In a future closed-loop deployment, `/tbot3_nav_monitor/safe_cmd_vel` can be remapped to `/cmd_vel` or inserted as a safety layer between Nav2 and the robot controller.


---

## Goal-Based Navigation Metrics

The package includes a second node:

    nav_goal_metrics_node

This node listens to:

    /goal_pose
    /odom

It publishes goal-based metrics on:

    /tbot3_nav_monitor/goal_metrics

The goal metrics include:

    goal execution time
    current goal error
    final goal error
    actual path length during goal
    straight-line optimal path distance
    path efficiency

Path efficiency is estimated as:

    path_efficiency = straight_line_distance / actual_path_length

A value closer to 1.0 means the robot path is closer to the ideal straight-line path.


---

## Optional Live Dashboard

The package includes an optional terminal-based live dashboard:

    ros2 run tbot3_nav_monitor live_dashboard_node

The dashboard subscribes to:

    /tbot3_nav_monitor/status
    /tbot3_nav_monitor/goal_metrics

It displays live navigation metrics, adaptive status, recovery count, goal error, execution time, actual path length, optimal path length, and path efficiency.

This dashboard is useful for headless Gazebo testing and demo videos when the Gazebo GUI is unstable.
