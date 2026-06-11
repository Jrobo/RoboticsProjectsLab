# ROS2 RGB-D Object Detection and 3D Localization

This project implements a real-time robotic perception pipeline using an Intel RealSense D435i RGB-D camera, YOLOv8 object detection, depth-based 3D localization, Open3D point-cloud processing, and ROS2 integration.

The system detects objects in RGB images, estimates their 3D position using aligned depth data and camera intrinsics, publishes the detected object pose as a ROS2 `PoseStamped` message, and visualizes the result using RViz markers. This project demonstrates a complete perception workflow suitable for robotic manipulation, human-robot interaction, and autonomous robotic systems.

---

## 1. Project Overview

Modern robots need to understand not only what objects are present in the environment, but also where those objects are located in 3D space. This project combines deep learning-based object detection with RGB-D sensing to estimate object positions relative to the camera frame.

The main goal is to convert a 2D YOLO object detection into a 3D object position using depth information from the RealSense camera.

Example output:

```text
banana: conf=0.59, pixel=(552,363), depth=0.507m, X=-0.057m, Y=0.000m, Z=0.507m
```

---

## 2. System Architecture

```text
Intel RealSense D435i
        |
        | RGB image
        | Aligned depth image
        | Camera intrinsics
        v
ROS2 RealSense Camera Node
        |
        v
YOLOv8 Object Detection
        |
        v
Bounding Box Center Pixel
        |
        v
Depth Extraction Around Object Center
        |
        v
Pixel-to-3D Projection
        |
        v
3D Object Position: X, Y, Z
        |
        +----------------------------+
        |                            |
        v                            v
ROS2 PoseStamped Publisher      RViz Marker Publisher
/detected_object_pose           /detected_object_marker
```

---

## 3. Key Features

* Real-time RGB image acquisition from Intel RealSense D435i
* Aligned depth image processing
* YOLOv8 object detection
* Depth-based 3D object localization
* Camera intrinsic-based pixel-to-3D projection
* ROS2 parameter support
* Object class filtering
* OpenCV visualization
* GIF result recording for demonstration
* Open3D point-cloud crop around detected object
* ROS2 `PoseStamped` publishing
* RViz marker publishing for robotics visualization

---

## 4. Hardware and Software

### Hardware

* Intel RealSense D435i RGB-D camera
* Ubuntu machine with USB 3.0 connection

### Software

* Ubuntu 22.04
* ROS2 Humble
* Intel RealSense ROS2 wrapper
* Python 3
* OpenCV
* Ultralytics YOLOv8
* Open3D
* NumPy
* cv_bridge

---

## 5. ROS2 Topics Used

### Input Topics

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

### Output Topics

```text
/detected_object_pose
/detected_object_marker
```

### Output Message Types

```text
/detected_object_pose      geometry_msgs/msg/PoseStamped
/detected_object_marker    visualization_msgs/msg/Marker
```

---

## 6. How 3D Localization Works

YOLOv8 detects an object in the RGB image and returns a 2D bounding box.

The center of the bounding box is calculated as:

```text
u = (x1 + x2) / 2
v = (y1 + y2) / 2
```

The aligned depth image provides the depth value at or around this pixel location.

Using the camera intrinsics from `CameraInfo`, the 2D pixel and depth are projected into 3D camera coordinates:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

Where:

```text
fx, fy = focal lengths
cx, cy = optical center
u, v   = pixel coordinates
Z      = depth value
X, Y, Z = 3D object position in camera frame
```

---

## 7. How to Run

### Step 1: Start the RealSense Camera

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### Step 2: Run RGB-D Object Localization

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_perception_ws/install/setup.bash

ros2 run rgbd_object_perception rgbd_localization_node
```

### Step 3: Run with Object Filter

Example for banana detection only:

```bash
ros2 run rgbd_object_perception rgbd_localization_node --ros-args -p object_filter:=banana
```

Example for cell phone detection only:

```bash
ros2 run rgbd_object_perception rgbd_localization_node --ros-args -p object_filter:="cell phone"
```

---

## 8. ROS2 Parameters

The localization node supports the following parameters:

```text
model_name              YOLO model file name
confidence_threshold    Minimum YOLO confidence
object_filter           Optional object class filter
camera_frame            Camera frame used for published pose and marker
```

Example:

```bash
ros2 run rgbd_object_perception rgbd_localization_node --ros-args \
-p model_name:=yolov8n.pt \
-p confidence_threshold:=0.35 \
-p object_filter:=banana \
-p camera_frame:=camera_color_optical_frame
```

---

## 9. Open3D Point-Cloud Crop

The project also includes an Open3D-based point-cloud crop node. It extracts a local point cloud from the detected object bounding box using the aligned depth image.

Run:

```bash
ros2 run rgbd_object_perception pointcloud_crop_node --ros-args \
-p confidence_threshold:=0.20 \
-p voxel_size:=0.003 \
-p min_points:=20
```

The point cloud is saved as a `.ply` file inside:

```text
results/pointclouds/
```

Example saved output:

```text
banana_crop_YYYYMMDD_HHMMSS.ply
```

---

## 10. RViz Visualization

Run RViz:

```bash
rviz2
```

In RViz:

```text
1. Set Fixed Frame to camera_color_optical_frame
2. Add Marker topic
3. Select /detected_object_marker
```

A green sphere marker will appear at the detected object location.

---

## 11. Results

Example detected object localization:

```text
banana: conf=0.59, pixel=(552,363), depth=0.507m, X=-0.057m, Y=0.000m, Z=0.507m
```

This confirms that the system successfully detects the object and estimates its 3D position relative to the camera.

Demonstration results include:

* Real-time YOLO object detection
* RGB-D 3D object localization
* Object-specific filtering
* Open3D point-cloud crop
* ROS2 `PoseStamped` output
* RViz marker visualization

---

## 12. Project Structure

```text
rgbd_object_perception/
├── core/
│   ├── detector.py
│   ├── depth_to_3d.py
│   └── pointcloud_utils.py
├── nodes/
│   ├── image_yolo_node.py
│   ├── gif_recorder_node.py
│   ├── rgbd_localization_node.py
│   ├── rgbd_localization_gif_recorder_node.py
│   └── pointcloud_crop_node.py
├── utils/
│   └── drawing.py
├── launch/
│   └── rgbd_localization.launch.py
├── results/
│   ├── demo_gifs/
│   └── pointclouds/
├── package.xml
├── setup.py
└── README.md
```

---

## 13. Interview Explanation

This project demonstrates a complete robotic perception pipeline. I used an Intel RealSense D435i camera to collect RGB and aligned depth images. YOLOv8 detects objects in the RGB image, and the aligned depth image is used to estimate the distance of the object from the camera. By combining the bounding box center, depth value, and camera intrinsic parameters, I converted 2D detections into 3D object positions.

I then published the 3D position as a ROS2 `PoseStamped` message and visualized it in RViz using a marker. I also added Open3D point-cloud cropping to extract the local 3D point cloud of the detected object. This makes the project useful for robotic manipulation, object tracking, and human-robot interaction.

---

## 14. Future Work

Planned improvements include:

* Export YOLO model to ONNX
* Add ONNX Runtime inference
* Add TensorRT acceleration for embedded deployment
* Add FP16 or INT8 quantization
* Add Dockerfile for reproducible setup
* Add GitHub Actions CI/CD for automatic ROS2 build testing
* Publish multiple object poses
* Add object tracking over time
* Integrate with a robot manipulator or mobile robot navigation stack

---

## 15. Skills Demonstrated

* ROS2 node development
* RealSense RGB-D camera integration
* Computer vision
* YOLOv8 object detection
* Depth image processing
* Camera intrinsic geometry
* 2D-to-3D projection
* OpenCV visualization
* Open3D point-cloud processing
* RViz visualization
* ROS2 message publishing
* Git and GitHub workflow
