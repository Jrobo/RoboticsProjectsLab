# ROS2 RGB-D Object Detection and 3D Localization for Robotic Perception

## Overview

This project implements a ROS2-based robotic perception pipeline for real-time object detection using YOLO and RGB image streams. The current version provides a modular ROS2 Humble package that subscribes to an RGB camera topic, runs YOLO object detection, and visualizes detected objects using OpenCV.

The project is designed to be extended with Intel RealSense RGB-D input, depth-based 3D localization, point cloud processing, and ROS2 object pose publishing.

## Application

The target application is tabletop robotic perception for object picking and assistive robotics. The system will detect common objects such as bottles, cups, books, boxes, mouse, and mobile phones, estimate their 3D position using depth data, and publish object poses for robotic manipulation.

## Current Milestone

Milestone 1: ROS2 YOLO RGB Detection Node

- ROS2 Humble Python package created
- YOLO detector module implemented
- OpenCV visualization utility implemented
- ROS2 image subscriber node implemented
- Node subscribes to `/camera/camera/color/image_raw`
- RealSense-ready RGB image pipeline prepared

## Planned Pipeline

Intel RealSense RGB-D Camera  
→ RGB image and aligned depth image  
→ YOLO object detection  
→ depth extraction at object center  
→ 2D pixel to 3D coordinate conversion  
→ ROS2 object pose publishing  
→ RViz / OpenCV / Open3D visualization

## Tools and Libraries

- ROS2 Humble
- Python
- OpenCV
- cv_bridge
- Ultralytics YOLO
- Intel RealSense ROS2 wrapper
- Open3D
- NumPy
- Git

## Package Structure

```text
rgbd_object_perception/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── rgbd_object_perception/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   └── image_yolo_node.py
│   ├── core/
│   │   ├── __init__.py
│   │   └── detector.py
│   └── utils/
│       ├── __init__.py
│       └── drawing.py
└── test/
