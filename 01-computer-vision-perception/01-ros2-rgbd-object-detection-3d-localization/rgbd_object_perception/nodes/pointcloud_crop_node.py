import time
from pathlib import Path

import cv2
import open3d as o3d
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from rgbd_object_perception.core.detector import YoloDetector
from rgbd_object_perception.core.depth_to_3d import get_median_depth, pixel_to_3d
from rgbd_object_perception.core.pointcloud_utils import (
    bbox_depth_to_pointcloud,
    voxel_downsample_pointcloud,
)
from rgbd_object_perception.utils.drawing import draw_detections


class PointCloudCropNode(Node):
    def __init__(self):
        super().__init__("pointcloud_crop_node")

        self.bridge = CvBridge()

        # =====================================================
        # ROS2 parameters
        # =====================================================
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("object_filter", "")
        self.declare_parameter("save_once", True)
        self.declare_parameter("voxel_size", 0.003)
        self.declare_parameter("min_points", 50)

        self.model_name = self.get_parameter("model_name").value
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.object_filter = (
            self.get_parameter("object_filter").value.strip().lower()
        )
        self.save_once = bool(self.get_parameter("save_once").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.min_points = int(self.get_parameter("min_points").value)

        # =====================================================
        # YOLO detector
        # =====================================================
        self.detector = YoloDetector(
            model_name=self.model_name,
            confidence_threshold=self.confidence_threshold,
        )

        # =====================================================
        # RealSense topics
        # =====================================================
        self.rgb_topic = "/camera/camera/color/image_raw"
        self.depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/camera/color/camera_info"

        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.saved = False
        self.frame_counter = 0

        # =====================================================
        # Output folder
        # =====================================================
        self.output_dir = (
            Path.home()
            / "ros2_perception_ws"
            / "src"
            / "rgbd_object_perception"
            / "results"
            / "pointclouds"
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # =====================================================
        # Subscribers
        # =====================================================
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.get_logger().info("Point cloud crop node started.")
        self.get_logger().info(f"YOLO model: {self.model_name}")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")

        if self.object_filter:
            self.get_logger().info(f"Object filter enabled: {self.object_filter}")
        else:
            self.get_logger().info("Object filter disabled. First valid detected object will be saved.")

        self.get_logger().info(f"Save once: {self.save_once}")
        self.get_logger().info(f"Voxel size: {self.voxel_size}")
        self.get_logger().info(f"Minimum points required: {self.min_points}")
        self.get_logger().info(f"Point clouds will be saved to: {self.output_dir}")

    def camera_info_callback(self, msg):
        """
        Read camera intrinsics from RealSense CameraInfo.
        """

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        """
        Store the latest aligned depth image.
        """

        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        )

    def rgb_callback(self, msg):
        """
        Main callback:
        1. Read RGB image
        2. Detect object using YOLO
        3. Read aligned depth
        4. Convert object bbox pixels to point cloud
        5. Voxel downsample
        6. Save .ply file
        """

        self.frame_counter += 1

        if self.save_once and self.saved:
            return

        if self.latest_depth is None:
            self.get_logger().warn("Waiting for aligned depth image...")
            return

        if self.fx is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        image_bgr = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

        detections = self.detector.detect(image_bgr)

        # Print detected classes every 20 frames
        if self.frame_counter % 20 == 0:
            if len(detections) == 0:
                self.get_logger().info("No YOLO detections in current frame.")
            else:
                detected_names = [
                    f'{det["class_name"]}:{det["confidence"]:.2f}'
                    for det in detections
                ]
                self.get_logger().info(f"Detected classes: {detected_names}")

        # Optional object filter
        if self.object_filter:
            detections = [
                det for det in detections
                if det["class_name"].lower() == self.object_filter
            ]

            if self.frame_counter % 20 == 0 and len(detections) == 0:
                self.get_logger().info(
                    f"No detections matched object_filter='{self.object_filter}'."
                )

        valid_detections_for_display = []

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]

            # Bounding box center
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # Get robust depth at bbox center
            depth_m = get_median_depth(
                self.latest_depth,
                u,
                v,
                window_size=9,
                depth_scale=0.001,
            )

            if depth_m is None or depth_m <= 0.0:
                self.get_logger().warn(
                    f'Invalid depth for {det["class_name"]} at pixel ({u}, {v}).'
                )
                det["position_3d"] = None
                valid_detections_for_display.append(det)
                continue

            # Pixel + depth to 3D point
            x, y, z = pixel_to_3d(
                u,
                v,
                depth_m,
                self.fx,
                self.fy,
                self.cx,
                self.cy,
            )

            det["position_3d"] = (x, y, z)
            valid_detections_for_display.append(det)

            self.get_logger().info(
                f'{det["class_name"]}: '
                f'conf={det["confidence"]:.2f}, '
                f'pixel=({u},{v}), '
                f'depth={depth_m:.3f}m, '
                f'X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m'
            )

            # Create local point cloud crop from bbox
            pcd = bbox_depth_to_pointcloud(
                rgb_image_bgr=image_bgr,
                depth_image=self.latest_depth,
                bbox=det["bbox"],
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                depth_scale=0.001,
                stride=2,
                max_depth_m=2.0,
            )

            raw_points = len(pcd.points)

            if raw_points == 0:
                self.get_logger().warn(
                    f'Point cloud for {det["class_name"]} has 0 raw points.'
                )
                continue

            pcd = voxel_downsample_pointcloud(
                pcd,
                voxel_size=self.voxel_size,
            )

            downsampled_points = len(pcd.points)

            self.get_logger().info(
                f'Point cloud points for {det["class_name"]}: '
                f'raw={raw_points}, downsampled={downsampled_points}'
            )

            if downsampled_points < self.min_points:
                self.get_logger().warn(
                    f'Point cloud too small for {det["class_name"]}: '
                    f'{downsampled_points} points. Not saving.'
                )
                continue

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            safe_name = det["class_name"].lower().replace(" ", "_")

            ply_path = self.output_dir / f"{safe_name}_crop_{timestamp}.ply"

            success = o3d.io.write_point_cloud(str(ply_path), pcd)

            if success:
                self.get_logger().info(
                    f"Saved object point cloud: {ply_path} "
                    f"with {downsampled_points} points"
                )
                self.saved = True
            else:
                self.get_logger().error(f"Failed to save point cloud: {ply_path}")

            if self.save_once and self.saved:
                break

        output = draw_detections(image_bgr, valid_detections_for_display)

        cv2.putText(
            output,
            "Point Cloud Crop Node | Ctrl+C to stop",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        cv2.imshow("YOLO RGB-D Point Cloud Crop", output)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudCropNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
