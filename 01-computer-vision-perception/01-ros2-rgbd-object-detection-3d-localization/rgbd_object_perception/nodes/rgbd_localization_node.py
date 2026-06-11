import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from rgbd_object_perception.core.detector import YoloDetector
from rgbd_object_perception.core.depth_to_3d import get_median_depth, pixel_to_3d
from rgbd_object_perception.utils.drawing import draw_detections


class RGBDLocalizationNode(Node):
    def __init__(self):
        super().__init__("rgbd_localization_node")

        self.bridge = CvBridge()

        # =====================================================
        # ROS2 parameters
        # =====================================================
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("object_filter", "")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        self.model_name = self.get_parameter("model_name").value
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.object_filter = (
            self.get_parameter("object_filter").value.strip().lower()
        )
        self.camera_frame = self.get_parameter("camera_frame").value

        # =====================================================
        # YOLO detector
        # =====================================================
        self.detector = YoloDetector(
            model_name=self.model_name,
            confidence_threshold=self.confidence_threshold
        )

        self.get_logger().info(f"YOLO model: {self.model_name}")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")

        if self.object_filter:
            self.get_logger().info(f"Object filter enabled: {self.object_filter}")
        else:
            self.get_logger().info("Object filter disabled. Showing all detections.")

        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        # =====================================================
        # RealSense ROS2 topics
        # =====================================================
        self.rgb_topic = "/camera/camera/color/image_raw"
        self.depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/camera/color/camera_info"

        # =====================================================
        # Depth image and camera intrinsics
        # =====================================================
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # =====================================================
        # Subscribers
        # =====================================================
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # =====================================================
        # Publishers
        # =====================================================
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/detected_object_pose",
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/detected_object_marker",
            10
        )

        self.get_logger().info("RGB-D localization node started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info("Publishing PoseStamped: /detected_object_pose")
        self.get_logger().info("Publishing RViz Marker: /detected_object_marker")

    def camera_info_callback(self, msg):
        """
        Read camera intrinsics from CameraInfo message.

        Camera matrix K:
            [fx  0 cx]
            [ 0 fy cy]
            [ 0  0  1]
        """

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        """
        Store latest aligned depth image.
        """

        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

    def publish_pose(self, class_name, x, y, z):
        """
        Publish object 3D position as PoseStamped.
        Orientation is identity because this node estimates position only.
        """

        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

    def publish_marker(self, class_name, x, y, z):
        """
        Publish object 3D position as RViz marker.
        """

        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.camera_frame

        marker.ns = "detected_objects"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

    def rgb_callback(self, msg):
        """
        Main callback:
        1. Read RGB image
        2. Detect objects using YOLO
        3. Optionally filter object class
        4. Read depth at bounding-box center
        5. Convert pixel + depth to X, Y, Z
        6. Publish PoseStamped and RViz Marker
        7. Visualize result
        """

        if self.latest_depth is None:
            self.get_logger().warn("Waiting for aligned depth image...")
            return

        if self.fx is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        image_bgr = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        detections = self.detector.detect(image_bgr)

        if self.object_filter:
            detections = [
                det for det in detections
                if det["class_name"].lower() == self.object_filter
            ]

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]

            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            depth_m = get_median_depth(
                self.latest_depth,
                u,
                v,
                window_size=9,
                depth_scale=0.001
            )

            if depth_m is None or depth_m <= 0.0:
                det["position_3d"] = None
                continue

            x, y, z = pixel_to_3d(
                u,
                v,
                depth_m,
                self.fx,
                self.fy,
                self.cx,
                self.cy
            )

            det["position_3d"] = (x, y, z)

            self.publish_pose(det["class_name"], x, y, z)
            self.publish_marker(det["class_name"], x, y, z)

            self.get_logger().info(
                f'{det["class_name"]}: '
                f'conf={det["confidence"]:.2f}, '
                f'pixel=({u},{v}), '
                f'depth={depth_m:.3f}m, '
                f'X={x:.3f}m, '
                f'Y={y:.3f}m, '
                f'Z={z:.3f}m | '
                f'Published /detected_object_pose and /detected_object_marker'
            )

        output = draw_detections(image_bgr, detections)

        cv2.imshow("RGB-D Object Localization", output)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = RGBDLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
