import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from rgbd_object_perception.core.detector import YoloDetector
from rgbd_object_perception.core.depth_to_3d import get_median_depth, pixel_to_3d
from rgbd_object_perception.utils.drawing import draw_detections


class RGBDLocalizationNode(Node):
    def __init__(self):
        super().__init__("rgbd_localization_node")

        self.bridge = CvBridge()

        self.detector = YoloDetector(
            model_name="yolov8n.pt",
            confidence_threshold=0.35
        )

        self.rgb_topic = "/camera/camera/color/image_raw"
        self.depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/camera/color/camera_info"

        self.latest_depth = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

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

        self.get_logger().info("RGB-D localization node started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")

    def camera_info_callback(self, msg):
        # Camera matrix K:
        # [fx  0 cx]
        # [ 0 fy cy]
        # [ 0  0  1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def rgb_callback(self, msg):
        if self.latest_depth is None:
            self.get_logger().warn("Waiting for aligned depth image...")
            return

        if self.fx is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detections = self.detector.detect(image_bgr)

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

            self.get_logger().info(
                f'{det["class_name"]}: conf={det["confidence"]:.2f}, '
                f'pixel=({u},{v}), depth={depth_m:.3f}m, '
                f'X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m'
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
