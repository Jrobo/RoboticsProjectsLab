import time
from pathlib import Path

import cv2
import imageio
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from rgbd_object_perception.core.detector import YoloDetector
from rgbd_object_perception.core.depth_to_3d import get_median_depth, pixel_to_3d
from rgbd_object_perception.utils.drawing import draw_detections


class RGBDLocalizationGifRecorderNode(Node):
    def __init__(self):
        super().__init__("rgbd_localization_gif_recorder_node")

        self.bridge = CvBridge()

        self.detector = YoloDetector(
            model_name="yolov8n.pt",
            confidence_threshold=0.35
        )

        self.rgb_topic = "/camera/camera/color/image_raw"
        self.depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/camera/color/camera_info"

        self.expected_object = input(
            "Enter object to record, e.g., banana, cell phone, scissors [press Enter for all]: "
        ).strip().lower()

        duration_text = input("Enter recording duration in seconds [default 5]: ").strip()
        self.record_duration = float(duration_text) if duration_text else 5.0

        name = self.expected_object if self.expected_object else "multi_object"
        self.safe_name = name.replace(" ", "_").lower()

        self.output_dir = (
            Path.home()
            / "ros2_perception_ws"
            / "src"
            / "rgbd_object_perception"
            / "results"
            / "demo_gifs"
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.gif_path = self.output_dir / f"{self.safe_name}_3d_localization_{timestamp}.gif"

        self.latest_depth = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.frames = []
        self.frame_count = 0
        self.start_time = None
        self.finished = False

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

        self.get_logger().info("RGB-D localization GIF recorder started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"GIF will be saved to: {self.gif_path}")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

    def rgb_callback(self, msg):
        if self.finished:
            return

        if self.latest_depth is None or self.fx is None:
            self.get_logger().warn("Waiting for depth image and camera intrinsics...")
            return

        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time

        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detections = self.detector.detect(image_bgr)

        # Filter only expected object for clean single-object GIF
        if self.expected_object:
            detections = [
                det for det in detections
                if det["class_name"].lower() == self.expected_object
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

        output = draw_detections(image_bgr, detections)

        cv2.putText(
            output,
            f"RGB-D 3D Localization | Recording: {elapsed:.1f}/{self.record_duration:.1f}s",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        cv2.imshow("RGB-D 3D Localization GIF Recorder", output)
        cv2.waitKey(1)

        self.frame_count += 1

        # Save every 3rd frame to keep GIF size reasonable
        if self.frame_count % 3 == 0:
            frame_rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
            self.frames.append(frame_rgb)

        if elapsed >= self.record_duration:
            self.finished = True
            self.save_gif()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def save_gif(self):
        if len(self.frames) == 0:
            self.get_logger().error("No frames captured. GIF not saved.")
            return

        imageio.mimsave(self.gif_path, self.frames, fps=8)

        self.get_logger().info("3D localization GIF saved successfully.")
        self.get_logger().info(f"Saved at: {self.gif_path}")


def main(args=None):
    rclpy.init(args=args)
    node = RGBDLocalizationGifRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
