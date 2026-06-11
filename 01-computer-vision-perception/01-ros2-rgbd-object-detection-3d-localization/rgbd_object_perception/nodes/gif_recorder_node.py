import time
from pathlib import Path

import cv2
import imageio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rgbd_object_perception.core.detector import YoloDetector
from rgbd_object_perception.utils.drawing import draw_detections


class GifRecorderNode(Node):
    def __init__(self):
        super().__init__("gif_recorder_node")

        self.bridge = CvBridge()
        self.detector = YoloDetector(
            model_name="yolov8n.pt",
            confidence_threshold=0.35
        )

        self.rgb_topic = "/camera/camera/color/image_raw"

        object_name = input("Enter object name for GIF file, e.g., banana: ").strip()
        if object_name == "":
            object_name = "object"

        duration_text = input("Enter recording duration in seconds [default 5]: ").strip()
        self.record_duration = float(duration_text) if duration_text else 5.0

        self.object_name = object_name.replace(" ", "_").lower()

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
        self.gif_path = self.output_dir / f"{self.object_name}_detection_{timestamp}.gif"

        self.frames = []
        self.start_time = None
        self.frame_count = 0
        self.finished = False

        self.subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            10
        )

        self.get_logger().info("GIF recorder node started.")
        self.get_logger().info(f"Listening to: {self.rgb_topic}")
        self.get_logger().info(f"GIF will be saved to: {self.gif_path}")

    def image_callback(self, msg):
        if self.finished:
            return

        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time

        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detections = self.detector.detect(image_bgr)
        output = draw_detections(image_bgr, detections)

        cv2.putText(
            output,
            f"Recording GIF: {elapsed:.1f}/{self.record_duration:.1f}s",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.imshow("YOLO GIF Recorder", output)
        cv2.waitKey(1)

        self.frame_count += 1

        # Save every 3rd frame to keep GIF small
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

        self.get_logger().info("GIF saved successfully.")
        self.get_logger().info(f"Saved at: {self.gif_path}")


def main(args=None):
    rclpy.init(args=args)
    node = GifRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
