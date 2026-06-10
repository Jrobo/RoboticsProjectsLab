import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rgbd_object_perception.core.detector import YoloDetector
from rgbd_object_perception.utils.drawing import draw_detections


class ImageYoloNode(Node):
    def __init__(self):
        super().__init__("image_yolo_node")

        self.bridge = CvBridge()
        self.detector = YoloDetector(
            model_name="yolov8n.pt",
            confidence_threshold=0.4
        )

        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("Image YOLO node started.")
        self.get_logger().info("Waiting for image topic: /camera/camera/color/image_raw")

    def image_callback(self, msg):
        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detections = self.detector.detect(image_bgr)
        output = draw_detections(image_bgr, detections)

        cv2.imshow("YOLO RGB Detection", output)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

