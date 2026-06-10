from ultralytics import YOLO


class YoloDetector:
    def __init__(self, model_name="yolov8n.pt", confidence_threshold=0.4):
        self.model = YOLO(model_name)
        self.confidence_threshold = confidence_threshold

    def detect(self, image_bgr):
        results = self.model(image_bgr, verbose=False)
        detections = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                confidence = float(box.conf[0])

                if confidence < self.confidence_threshold:
                    continue

                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                detections.append({
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "bbox": [int(x1), int(y1), int(x2), int(y2)]
                })

        return detections
