import cv2


def draw_detections(image, detections):
    output = image.copy()

    for det in detections:
        x1, y1, x2, y2 = det["bbox"]

        label = f'{det["class_name"]}: {det["confidence"]:.2f}'

        if "position_3d" in det and det["position_3d"] is not None:
            x, y, z = det["position_3d"]
            label += f" | X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"

        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.putText(
            output,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

        # Draw bounding-box center
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        cv2.circle(output, (u, v), 4, (0, 0, 255), -1)

    return output
