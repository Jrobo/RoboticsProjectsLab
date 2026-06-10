import cv2


def draw_detections(image, detections):
    output = image.copy()

    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        label = f'{det["class_name"]}: {det["confidence"]:.2f}'

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

    return output
