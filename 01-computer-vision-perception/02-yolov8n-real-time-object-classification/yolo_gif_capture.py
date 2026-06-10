import cv2
import time
from ultralytics import YOLO
import imageio

def main():
    model = YOLO("yolov8n.pt")

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Error: Could not open webcam.")
        return

    print("✅ Recording for 1 minute or press 'q' to stop early...")

    gif_frames = []
    fps = 10  # lower FPS for GIF size optimization
    max_duration = 60  # in seconds
    width, height = 480, 320  # resize to reduce gif size
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame.")
            break

        frame = cv2.resize(frame, (width, height))
        results = model.predict(source=frame, show=False, conf=0.5)
        annotated_frame = results[0].plot()

        gif_frames.append(cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB))  # convert for imageio

        cv2.imshow("YOLOv8n Real-Time Object Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if time.time() - start_time > max_duration:
            print("⏱️ Recording stopped after 1 minute.")
            break

        time.sleep(1 / fps)

    cap.release()
    cv2.destroyAllWindows()

    print("💾 Saving GIF... This may take a moment.")
    imageio.mimsave("yolo_detection.gif", gif_frames, fps=fps)
    print("✅ GIF saved as 'yolo_detection.gif'.")

if __name__ == "__main__":
    main()


