# Real-Time Object Classification using YOLOv8n and Webcam

This project performs **real-time object detection** from a webcam stream using **YOLOv8n (nano)** and automatically saves a short **animated GIF** showing the detection results. It's ideal for showcasing lightweight AI in action using Ultralytics YOLOv8, OpenCV, and Python.

---

## Demo

> ▶️ A webcam window opens (until you press `q`) and saves a `yolo_detection.gif`.

![Demo](https://github.com/Jrobo/YOLOv8n-real-time-object-classification/blob/main/yolo_detection.gif)

---

## 📁 Project Structure

```
your_project/
│
├── yolo_webcam_realtime.py     # Main detection script
├── yolov8n.pt                  # YOLOv8n model (auto-downloads if missing)
├── yolo_detection.gif          # Output animated GIF (auto-created)
├── requirements.txt            # Python dependencies
└── README.md                   # Project documentation
```

---

## Features

- Runs YOLOv8n in real-time on webcam feed
- Automatically saves an animated GIF of detected frames
- Optimized for speed (320x480, 10 fps)
- Detects over 80 object categories from the COCO dataset

---

## 🔧 Installation

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/yourusername/YOLOv8n-real-time-object-classification.git
cd YOLOv8n-real-time-object-classification
```

### 2️⃣ Create a Virtual Environment (Optional but Recommended)

```bash
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

### 3️⃣ Install Dependencies

```bash
pip install -r requirements.txt
```

**`requirements.txt`:**
```
ultralytics
opencv-python
imageio
```

---

## ▶️ Run the Script

From the terminal, run:

```bash
python yolo_webcam_realtime.py
```

This will:

- Launch webcam feed
- Detect objects in real-time
- Show bounding boxes on-screen
- Save a GIF file `yolo_detection.gif` after 60 seconds or on early exit (`q` key)

---

## ⚙️ Configuration

You can customize these values in the script:

```python
fps = 10                  # FPS of the saved GIF
max_duration = 60         # Duration in seconds to record
width, height = 480, 320  # Resize frames to optimize GIF size
```

Change the YOLO model (for better accuracy or speed):

```python
model = YOLO("yolov8n.pt")      # Default: Nano model
model = YOLO("yolov8s.pt")      # Small
model = YOLO("yolov8m.pt")      # Medium
```

---

## Portfolio Tips

- Show this as a **live computer vision + AI** project on your GitHub
- Highlight use of **YOLOv8**, **OpenCV**, and **GIF generation**
- Mention skills: real-time optimization, webcam I/O, model inference


---

## 📚 Resources

- [Ultralytics YOLOv8 Documentation](https://docs.ultralytics.com)
- [COCO Object Categories](https://github.com/amikelive/coco-labels)

---

## 🪪 License

This project is under the MIT License.

---

## Acknowledgements

Thanks to [Ultralytics](https://github.com/ultralytics) for making YOLOv8 open source.

---

## Contact

- GitHub: https://github.com/Jrobo?tab=repositories
- Email: kaziabduljamil2024@gmail.com
