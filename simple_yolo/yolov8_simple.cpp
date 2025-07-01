#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace dnn;
using namespace std;

const float CONFIDENCE_THRESHOLD = 0.4;
const float NMS_THRESHOLD = 0.5;
const int INPUT_WIDTH = 640;
const int INPUT_HEIGHT = 640;

vector<string> loadClassList(const string& filename) {
    vector<string> classList;
    ifstream file(filename);
    string line;
    while (getline(file, line)) classList.push_back(line);
    return classList;
}

int main() {
    // Load model
    Net net = readNetFromONNX("models/yolov8n.onnx");
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    // Load image
    Mat image = imread("input.jpg");
    if (image.empty()) {
        cerr << "Image not found!" << endl;
        return -1;
    }

    // Preprocessing
    Mat blob;
    blobFromImage(image, blob, 1.0/255.0, Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);
    net.setInput(blob);

    // Forward pass
    vector<Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    // Load class labels
    vector<string> classNames = loadClassList("coco.names");

    // Post-processing
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    float x_factor = image.cols / (float)INPUT_WIDTH;
    float y_factor = image.rows / (float)INPUT_HEIGHT;
    float* data = (float*)outputs[0].data;

    const int dimensions = 85; // YOLOv8: 4 bbox + 1 obj + 80 class = 85
    const int rows = 8400;     // for 640x640

    for (int i = 0; i < rows; i++) {
        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD) {
            float* class_scores = data + 5;
            Mat scores(1, classNames.size(), CV_32FC1, class_scores);
            Point classIdPoint;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);
            if (max_class_score > 0.25) {
                int centerX = (int)(data[0] * x_factor);
                int centerY = (int)(data[1] * y_factor);
                int width   = (int)(data[2] * x_factor);
                int height  = (int)(data[3] * y_factor);
                int left    = centerX - width / 2;
                int top     = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back(confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
        data += dimensions;
    }

    // Apply NMS
    vector<int> indices;
    NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);

    for (int idx : indices) {
        Rect box = boxes[idx];
        rectangle(image, box, Scalar(0, 255, 0), 2);
        string label = format("%.2f", confidences[idx]);
        label = classNames[classIds[idx]] + ": " + label;
        putText(image, label, Point(box.x, box.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    }

    imshow("YOLOv8 Detection", image);
    waitKey(0);

    return 0;
}
