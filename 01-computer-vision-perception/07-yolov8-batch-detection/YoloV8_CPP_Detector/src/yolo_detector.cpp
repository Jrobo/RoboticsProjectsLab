#include "yolo_detector.hpp"

YoloDetector::YoloDetector(const std::string& model_path) {
    loadClassNames();
    net = cv::dnn::readNetFromONNX(model_path);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

void YoloDetector::loadClassNames() {
    class_names = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
        "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
        "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
        "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
        "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
        "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };
}

cv::Mat YoloDetector::detect(const cv::Mat& image) {
    cv::Mat blob;
    int width = 640, height = 640;

    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(width, height), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (const auto& output : outputs) {
        const float* data = (float*)output.data;
        for (int i = 0; i < output.rows; ++i, data += output.cols) {
            float confidence = data[4];
            if (confidence >= conf_threshold) {
                const float* scores = data + 5;
                cv::Mat scores_mat(1, class_names.size(), CV_32FC1, (void*)scores);
                cv::Point class_id_point;
                double max_score;
                cv::minMaxLoc(scores_mat, 0, &max_score, 0, &class_id_point);

                if (max_score > conf_threshold) {
                    int cx = (int)(data[0] * image.cols);
                    int cy = (int)(data[1] * image.rows);
                    int w = (int)(data[2] * image.cols);
                    int h = (int)(data[3] * image.rows);
                    int x = cx - w / 2;
                    int y = cy - h / 2;

                    class_ids.push_back(class_id_point.x);
                    confidences.push_back((float)max_score);
                    boxes.emplace_back(x, y, w, h);
                }
            }
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    cv::Mat result = image.clone();
    for (int idx : indices) {
        cv::Rect box = boxes[idx];
        cv::rectangle(result, box, cv::Scalar(0, 255, 0), 2);
        std::string label = cv::format("%s: %.2f", class_names[class_ids[idx]].c_str(), confidences[idx]);
        cv::putText(result, label, cv::Point(box.x, box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }

    return result;
}

