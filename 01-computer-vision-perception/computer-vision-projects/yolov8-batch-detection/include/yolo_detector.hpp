#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class YoloDetector {
public:
    explicit YoloDetector(const std::string& model_path);
    cv::Mat detect(const cv::Mat& image);

private:
    cv::dnn::Net net;
    std::vector<std::string> class_names;
    float conf_threshold = 0.25;
    float nms_threshold = 0.4;

    void loadClassNames();
};

