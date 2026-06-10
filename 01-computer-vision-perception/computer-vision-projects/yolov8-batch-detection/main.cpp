
#include "yolo_detector.hpp"
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: ./yolo_app <model.onnx> <image.jpg>" << std::endl;
        return -1;
    }

    std::string model_path = argv[1];
    std::string image_path = argv[2];

    YoloDetector detector(model_path);
    cv::Mat image = cv::imread(image_path);

    if (image.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return -1;
    }

    cv::Mat output_img = detector.detect(image);

    cv::imshow("Detection", output_img);
    cv::imwrite("output/output.jpg", output_img);
    cv::waitKey(0);

    return 0;
}

