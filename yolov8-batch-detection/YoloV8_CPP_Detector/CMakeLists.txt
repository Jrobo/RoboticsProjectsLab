cmake_minimum_required(VERSION 3.15)

# Explicitly set architecture to x86_64 (change to arm64 if you have Apple Silicon)
set(CMAKE_OSX_ARCHITECTURES arm64)


project(YoloV8_CPP_Detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV with required components
find_package(OpenCV REQUIRED core imgproc dnn highgui)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    include
)

add_executable(yolo_app
    main.cpp
    src/yolo_detector.cpp
)

target_link_libraries(yolo_app
    ${OpenCV_LIBS}
)
