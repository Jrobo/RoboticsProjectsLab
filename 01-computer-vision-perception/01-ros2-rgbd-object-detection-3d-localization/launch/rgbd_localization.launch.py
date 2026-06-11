from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="yolov8n.pt",
        description="YOLO model name or path"
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.35",
        description="YOLO confidence threshold"
    )

    object_filter_arg = DeclareLaunchArgument(
        "object_filter",
        default_value="",
        description="Object class filter, e.g., banana, scissors, cell phone"
    )

    rgbd_localization_node = Node(
        package="rgbd_object_perception",
        executable="rgbd_localization_node",
        name="rgbd_localization_node",
        output="screen",
        parameters=[
            {
                "model_name": LaunchConfiguration("model_name"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "object_filter": LaunchConfiguration("object_filter"),
            }
        ],
    )

    return LaunchDescription([
        model_name_arg,
        confidence_threshold_arg,
        object_filter_arg,
        rgbd_localization_node,
    ])
