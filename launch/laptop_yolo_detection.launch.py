from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
                description="Compressed camera topic received from the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "bbox_topic",
                default_value="/vision/buoy_bbox",
                description="BBox topic published back to the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="/home/auv/models/buoy.pt",
                description="Ultralytics YOLO .pt model path. Replace with the trained buoy model path.",
            ),
            DeclareLaunchArgument(
                "target_class_name",
                default_value="buoy",
                description="Target class name. Set empty string to accept every class when target_class_id < 0.",
            ),
            DeclareLaunchArgument(
                "target_class_id",
                default_value="-1",
                description="Target class id. Overrides target_class_name when >= 0.",
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("device", default_value="cuda:0"),
            DeclareLaunchArgument("imgsz", default_value="640"),
            Node(
                package="auv_buoy_vision_control",
                executable="yolo_buoy_detector",
                name="yolo_buoy_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "target_class_name": LaunchConfiguration("target_class_name"),
                        "target_class_id": ParameterValue(LaunchConfiguration("target_class_id"), value_type=int),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "device": LaunchConfiguration("device"),
                        "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                    }
                ],
            ),
        ]
    )
