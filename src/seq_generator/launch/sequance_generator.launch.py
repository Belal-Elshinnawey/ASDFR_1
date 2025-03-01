from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, TimerAction

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
        # -------- Launched actions --------

    # Launch argument for Twist command mode
    use_twist_cmd_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_twist_cmd",
        default_value="true",
        description="Whether to use Twist command mode. If set to false, uses individual motor command mode.",
    )
    relbot_simulator_node = Node(
        package='relbot_simulator',
        executable='relbot_simulator',
        name='relbot_simulator',
        parameters=[
            {"use_twist_cmd": LaunchConfiguration(use_twist_cmd_arg.name)},
        ],
    )

    sequence_config_file_path = PathJoinSubstitution([
        get_package_share_directory('sequence_generator'),
        'config',
        'sequence_generator.yml'
    ])
    sequence_generator_node = Node(
        package='sequence_generator',
        executable='sequence_generator_node',
        name='sequence_generator_node',
        parameters=[sequence_config_file_path],
    )

    color_segmenter_config_file_path = PathJoinSubstitution([
        get_package_share_directory('brightness_calculator'),
        'config',
        'colour_segmenter.yml'
    ])
    colour_segmenter_node = Node(
        package='brightness_calculator',
        executable='colour_segmenter_node',
        name='colour_segmenter_node',
        parameters=[color_segmenter_config_file_path],
    )

    object_identifier_config_file_path = PathJoinSubstitution([
        get_package_share_directory('object_identifier'),
        'config',
        'object_identifier.yml'
    ])
    object_identifier_node = Node(
        package='sequence_generator',
        executable='sequence_generator_node',
        name='sequence_generator_node',
        parameters=[object_identifier_config_file_path],
    )

    cam2image_config_file_path = PathJoinSubstitution([
        get_package_share_directory('cam2image_vm2ros'),
        'config',
        'cam2image.yaml'
    ])
    cam2image = Node(
        package='cam2image_vm2ros',
        executable='cam2image',
        name='cam2image',
        parameters=[cam2image_config_file_path],
    )

    return LaunchDescription([
        use_twist_cmd_arg,
        relbot_simulator_node,
        sequence_generator_node,
        colour_segmenter_node,
        object_identifier_node,
        cam2image
    ])