import yaml
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def get_static_tf(args):
    split_args = args.split(" ")
    parsed_args = []
    for arg in split_args:
        arg = arg.strip()
        if len(arg) == 0:
            continue
        parsed_args.append(arg)

    x = parsed_args[0]
    y = parsed_args[1]
    z = parsed_args[2]
    qx = parsed_args[3]
    qy = parsed_args[4]
    qz = parsed_args[5]
    qw = parsed_args[6]
    target_frame = parsed_args[7]
    source_frame = parsed_args[8]

    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[x, y, z, qx, qy, qz, qw, target_frame, source_frame]
    )

def generate_launch_description():
    bw_description_path = get_package_share_path("bw_description")
    joint_path = bw_description_path / "config/robeert_joints.yaml"
    model_path = bw_description_path / "urdf/robeert.urdf.xml"
    model_arg = DeclareLaunchArgument(
        name="model", default_value=str(model_path),
        description="Absolute path to robot urdf file"
    )
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    with open(joint_path) as file:
        joint_names = yaml.safe_load(file)

    return LaunchDescription([
        model_arg,
        get_static_tf(" 0.151   0.0     0.187   0  0  0  1  base_link camera_link"),
        get_static_tf(" 0.0775  0.0     0.230   0  0  1  0  base_link laser"),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[
                model_path,
                {"source_list": ["/bw/joint_states"]},
                {"rate": 10.0},
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),
        Node(
            package="bw_description",
            namespace="bw",
            executable="bw_description",
            name="bw_description",
            parameters=[{"joint_names": joint_names}]
        )
    ])
