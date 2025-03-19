import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = os.path.join(urdf_tutorial_path, 'urdf', 'first_robot.urdf')
    
    declare_model_arg = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF Absolute path'
    )
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]
        ),
        value_type=str
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': robot_description}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
    )
    
    rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable = 'rviz2'
    )
    
    return launch.LaunchDescription([
        declare_model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node   
    ])