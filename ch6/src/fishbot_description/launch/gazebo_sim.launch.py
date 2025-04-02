import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.parameter_descriptions

def generate_launch_description():
    robot_name_in_model = "fishbot"
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = os.path.join(urdf_tutorial_path, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_world_path = os.path.join(urdf_tutorial_path, 'world', 'custom_room.world')
    
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='urdf absolute path'
    )
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]
        ),
        value_type=str
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_name_in_model, ]
    )
    
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_joint_state_broadcaster'],
        output='screen'
    )
    
    return launch.LaunchDescription(
        [
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=spawn_entity_node,
                    on_exit=[load_joint_state_controller],
                )  
            ),
            action_declare_arg_mode_path,
            robot_state_publisher_node,
            launch_gazebo,
            spawn_entity_node
        ]
    )