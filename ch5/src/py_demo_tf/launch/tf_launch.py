import launch
import launch_ros

def generate_launch_description():
    action_node_static_tf = launch_ros.actions.Node(
        package = 'py_demo_tf',
        executable = 'static_tf_broadcaster',
        output = 'screen',
    )
    
    action_node_dynamic_tf = launch_ros.actions.Node(
        package = 'py_demo_tf',
        executable ='dynamic_tf_broadcaster',
        output ='screen',
    )
    
    action_node_tf_listener = launch_ros.actions.Node(
        package = 'py_demo_tf',
        executable ='tf_listener',
        output ='screen',
    )
    
    launch_description = launch.LaunchDescription([
        action_node_static_tf,
        action_node_dynamic_tf,
        action_node_tf_listener
    ])
    
    return launch_description
    
    