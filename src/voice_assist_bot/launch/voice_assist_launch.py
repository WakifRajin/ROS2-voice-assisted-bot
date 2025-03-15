import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='voice_assist_bot',
            executable='speaker',
            name='speaker_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='voice_assist_bot',
            executable='control',
            name='control_node',
            output='screen'
        ),
    ])
