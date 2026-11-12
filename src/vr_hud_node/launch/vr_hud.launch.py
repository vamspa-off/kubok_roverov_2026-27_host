from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # Gamepad driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    # Buttons and axes handler
    joy_handler_node = Node(
        package='vr_hud_node',
        executable='joy_handler',
        name='joy_handler'
    )

    # Display node
    vr_hud_node = Node(
        package='vr_hud_node',
        executable='vr_hud_src',
        name='vr_hud_src',
        output='screen'
    )

    # Shutdown all in vr_hud_node exits
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=vr_hud_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='vr_hud_src exited'))
            ]
        )
    )

    return LaunchDescription([
        joy_node,
        joy_handler_node,
        vr_hud_node,
        shutdown_on_exit
    ])
