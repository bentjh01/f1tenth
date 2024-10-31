from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    f1tenth_control = Node(
        package="f1tenth_control",
        executable="twist2ackermann",
    )

    reactive_nav = Node(
        package="reactive_navigation",
        executable="disparity_extender",
    )

    ld.add_action(f1tenth_control)
    ld.add_action(reactive_nav)

    return ld
