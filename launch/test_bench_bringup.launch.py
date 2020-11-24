from launch import LaunchDescription
from launch_ros import actions

def generate_launch_description():
    return LaunchDescription(
        [
            actions.Node(
                package="test_bench",
                node_executable="dg_main_test_bench",
                output="screen",
                prefix=["xterm -e"],
            )
        ]
    )
