from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        return  LaunchDescription([
                Node(
                    package='amr_sweeper_fsm',
                    executable='amr_service_caller',
                    name='amr_state_machine',
                    output='screen',
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
        ])

