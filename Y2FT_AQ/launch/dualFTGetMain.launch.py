# launch/dual_ftget.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rft = Node(
        package='Y2FT_AQ',
        executable='RFTGetMain',
        name='rft_get_main',
        output='screen',
        emulate_tty=True,
    )

    lft = Node(
        package='Y2FT_AQ',
        executable='LFTGetMain',
        name='lft_get_main',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([rft, lft])
