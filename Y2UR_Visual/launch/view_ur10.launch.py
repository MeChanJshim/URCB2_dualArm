# 올바른 import
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # 여기가 중요!
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Include the UR10 launch file
    load_ur10_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('Y2UR_Visual'),
                'launch',
                'load_ur10.launch.py'
            ])
        )
    )
    
    # Joint state publisher GUI (commented out in original)
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )
    
    # Joint state publisher (commented out in original)
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen'
    )
    
    # RViz node with configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('Y2UR_Visual'),
            'cfg',
            'view_robot.rviz'
        ])],
        output='screen',
        # ROS2에서는 required 대신 on_exit를 사용하거나 생략
        # on_exit=launch.actions.Shutdown()  # 필요시 주석 해제
    )

    return LaunchDescription([
        # Include launch file
        load_ur10_launch,
        
        # Nodes
        robot_state_publisher,
        rviz_node,
        
        # 필요시 주석 해제
        # joint_state_publisher_gui,
        # joint_state_publisher,
    ])