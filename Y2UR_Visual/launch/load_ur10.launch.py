# 올바른 import
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # 여기가 중요!
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # UR10 specific parameter files with default paths
    joint_limit_params_arg = DeclareLaunchArgument(
        'joint_limit_params',
        default_value=PathJoinSubstitution([ #config/ur10/joint_limits.yaml
            FindPackageShare('Y2UR_Visual'),
            'config',
            'ur10',
            'joint_limits.yaml'
        ]),
        description='YAML file containing the joint limit values'
    )
    
    kinematics_params_arg = DeclareLaunchArgument(
        'kinematics_params',
        default_value=PathJoinSubstitution([ #config/ur10/default_kinematics.yaml
            FindPackageShare('Y2UR_Visual'),
            'config',
            'ur10',
            'default_kinematics.yaml'
        ]),
        description="YAML file containing the robot's kinematic parameters"
    )
    
    physical_params_arg = DeclareLaunchArgument(
        'physical_params',
        default_value=PathJoinSubstitution([ #config/ur10/physical_parameters.yaml
            FindPackageShare('Y2UR_Visual'),
            'config',
            'ur10',
            'physical_parameters.yaml'
        ]),
        description='YAML file containing the physical parameters of the robots'
    )
    
    visual_params_arg = DeclareLaunchArgument(
        'visual_params',
        default_value=PathJoinSubstitution([ #config/ur10/visual_parameters.yaml
            FindPackageShare('Y2UR_Visual'),
            'config',
            'ur10',
            'visual_parameters.yaml'
        ]),
        description='YAML file containing the visual model of the robots'
    )
    
    # Common parameters
    transmission_hw_interface_arg = DeclareLaunchArgument(
        'transmission_hw_interface',
        default_value='hardware_interface/PositionJointInterface'
    )
    
    safety_limits_arg = DeclareLaunchArgument(
        'safety_limits',
        default_value='false',
        description='If True, enable the safety limits controller'
    )
    
    safety_pos_margin_arg = DeclareLaunchArgument(
        'safety_pos_margin',
        default_value='0.15',
        description='The lower/upper limits in the safety controller'
    )
    
    safety_k_position_arg = DeclareLaunchArgument(
        'safety_k_position',
        default_value='20',
        description='Used to set k position in the safety controller'
    )
    
    # Robot model is fixed to ur10
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='ur10'
    )

    # Include the common launch file and pass all arguments
    load_ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ #launch/load_ur.launch.py
                FindPackageShare('Y2UR_Visual'), 
                'launch',
                'load_ur.launch.py'
            ])
        ),
        launch_arguments={
            'joint_limit_params': LaunchConfiguration('joint_limit_params'),
            'kinematics_params': LaunchConfiguration('kinematics_params'),
            'physical_params': LaunchConfiguration('physical_params'),
            'visual_params': LaunchConfiguration('visual_params'),
            'transmission_hw_interface': LaunchConfiguration('transmission_hw_interface'),
            'safety_limits': LaunchConfiguration('safety_limits'),
            'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
            'safety_k_position': LaunchConfiguration('safety_k_position'),
            'robot_model': LaunchConfiguration('robot_model'),
        }.items()
    )

    return LaunchDescription([
        # Launch arguments
        joint_limit_params_arg,
        kinematics_params_arg,
        physical_params_arg,
        visual_params_arg,
        transmission_hw_interface_arg,
        safety_limits_arg,
        safety_pos_margin_arg,
        safety_k_position_arg,
        robot_model_arg,
        
        # Include launch file
        load_ur_launch,
    ])