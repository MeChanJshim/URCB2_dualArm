# 올바른 import
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # 여기가 중요!
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    joint_limit_params_arg = DeclareLaunchArgument(
        'joint_limit_params',
        description='YAML file containing the joint limit values'
    )
    
    kinematics_params_arg = DeclareLaunchArgument(
        'kinematics_params',
        description="YAML file containing the robot's kinematic parameters. These will be different for each robot as they contain the robot's calibration."
    )
    
    physical_params_arg = DeclareLaunchArgument(
        'physical_params',
        description='YAML file containing the physical parameters of the robots'
    )
    
    visual_params_arg = DeclareLaunchArgument(
        'visual_params',
        description='YAML file containing the visual model of the robots'
    )
    
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
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model'
    )

    # Get launch configurations
    joint_limit_params = LaunchConfiguration('joint_limit_params')
    kinematics_params = LaunchConfiguration('kinematics_params')
    physical_params = LaunchConfiguration('physical_params')
    visual_params = LaunchConfiguration('visual_params')
    transmission_hw_interface = LaunchConfiguration('transmission_hw_interface')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    robot_model = LaunchConfiguration('robot_model')

    # Find the package path for the xacro file
    ur_xacro_file = PathJoinSubstitution([
        FindPackageShare('Y2UR_Visual'),
        'urdf',
        'ur.xacro'
    ])

    # Generate robot description using xacro command
    robot_description = ParameterValue(
        Command([
            'xacro ',
            ur_xacro_file,
            ' robot_model:=', robot_model,
            ' joint_limit_params:=', joint_limit_params,
            ' kinematics_params:=', kinematics_params,
            ' physical_params:=', physical_params,
            ' visual_params:=', visual_params,
            ' transmission_hw_interface:=', transmission_hw_interface,
            ' safety_limits:=', safety_limits,
            ' safety_pos_margin:=', safety_pos_margin,
            ' safety_k_position:=', safety_k_position
        ]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        output='screen'
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
        
        # Nodes
        robot_state_publisher,
    ])