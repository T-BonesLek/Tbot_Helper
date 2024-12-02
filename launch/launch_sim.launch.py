import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'tbot_helper'

    # Set the GAZEBO_MODEL_PATH environment variable
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory(package_name), 'models')
    )

    # Include the RSP launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    #include the joystick launch file
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )])
    )

     # Include the navigation launch file
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tbot_helper'), 'launch', 'navigation_launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the SLAM launch file
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]), launch_arguments={
            'params_file': './src/tbot_helper/config/mapper_params_online_async.yaml',
            'use_sim_time': 'true'
        }.items()
    )

    # Include the twist_mux node
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    # Load the joystick parameters
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    # Load the world in Gazebo
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'simple.world')

    # Load the Gazebo parameters
    gazebo_params_path = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path
        }.items()
    )

    # Spawn the entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tbot_helper'],
        output='screen'
    )


    # Controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')],
        output='screen'
    )

    # Spawner nodes for controllers with delay
    diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen'
        )]
    )

    joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad'],
            output='screen'
        )]
    )

    # Teleop node with delay
    teleop_node = TimerAction(
        period=10.0,
        actions=[Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/cmd_vel_joy')],
        )]
    )

    # Launch RViz as a node with delay
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'tbotSlam.rviz')
    rviz = TimerAction(
        period=15.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )]
    )

    # Open a new terminal for teleop twist with delay
    teleop_twist = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['terminator', '-e', 'bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_cont/cmd_vel_unstamped"'],
            output='screen'
        )]
    )

    # Launch them all!
    return LaunchDescription([
        gazebo_model_path,  # Include the environment variable
        rsp,
        joy,
        nav2_bringup,
        slam_toolbox,
        twist_mux,
        gazebo,
        spawn_entity,
        controller_manager,
        diff_drive_spawner,
        joint_broad_spawner,
        teleop_node,
        rviz,
        # teleop_twist,
    ])
