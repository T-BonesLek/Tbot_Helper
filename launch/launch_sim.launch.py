import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
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

    # Load the joystick parameters
    joy_params = os.path.join(get_package_share_directory('tbot_helper'),'config','joystick.yaml')

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

    diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
            )


    # Open a new terminal for RViz
    rviz = ExecuteProcess(
        cmd=['terminator', '-e', 'bash -c "source /opt/ros/humble/setup.bash && rviz2 -d .rviz2/config/devtbot.rviz"'],
        output='screen'
    )

    # Open a new terminal for teleop twist
    teleop_twist = ExecuteProcess(
        cmd=['terminator', '-e', 'bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped"'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        gazebo_model_path,  # Include the environment variable
        rsp,
        joy,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        teleop_node,
        rviz,
        teleop_twist,
    ])
