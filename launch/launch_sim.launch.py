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
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Load the world in Gazebo
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'simple.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn the entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tbot_helper'],
        output='screen'
    )

    # Open a new terminal for RViz
    rviz = ExecuteProcess(
        cmd=['terminator', '-e', 'bash -c "source /opt/ros/humble/setup.bash && rviz2 -d .rviz2/config/devtbot.rviz"'],
        output='screen'
    )

    # Open a new terminal for teleop twist
    teleop_twist = ExecuteProcess(
        cmd=['terminator', '-e', 'bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        gazebo_model_path,  # Include the environment variable
        rsp,
        gazebo,
        spawn_entity,
        # rviz,
        teleop_twist,
    ])
