# Ocean + WAM-V with an arm on the deck + ros2_control, in one command.
#
#   ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py
#   ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py xacro_args:="kinematics_params:=/path/to/calib.yaml"
#
# Steps:
#   1. xacro -> one URDF file (used for both Gazebo spawn and robot_state_publisher)
#   2. ocean_sim2sim_gazebo/empty_ocean.launch.py: world + spawn the boat at the origin + VRX bridges
#   3. robot_state_publisher (gz_ros2_control reads robot_description from it)
#   4. controller spawner: joint_state_broadcaster, arm_controller, gripper_controller

import os
import shlex
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # 1. one URDF for everything
    xacro_cmd = ['xacro', arg('xacro_file')] + shlex.split(arg('xacro_args'))
    urdf = subprocess.run(xacro_cmd, check=True, capture_output=True, text=True).stdout
    fd, urdf_path = tempfile.mkstemp(prefix='ocean_sim2sim_', suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(urdf)

    # 2. world + boat
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ocean_sim2sim_gazebo'), 'launch', 'empty_ocean.launch.py')),
        launch_arguments={
            'world': arg('world'),
            'urdf': urdf_path,
            'headless': arg('headless'),
            'paused': arg('paused'),
        }.items())

    # 3. robot description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}])

    # 4. controllers (spawner waits for the controller manager inside Gazebo)
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                   '--controller-manager-timeout', '120'])

    return [sim, rsp, spawner]


def generate_launch_description():
    default_xacro = os.path.join(
        get_package_share_directory('ocean_sim2sim_description'), 'urdf', 'wamv_with_arm.urdf.xacro')
    args = [
        ('world', 'empty_ocean', 'Gazebo world name'),
        ('xacro_file', default_xacro, 'Boat + arm description'),
        ('xacro_args', '', 'Extra xacro args, e.g. "arm_mount_rpy:=\'0 0 1.57\'"'),
        ('headless', 'False', 'True to run without GUI'),
        ('paused', 'False', 'True to start paused'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch)])
