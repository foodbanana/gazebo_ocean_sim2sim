# Ocean + WAM-V with an arm on the deck + ros2_control, in one command.
#
#   ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py
#   ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py xacro_args:="kinematics_params:=/path/to/calib.yaml"
#
# Steps:
#   1. xacro -> one URDF file (used for both Gazebo spawn and robot_state_publisher)
#   2. ocean_sim2sim_gazebo/empty_ocean.launch.py: world + spawn the boat at the origin + VRX bridges
#   3. robot_state_publisher (gz_ros2_control reads robot_description from it)
#   4. controller spawner: joint_state_broadcaster, arm_state_broadcaster, arm_controller, gripper_controller
#   5. ocean_sim2sim_interface: Isaac Sim / real-robot topics (/joint_command, /gripper/*, /protective_stop).
#   6. ros_gz_bridge for the arm sensors (wrist camera -> /cam/wrist/color/*).
#   7. scene_node (drone to grasp, /sim/drone_kill, /sim/reset) + its Gazebo bridges (also /sim/robot_contacts for the
#      protective stop).
#      It exits on an invalid command -> the whole launch shuts down (no silent fallback).

import os
import shlex
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    if arg('drone_motion') not in ('static', 'hover', 'rc'):
        raise RuntimeError(f"drone_motion must be static, hover or rc: {arg('drone_motion')}")
    if arg('drone_motion') == 'rc' and arg('drone_mode') != 'px4':
        raise RuntimeError('drone_motion:=rc needs drone_mode:=px4 (the transmitter flies the PX4 drone)')

    # operator views (Side / Top cameras + GUI panels, not recorded; Isaac Sim --aux-views: on with the GUI by default)
    headless = arg('headless').lower() in ('true', '1', 'yes')
    aux = arg('aux_views').lower()
    if aux not in ('', 'true', 'false'):
        raise RuntimeError(f"aux_views must be true, false or empty (= on with the GUI): {arg('aux_views')}")
    aux = (not headless) if aux == '' else aux == 'true'
    if aux and headless:
        raise RuntimeError('aux_views:=true needs the GUI (headless:=False)')

    # 1. one URDF for everything
    xacro_cmd = (['xacro', arg('xacro_file')] + shlex.split(arg('xacro_args'))
                 + (['aux_views:=true', f"aux_views_rate:={arg('aux_views_rate')}"] if aux else []))
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
            'wave_gain': arg('wave_gain'),
            'wave_period': arg('wave_period'),
            'aux_views': str(aux),
        }.items())

    # 3. robot description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}],
        remappings=[('joint_states', '/joint_state_broadcaster/joint_states')])

    # 4. controllers (spawner waits for the controller manager inside Gazebo)
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 'arm_state_broadcaster', 'arm_controller', 'gripper_controller',
                   '--controller-manager-timeout', '120',
                   # default 10 s: at a busy start-up the load call answered later, the spawner retried and failed
                   # with "already loaded" (controllers stayed inactive)
                   '--service-call-timeout', '60'])

    # 5. Isaac Sim / real-robot topics
    interface = Node(
        package='ocean_sim2sim_interface',
        executable='interface_node',
        output='screen',
        parameters=[arg('interface_config')])
    stop_on_interface_exit = RegisterEventHandler(OnProcessExit(
        target_action=interface,
        on_exit=[EmitEvent(event=Shutdown(reason='ocean_sim2sim_interface exited'))]))

    # remove the temporary URDF when the launch ends (Ctrl+C or Gazebo closed)
    def remove_urdf(_context):
        if os.path.exists(urdf_path):
            os.remove(urdf_path)
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=remove_urdf)]))

    # 6. arm sensors (cameras) Gazebo -> ROS
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_sensor_bridge',
        output='screen',
        parameters=[{'config_file': arg('bridge_config'), 'use_sim_time': True}])

    # 7. episode scene: drone + /sim/drone_kill, /sim/reset
    world = os.path.basename(os.path.splitext(arg('world'))[0])
    imu_gz = f'/world/{world}/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu'
    scene_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scene_bridge',
        output='screen',
        arguments=[f'/world/{world}/create@ros_gz_interfaces/srv/SpawnEntity',
                   f'/world/{world}/remove@ros_gz_interfaces/srv/DeleteEntity',
                   '/model/wamv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/drone/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                   '/drone/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                   '/drone/state@std_msgs/msg/String[gz.msgs.StringMsg',
                   # evaluation only (grasp judge), not policy inputs
                   '/drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/drone/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                   # per robot link contact forces (ContactMonitor plugin, JSON) -> protective stop of the interface
                   '/sim/robot_contacts@std_msgs/msg/String[gz.msgs.StringMsg',
                   # boat IMU (stock VRX WAM-V IMU: pose, 100 Hz, noise) -> /base/imu (Isaac Sim moving-base topic)
                   f'{imu_gz}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[('/model/wamv/odometry', '/sim/boat_odometry'), (imu_gz, '/base/imu')],
        parameters=[{'use_sim_time': True}])
    scene = Node(
        package='ocean_sim2sim_interface',
        executable='scene_node',
        output='screen',
        parameters=[arg('interface_config'), {'world': world, 'drone_mode': arg('drone_mode'),
                                              'drone_motion': arg('drone_motion'), 'hover_preset': arg('hover_preset')}])
    nodes = [cleanup, sim, rsp, spawner, interface, stop_on_interface_exit, sensor_bridge, scene_bridge, scene]
    if arg('drone_mode') == 'px4':
        # PX4 SITL bridge (ocean_sim2sim_drone, docs/drone_px4.md). Needs the Python venv with pymavlink: launch from a
        # terminal where ~/vrx_ws/.venv is activated (README). It starts PX4 itself (px4_dir).
        nodes.append(Node(
            package='ocean_sim2sim_drone',
            executable='px4_bridge_node.py',
            output='screen',
            parameters=[{'px4_dir': arg('px4_dir'), 'prop_spin': arg('prop_spin').lower() in ('true', '1', 'yes'),
                         # drone_motion rc: the real transmitter flies the drone after the air start (docs/teleop.md)
                         'rc': arg('drone_motion') == 'rc', 'rc_port': arg('rc_port')}]))
    return nodes


def generate_launch_description():
    default_xacro = os.path.join(
        get_package_share_directory('ocean_sim2sim_description'), 'urdf', 'wamv_with_arm.urdf.xacro')
    default_iface = os.path.join(
        get_package_share_directory('ocean_sim2sim_interface'), 'config', 'ur5_rh_p12_interface.yaml')
    args = [
        ('world', 'empty_ocean', 'Gazebo world name'),
        ('interface_config', default_iface, 'ocean_sim2sim_interface parameters (arm joints, gripper)'),
        ('bridge_config',
         os.path.join(get_package_share_directory('ur5_rh_p12_description'), 'config', 'gz_bridge.yaml'),
         'ros_gz_bridge config for the arm sensors'),
        ('xacro_file', default_xacro, 'Boat + arm description'),
        ('xacro_args', '', 'Extra xacro args, e.g. "arm_mount_rpy:=\'0 0 1.57\'"'),
        ('headless', 'False', 'True to run without GUI'),
        ('paused', 'False', 'True to start paused'),
        ('wave_gain', '', 'VRX wavefield gain (empty = world file value). docs/waves.md'),
        ('wave_period', '', 'VRX wavefield peak period [s] (empty = world file value)'),
        ('drone_mode', 'px4', 'px4 = PX4 SITL flies the drone (venv + PX4-Autopilot needed); fixed = held on a world anchor'),
        ('drone_motion', 'hover', 'px4: static, hover (sine around the hover point, Isaac Sim collection setting) or rc '
                                  '(real transmitter via the Pico serial device after the air start, docs/teleop.md)'),
        ('rc_port', '', 'drone_motion rc: serial device of the Pico (empty = ocean_sim2sim_drone config/rc_input.yaml port)'),
        ('hover_preset', 'mid', 'px4 hover sine preset: small | mid | big (Isaac Sim drone_iris.yaml)'),
        ('prop_spin', 'true', 'px4: props turn for show (Isaac Sim rule: 100 rad/s at >= 0.1 N thrust, 5 rad/s below, '
                              '0 off; Isaac Sim default on with GUI, off headless). false = props held still'),
        ('aux_views', '', 'operator views Side / Top in the GUI (empty = on with the GUI, off headless; not recorded). '
                          'docs/teleop.md'),
        ('aux_views_rate', '15', 'aux view camera rate [Hz]'),
        ('px4_dir', '~/PX4-Autopilot', 'PX4-Autopilot v1.16.0 directory (built: make px4_sitl_default none)'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch)])
