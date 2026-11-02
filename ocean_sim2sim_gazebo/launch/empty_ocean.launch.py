# Launch the empty_ocean world and spawn a single USV at the world origin.
#
# The boat model is a launch argument, so no YAML config is needed when
# switching models:
#
#   # stock WAM-V
#   ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py
#
#   # custom boat model (xacro or urdf)
#   ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py urdf:=/path/to/my_wamv.urdf.xacro
#
# Reuses the helpers in vrx_gz.launch, the same ones competition.launch.py uses.

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
import os
import signal

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() == 'true'

    world_name = os.path.splitext(arg('world'))[0]
    sim_mode = arg('sim_mode')
    robot_urdf = arg('urdf')
    # The boat is always spawned at the world origin.
    model = Model(arg('name'), arg('model'), [0, 0, 0, 0, 0, 0])
    if robot_urdf:
        model.set_urdf(robot_urdf)

    # Let Gazebo resolve package://<pkg>/... mesh URIs of any installed ROS package
    # (e.g. ur_description, realsense2_description used by the arm on the deck).
    launch_processes = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(prefix, 'share'))
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if prefix
    ]
    launch_processes.extend(vrx_gz.launch.simulation(
        world_name, flag('headless'), flag('paused'), arg('extra_gz_args')))

    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, [model]))

    if sim_mode in ('full', 'bridge') and flag('bridge_competition_topics'):
        launch_processes.extend(vrx_gz.launch.competition_bridges(
            world_name_base, flag('competition_mode')))

    launch_processes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_gazebo)])))
    return launch_processes


def stop_gazebo(_context):
    """SIGINT the Gazebo server(s) of this launch when the launch shuts down by itself.

    ros_gz_sim starts Gazebo as `sh -c "ruby gz sim ..."`. When the launch ends by itself (e.g. a node
    requested a shutdown), it signals only that wrapper, and the real `gz sim` server keeps running.
    Ctrl+C in a terminal does not have this problem (the whole process group gets the signal).
    The server is in our process group, so find it there.
    """
    group = os.getpgrp()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').strip()
            if cmd.startswith(b'gz sim') and os.getpgid(int(pid)) == group:
                os.kill(int(pid), signal.SIGINT)
        except (OSError, ProcessLookupError):
            pass


def generate_launch_description():
    args = [
        ('world', 'empty_ocean', 'Name of world'),
        ('urdf', '', 'Boat model (xacro or urdf). Empty = stock WAM-V.'),
        ('name', 'wamv', 'Name of robot to spawn'),
        ('model', 'wam-v', 'Model type'),
        ('sim_mode', 'full',
         'full: spawn robot and bridges, sim: spawn only, bridge: bridges only'),
        ('bridge_competition_topics', 'True', 'Bridge competition topics'),
        ('competition_mode', 'False', 'True to disable debug topics'),
        ('headless', 'False', 'True to run without GUI'),
        ('paused', 'False', 'True to start paused'),
        ('extra_gz_args', '', 'Additional arguments for gz sim'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc)
         for n, d, desc in args]
        + [OpaqueFunction(function=launch)])
