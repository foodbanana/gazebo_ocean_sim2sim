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
from launch.substitutions import LaunchConfiguration
import os

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

    return launch_processes


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
