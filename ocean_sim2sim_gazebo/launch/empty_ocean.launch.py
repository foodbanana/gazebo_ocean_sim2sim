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
import tempfile
import shutil
import re
import signal

from ament_index_python.packages import get_package_share_directory

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() == 'true'

    world_name = os.path.splitext(arg('world'))[0]
    tmp_dirs = []
    aux = flag('aux_views')
    if arg('wave_gain') or arg('wave_period') or aux:
        world_name = _world_copy(world_name, arg('wave_gain'), arg('wave_period'), aux, tmp_dirs)
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

    launch_processes.append(RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=stop_gazebo),
        OpaqueFunction(function=lambda _c: [shutil.rmtree(d, ignore_errors=True) for d in tmp_dirs])])))
    return launch_processes


AUX_VIEW_PANELS = ''.join(f'''
      <plugin filename="ImageDisplay" name="{title}">
        <gz-gui>
          <title>{title}</title>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <topic>/aux_view/{name}</topic>
        <topic_picker>false</topic_picker>
      </plugin>''' for title, name in (('Side view', 'side'), ('Top view', 'top')))


def _world_copy(world_name, gain, period, aux_views, tmp_dirs):
    """Copy of the world with other VRX wavefield gain / period (the vrx::PublisherPlugin message on
    /vrx/wavefield/parameters; the boat buoyancy and the wave visual both read it, docs/waves.md) and / or the GUI
    panels of the operator views (aux_views: ImageDisplay of /aux_view/side and /aux_view/top, cameras from
    aux_views.xacro, docs/teleop.md). Same file name (= world name, used in the topic names) in a temporary folder,
    removed on shutdown."""
    src = world_name + '.sdf'
    if not os.path.isabs(src):
        src = os.path.join(get_package_share_directory('ocean_sim2sim_gazebo'), 'worlds', src)
    with open(src) as f:
        sdf = f.read()
    for key, val in (('gain', gain), ('period', period)):
        if not val:
            continue
        float(val)                                        # error on a bad number
        pat = re.compile(r'(key:\s*"' + key + r'"\s*value\s*\{\s*type:\s*DOUBLE\s*double_value:\s*)[-0-9.eE+]+')
        sdf, n = pat.subn(r'\g<1>' + val, sdf)
        if n != 1:
            raise RuntimeError(f'{src}: wavefield parameter "{key}" found {n} times (expected 1)')
    if aux_views:
        if sdf.count('</gui>') != 1:
            raise RuntimeError(f'{src}: no single <gui> block for the aux view panels')
        sdf = sdf.replace('</gui>', AUX_VIEW_PANELS + '\n    </gui>')
    d = tempfile.mkdtemp(prefix='ocean_sim2sim_world_')
    tmp_dirs.append(d)
    with open(os.path.join(d, os.path.basename(src)), 'w') as f:
        f.write(sdf)
    return os.path.join(d, os.path.splitext(os.path.basename(src))[0])


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
        ('wave_gain', '', 'VRX wavefield gain (empty = as in the world file; empty_ocean 0.3, calm 0). docs/waves.md'),
        ('wave_period', '', 'VRX wavefield peak period [s] (empty = as in the world file; empty_ocean 5)'),
        ('aux_views', 'False', 'True: GUI panels Side view / Top view (cameras: urdf xacro arg aux_views:=true)'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc)
         for n, d, desc in args]
        + [OpaqueFunction(function=launch)])
