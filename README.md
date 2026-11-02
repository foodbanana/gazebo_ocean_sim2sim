# gazebo_ocean_sim2sim

Gazebo (Harmonic) ocean environment for **sim2sim** evaluation of manipulation policies
trained in Isaac Sim: a robot arm mounted on a WAM-V boat, moving with the waves.

Built on top of [VRX](https://github.com/osrf/vrx) (wave field, buoyancy, hydrodynamics, wind).
VRX is used as a dependency and is not modified.

## Packages

| Package | Contents |
|---|---|
| `ocean_sim2sim_gazebo` | `empty_ocean` world (VRX Coast Waves only, no terrain) and launch files |
| `ur5_rh_p12_description` | UR5 (CB3) + ROBOTIS RH-P12-RN(A) gripper + RealSense D435i wrist camera as one URDF |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- VRX built in the same workspace
- `sudo apt install ros-jazzy-xacro ros-jazzy-ur-description ros-jazzy-realsense2-description`

## Build

```bash
mkdir -p ~/vrx_ws/src && cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx.git
git clone <this repository> gazebo_ocean_sim2sim
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install
source install/setup.bash
```

## Run

```bash
# empty ocean + stock WAM-V at the world origin
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py

# custom boat model
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py urdf:=/path/to/boat.urdf.xacro

# same setup in the Nathan Benderson Park world (VRX, downloads ~200 MB terrain from Gazebo Fuel once)
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py world:=nbpark
```

Waves and wind are set in `ocean_sim2sim_gazebo/worlds/empty_ocean.sdf`
(`vrx::PublisherPlugin` `gain` / `period` / `direction` / `steepness`, and `vrx::USVWind`).

## Robot arm description

```bash
xacro $(ros2 pkg prefix ur5_rh_p12_description)/share/ur5_rh_p12_description/urdf/ur5_rh_p12_d435i.urdf.xacro
```

| Argument | Default | Meaning |
|---|---|---|
| `kinematics_params` | `ur_description` UR5 default | UR calibration file of your robot |
| `mount_variant` | `t10` | wrist camera mount (`t10` / `t7` / `t0` = camera tilt 10 / 7 / 0 deg) |
| `gripper_yaw` | `1.5708` | gripper rotation about the tool axis |

The root link `robot_mount` is not welded to the world, so the arm can be attached to any parent link.

## License

Apache-2.0 (see `LICENSE`).

- Gripper xacro and meshes (`ur5_rh_p12_description/urdf/rh_p12_rn_a.urdf.xacro`,
  `ur5_rh_p12_description/meshes/rh_p12_rn_a/`): Apache-2.0, from
  [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
  (see `LICENSE_open_manipulator`).
