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
| `ocean_sim2sim_description` | WAM-V (VRX hull, hydrodynamics, 2 aft thrusters, IMU) with a pluggable robot arm on the deck |
| `ocean_sim2sim_interface` | Isaac Sim / real-robot topics on top of ros2_control (`/joint_command`, `/gripper/command`, `/joint_states`, `/gripper/joint_states`) |
| `ocean_sim2sim_bringup` | One launch file: ocean + WAM-V with the arm + controllers + interface node |

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
# ocean + WAM-V + UR5 / RH-P12-RN / D435i + controllers (see docs/ros2_control.md)
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py

# empty ocean + stock WAM-V at the world origin
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py

# WAM-V with the UR5 + RH-P12-RN + D435i arm on the deck
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py \
  urdf:=$(ros2 pkg prefix ocean_sim2sim_description)/share/ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro

# custom boat model
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py urdf:=/path/to/boat.urdf.xacro

# same setup in the Nathan Benderson Park world (VRX, downloads ~200 MB terrain from Gazebo Fuel once)
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py world:=nbpark
```

`empty_ocean` uses DART with the **Bullet** collision detector. With self-collision on
(`wamv_with_arm.urdf.xacro` `self_collide:=true`, default) the default ODE detector ran at RTF ~0.4,
Bullet at 1.0. VRX worlds such as `nbpark` keep ODE, so expect a slower simulation there.

Waves and wind are set in `ocean_sim2sim_gazebo/worlds/empty_ocean.sdf`
(`vrx::PublisherPlugin` `gain` / `period` / `direction` / `steepness`, and `vrx::USVWind`).

## Boat with an arm (`ocean_sim2sim_description`)

`urdf/wamv_with_arm.urdf.xacro` = WAM-V + one fixed joint `wamv/base_link -> <arm root link>`.
Any arm works if its xacro is a complete `<robot>` file whose args all have defaults and whose root link
is not welded to the world.

| Argument | Default | Meaning |
|---|---|---|
| `arm_xacro` | `ur5_rh_p12_description/urdf/ur5_rh_p12_d435i.urdf.xacro` | arm to mount |
| `arm_root_link` | `robot_mount` | root link of that arm |
| `arm_mount_xyz` / `arm_mount_rpy` | `0 0 1.30` / `0 0 0` | pose on the deck (deck top centre, facing the bow) |
| `self_collide` | `true` | collisions between links of the model (arm <-> boat, arm <-> itself) |

## Arm control (ros2_control)

`gz_ros2_control` with `joint_trajectory_controller` (position) for the 6 arm joints and the gripper,
plus two `joint_state_broadcaster`s (all joints at 240 Hz, arm-only `/joint_states` at 120 Hz). Physics runs at 240 Hz. Details and test results: [`docs/ros2_control.md`](docs/ros2_control.md).
Gain tuning and overshoot notes: [`docs/gain_tuning.md`](docs/gain_tuning.md)
(measure with `ros2 run ocean_sim2sim_bringup drive_test.py`).

Policies and teleop use the same topics as the real robot and the Isaac Sim setup
([`docs/interface.md`](docs/interface.md)):

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    position: [0.3, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]}"
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 1150.0}"   # raw 0 (open) ~ 1150 (closed)
```

## Wrist camera

D435i color, 640 x 480 rgb8, same intrinsics as the Isaac Sim / real camera:
`/cam/wrist/color/image_raw`, `/cam/wrist/color/camera_info` ([`docs/wrist_camera.md`](docs/wrist_camera.md)).

```bash
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw
```

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
