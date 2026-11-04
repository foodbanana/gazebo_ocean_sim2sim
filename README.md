# gazebo_ocean_sim2sim

Gazebo (Harmonic) ocean environment for **sim2sim** evaluation of manipulation policies
trained in Isaac Sim: a UR5 arm with a RH-P12-RN gripper and a D435i wrist camera, mounted on a WAM-V boat that moves
with the waves, grasping a PX4-flown drone that hovers above it.

Built on top of [VRX](https://github.com/osrf/vrx) (wave field, buoyancy, hydrodynamics, wind).
VRX is used as a dependency and is not modified. The physical inputs (gripper force, friction, drone mass and thrust
model, PX4 parameters, camera intrinsics, ROS 2 topics) are the same values as in the Isaac Sim project; the remaining
differences in the results are recorded as the sim2sim gap ([below](#known-sim2sim-gaps)).

Detailed notes (design, measurements, decisions) are in [`docs/`](#documentation) (Korean).

## Requirements

| What | Version / notes |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (the ROS 2 Jazzy `ros_gz` vendor packages) |
| VRX | [osrf/vrx](https://github.com/osrf/vrx) branch `jazzy`, in the **same colcon workspace** |
| PX4 (default drone mode) | [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) **v1.16.0**, built with `make px4_sitl_default none`. Not needed with `drone_mode:=fixed` |
| Python venv | `~/vrx_ws/.venv` for `pymavlink` (PX4 drone) and `openpi-client` (policy client). The system Python is not touched |
| Policy server | an [openpi](https://github.com/Physical-Intelligence/openpi) policy server (`scripts/serve_policy.py`), **not part of this repository** (only for policy runs) |

ROS 2 / Gazebo packages (all declared in the `package.xml` files, so `rosdep install` below installs them):

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-control-msgs ros-jazzy-tf2-ros \
  ros-jazzy-ur-description ros-jazzy-realsense2-description \
  python3-numpy python3-yaml python3-scipy \
  ros-jazzy-rqt-image-view            # optional: view the wrist camera
```

`python3-scipy` is used by `grasp_demo.py` (not declared in a `package.xml`).

### PX4-Autopilot (once)

```bash
git clone --recursive -b v1.16.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
# build dependencies: see the PX4 docs "Ubuntu Development Environment" (Tools/setup/ubuntu.sh)
make px4_sitl_default none          # builds build/px4_sitl_default/bin/px4, starts nothing
```

Another location works: pass it at launch with `px4_dir:=/path/to/PX4-Autopilot` (default `~/PX4-Autopilot`).
The bridge node starts and stops PX4 itself (fresh rootfs per start under `~/.ros/ocean_sim2sim_drone`).

## Build

```bash
mkdir -p ~/vrx_ws/src && cd ~/vrx_ws/src
git clone -b jazzy https://github.com/osrf/vrx.git
git clone https://github.com/foodbanana/gazebo_ocean_sim2sim.git
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Python venv (ROS 2 docs "Using Python Packages with ROS 2"): sees the system / ROS packages (numpy, yaml, rclpy),
# adds only what is missing. COLCON_IGNORE keeps colcon from scanning it.
python3 -m venv --system-site-packages .venv && touch .venv/COLCON_IGNORE
# PX4 link (ocean_sim2sim_drone), same versions as the Isaac Sim project
.venv/bin/python -m pip install pymavlink==2.4.50 fastcrc==0.5.0
# openpi policy client (ocean_sim2sim_policy): websocket client only, no JAX / GPU. Versions as tested (docs/policy.md);
# numpy < 2 is satisfied by the system numpy 1.26 (not replaced)
.venv/bin/python -m pip install websockets==17.3 msgpack==1.2.3 dm-tree==0.1.10 absl-py==2.5.1
.venv/bin/python -m pip install "openpi-client @ git+https://github.com/Physical-Intelligence/openpi.git#subdirectory=packages/openpi-client"
#   (or from a local openpi clone / your fork: .venv/bin/python -m pip install -e <openpi>/packages/openpi-client)

# build with the venv active, so the Python entry points use the venv interpreter
source .venv/bin/activate
colcon build --merge-install --symlink-install
source install/setup.bash
```

Every terminal that launches or runs something:

```bash
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash
```

Remove the venv by deleting `~/vrx_ws/.venv` (nothing else is installed). Leave it with `deactivate`.

## Run

```bash
# ocean + WAM-V + UR5 / RH-P12-RN / D435i + controllers + interface + PX4 drone (all defaults)
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py

ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm   # no waves (tests, grasp demo)
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py drone_mode:=fixed         # no PX4 / venv needed: drone held on a world anchor
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py wave_gain:=0.18 headless:=true
```

### Launch arguments (`ocean_sim2sim_bringup/launch/wamv_arm.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `world` | `empty_ocean` | Gazebo world. `empty_ocean` (VRX waves, gain 0.3, period 5 s), `empty_ocean_calm` (gain 0) |
| `headless` | `False` | `True`: no GUI |
| `paused` | `False` | `True`: start paused |
| `wave_gain` | (empty = world file value) | VRX wavefield gain ([`docs/waves.md`](docs/waves.md)) |
| `wave_period` | (empty = world file value) | VRX wavefield peak period [s] |
| `drone_mode` | `px4` | `px4`: PX4 SITL v1.16.0 flies the drone (venv + PX4-Autopilot needed). `fixed`: drone held still on a world-fixed anchor |
| `drone_motion` | `hover` | `px4` only. `static`: hold the hover point. `hover`: sine motion around it (Isaac Sim data collection setting). `rc`: a pilot flies it with the real transmitter after the air start ([`docs/teleop.md`](docs/teleop.md)) |
| `rc_port` | (empty = `rc_input.yaml` port, `/dev/ttyACM0`) | `drone_motion:=rc`: serial device of the Pico that reads the transmitter receiver |
| `hover_preset` | `mid` | `px4` + `hover` only: sine amplitude `small` / `mid` / `big` (Isaac Sim `drone_iris.yaml`) |
| `prop_spin` | `true` | `px4` only: props turn for show (Isaac Sim rule). `false`: props held still |
| `aux_views` | (empty = on with the GUI) | operator views `Side view` / `Top view` docked in the GUI (cameras on the robot base, Isaac Sim aux viewports; not recorded). `false` = off ([`docs/teleop.md`](docs/teleop.md)) |
| `aux_views_rate` | `15` | aux view camera rate [Hz] |
| `px4_dir` | `~/PX4-Autopilot` | PX4-Autopilot v1.16.0 directory (built with `make px4_sitl_default none`) |
| `xacro_file` | `ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro` | boat + arm description |
| `xacro_args` | (empty) | extra xacro args (table below), e.g. `xacro_args:="collision_meshes:=convex"` |
| `interface_config` | `ocean_sim2sim_interface/config/ur5_rh_p12_interface.yaml` | interface / scene node parameters (joints, gripper, protective stop, drone targets, reset) |
| `bridge_config` | `ur5_rh_p12_description/config/gz_bridge.yaml` | `ros_gz_bridge` config for the arm sensors (wrist camera) |

Useful `xacro_args` (boat: `wamv_with_arm.urdf.xacro`, arm: `ur5_rh_p12_d435i.urdf.xacro`; xacro args are global):

| xacro arg | Default | Meaning |
|---|---|---|
| `collision_meshes` | `original` | `original` meshes, or `convex` (the hulls Isaac Sim / PhysX collides with, [`docs/grasp.md`](docs/grasp.md) 1-1) |
| `self_collide` | `true` | collisions between links of the model (arm <-> boat, arm <-> itself) |
| `gripper_effort_limit` | `2.28` (boat xacro) | gripper drive joint torque [Nm] = Isaac Sim maxForce |
| `gripper_friction` | `0.6` | finger friction coefficient |
| `kinematics_params` | `ur_description` UR5 default | UR calibration file of your robot |
| `mount_variant` | `t10` | wrist camera mount (`t10` / `t7` / `t0` = camera tilt 10 / 7 / 0 deg) |
| `gripper_yaw` | `1.5708` | gripper rotation about the tool axis |
| `wrist_camera_update_rate` | `30` | wrist camera rate [Hz] |
| `arm_xacro` / `arm_root_link` | UR5 + RH-P12-RN + D435i / `robot_mount` | arm to mount and its root link ([below](#boat-with-an-arm)) |
| `arm_mount_xyz` / `arm_mount_rpy` | `0 0 1.30` / `0 0 0` | arm pose on the deck (deck top centre, facing the bow) |

### What happens at start-up

1. Gazebo starts the world, spawns the WAM-V with the arm, the controllers come up
   (`joint_state_broadcaster`, `arm_state_broadcaster`, `arm_controller`, `gripper_controller`).
2. The scene node lets the boat settle (3 s), averages the robot base pose (5 s) and spawns the drone above the arm.
3. `drone_mode:=px4`: PX4 is started, armed in Offboard and the drone is released in the air once its thrust carries its
   weight ("air start", no floor at sea). The first start takes about **20 - 30 s sim**; the log then shows
   `PX4 drone flying: ...`. `drone_mode:=fixed`: the drone hangs on its anchor after about 8 s sim.

Until then `/sim/reset` and `/sim/drone_kill` answer `success=False` (`scene not ready yet`).
The arm already accepts commands. Details: [`docs/scene.md`](docs/scene.md), [`docs/drone_px4.md`](docs/drone_px4.md).

### Topics and services

Same names and types as the real robot and the Isaac Sim setup ([`docs/interface.md`](docs/interface.md)).
All stamps are sim time (`/clock`); use `ros2 topic hz --use-sim-time ...`.

| Name | Type | Direction | Meaning |
|---|---|---|---|
| `/joint_command` | `sensor_msgs/JointState` | -> sim | arm 6 joint position targets [rad] (e.g. 60 Hz), 33 ms linear interpolation |
| `/gripper/command` | `std_msgs/Float64` | -> sim | gripper raw 0 (open) ~ 1150 (closed) |
| `/joint_states` | `sensor_msgs/JointState` | sim -> | arm 6 joints, 120 Hz (`/joint_state_broadcaster/joint_states`: all 10 joints, 240 Hz) |
| `/gripper/joint_states` | `sensor_msgs/JointState` | sim -> | `rh_p12_rn` present raw (0 ~ 1150), ~30 Hz (`/gripper/target`: last goal) |
| `/cam/wrist/color/image_raw`, `/cam/wrist/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | sim -> | D435i color 640 x 480 rgb8, ~30 Hz, real-camera intrinsics; subscribe RELIABLE ([`docs/wrist_camera.md`](docs/wrist_camera.md)) |
| `/protective_stop` | `std_msgs/Bool` | sim -> | true while the protective stop is active (contact force / joint jump; [`docs/interface.md`](docs/interface.md) 8) |
| `/base/imu` | `sensor_msgs/Imu` | sim -> | boat IMU (stock VRX WAM-V IMU, 100 Hz, with noise) |
| `/base/odom` | `nav_msgs/Odometry` | sim -> | robot base (`robot_mount`) pose relative to the episode start, 30 Hz |
| `/sim/reset` | `std_srvs/Trigger` | service | new episode: drone off, arm to the start pose, gripper open, drone at the next (seeded) target point; JSON answer |
| `/sim/drone_kill` | `std_srvs/Trigger` | service | drone motors off (after a grasp) |

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    position: [0.3, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]}"
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 1150.0}"   # close (0 = open)
ros2 service call /sim/reset std_srvs/srv/Trigger          # PX4: ~23 s sim (new air start), fixed: ~2.7 s
ros2 service call /sim/drone_kill std_srvs/srv/Trigger
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw
```

### Drone CLI (`drone_mode:=px4`)

`drone_cmd.py` sends one command to the PX4 commander inside the bridge node (UDP, no ROS / pymavlink needed):

```bash
ros2 run ocean_sim2sim_drone drone_cmd.py status
ros2 run ocean_sim2sim_drone drone_cmd.py goto 0.6 0.1 1.5 [--yaw-deg 0]   # world m; in hover mode moves the sine centre
ros2 run ocean_sim2sim_drone drone_cmd.py hold | land | kill
```

Move commands are refused until the scene has started the drone (`status` shows `ready: true`); `kill` is always accepted.
ROS side: services `/drone/px4/{restart,arm,kill,status}`, state `/drone/px4/state` ([`docs/drone_px4.md`](docs/drone_px4.md)).

### Stopping

Ctrl+C in the launch terminal stops Gazebo, the bridges and PX4. If a run was killed hard, a Gazebo server, bridges or
PX4 can be left behind; a second server makes everything slow and the controllers do not come up. Check and clean up:

```bash
pgrep -af 'gz sim|parameter_bridge|px4_bridge_node|px4_sitl_default/bin/px4'
pkill -INT -f 'gz sim'; pkill -f 'ros_gz_bridge/parameter_bridge'; pkill -f 'px4_bridge_node'; pkill -f 'px4_sitl_default/bin/px4'
```

### Gazebo-only launch (`ocean_sim2sim_gazebo`)

```bash
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py                       # empty ocean + stock WAM-V at the origin
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py urdf:=/path/to/boat.urdf.xacro   # custom boat model
ros2 launch ocean_sim2sim_gazebo empty_ocean.launch.py world:=nbpark        # a VRX world (downloads ~200 MB from Gazebo Fuel once)
```

Arguments: `world`, `urdf`, `name` (`wamv`), `model` (`wam-v`), `sim_mode` (`full` | `sim` | `bridge`),
`bridge_competition_topics` (`True`), `competition_mode` (`False`), `headless`, `paused`, `extra_gz_args`,
`wave_gain`, `wave_period`. No controllers, interface or drone.

`empty_ocean` uses DART with the **Bullet** collision detector: with self-collision on, ODE ran at RTF ~0.4, Bullet at
1.0 (detector comparison: [`docs/drone_px4.md`](docs/drone_px4.md) 8). VRX worlds such as `nbpark` keep ODE, so expect a
slower simulation there. Waves and wind are set in `ocean_sim2sim_gazebo/worlds/empty_ocean.sdf`
(`vrx::PublisherPlugin` `gain` / `period` / `direction` / `steepness`, and `vrx::USVWind`), or with `wave_gain` / `wave_period`.

## Tools

Run with the main launch up (except `regression.py`, which starts its own). Reports go to `~/ocean_sim2sim_reports/` (`--report-dir`).

| Command | What it does | Doc |
|---|---|---|
| `ros2 run ocean_sim2sim_bringup drive_test.py [--tests hold interp a c step gripper]` | arm / gripper response (overshoot, settling, latency) through `/joint_command` | [`docs/gain_tuning.md`](docs/gain_tuning.md) |
| `ros2 run ocean_sim2sim_bringup grasp_test.py --tests E G3 G1 G4 H` | gripper force / friction tests (calm world): empty close, pull-out force, hold | [`docs/grasp.md`](docs/grasp.md) |
| `ros2 run ocean_sim2sim_bringup grasp_demo.py --case all` | scripted drone grasp (success, offset_y, offset_x, empty, miss) + judge, calm world | [`docs/grasp_demo.md`](docs/grasp_demo.md) |
| `ros2 run ocean_sim2sim_bringup regression.py [--quick] [--only grip,pstop,demo,waves]` | one-command regression suite (own headless bringups, ~10 min, `--quick` ~5 min); exit code 0 = all PASS | [`docs/regression.md`](docs/regression.md) |
| `ros2 run ocean_sim2sim_policy policy_client_node.py --reset --episodes 5 --duration 30` | policy client only (same CLI as the Isaac Sim client, no judge) | [`docs/policy.md`](docs/policy.md) |
| `ros2 run ocean_sim2sim_policy fake_policy_server.py [--delay 0.18]` | fake openpi server (constant action chunks) to test the plumbing without a model | [`docs/policy.md`](docs/policy.md) |
| `ros2 run ocean_sim2sim_drone drone_link_test.py --test hover` | drone plugin check without PX4 (`drone_mode:=fixed`) | [`docs/drone_px4.md`](docs/drone_px4.md) |
| `ros2 run ocean_sim2sim_drone rc_watch.py [--port DEV]` | transmitter serial values and mapped sticks (no sim) | [`docs/teleop.md`](docs/teleop.md) |

### Policy inference

The policy server runs outside this repository, e.g. in an openpi checkout with its own `uv` environment
(`XLA_PYTHON_CLIENT_PREALLOCATE=false` keeps JAX from taking 75 % of the GPU up front):

```bash
# terminal 1: Gazebo stack
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py
# terminal 2: policy server (in the openpi checkout), wait for "server listening on 0.0.0.0:8000"
XLA_PYTHON_CLIENT_PREALLOCATE=false uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=<train config> --policy.dir=<checkpoint dir>
# terminal 3 (venv active): policy client; --host <ip> if the server is on another PC. Success is judged by eye in the GUI
ros2 run ocean_sim2sim_policy policy_client_node.py --reset --episodes 5 --duration 30
# terminal 4: operator stops the drone motors once it is grasped (or use --auto-kill)
ros2 service call /sim/drone_kill std_srvs/srv/Trigger
```

The client sends the wrist image only (no third-view camera), state = arm 6 joints + gripper, and executes 10-step
action chunks at 60 Hz with the Isaac Sim safety limits. Configuration: `ocean_sim2sim_policy/config/policy_client.yaml`.

### Teleoperation and recording

Same operation as the Isaac Sim data collection ([`docs/teleop.md`](docs/teleop.md)): the arm follows a PICO 4 Ultra
controller (XRoboToolkit; its Python binding lives in its own venv, `~/xrobotoolkit/venv` or `XR_PYTHON`), a pilot can fly
the PX4 drone with a Radiolink transmitter (receiver -> Raspberry Pi Pico -> USB serial), and episodes are recorded as
rosbag2 bags with the boat IMU and base odometry.

```bash
# terminal 1 (venv active): sim; drone_motion:=rc hands the drone to the transmitter after the air start
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py drone_motion:=rc rc_port:=/dev/ttyACM0
# terminal 2: arm teleop (XRoboToolkit PC service running, headset app WORKING, both controllers sent)
ros2 run ocean_sim2sim_teleop xr_teleop_node.py
# terminal 3: recorder (keys r / k / d / q; controller A record, B reset, X drone motors off). Bags: ocean_gazebo_bags/ in this repo (git-ignored)
ros2 run ocean_sim2sim_teleop record_toggle.py <task name>
# afterwards: boat IMU / base motion of a bag
ros2 run ocean_sim2sim_teleop plot_bag_imu.py ocean_gazebo_bags/<bag> --csv
```

Without the devices: `ros2 run ocean_sim2sim_teleop check_xr_teleop.py` (scripted controller against the running sim) and
`ros2 run ocean_sim2sim_drone check_rc.py --link <path>` (fake Pico on a pty, launch with `rc_port:=<path>`).

## Packages

| Package | Contents |
|---|---|
| `ocean_sim2sim_bringup` | `wamv_arm.launch.py` (ocean + WAM-V with the arm + controllers + interface + scene + PX4 bridge), test tools `drive_test.py`, `grasp_test.py`, `grasp_demo.py`, `regression.py` and their configs |
| `ocean_sim2sim_gazebo` | worlds `empty_ocean` / `empty_ocean_calm` (VRX Coast Waves only, no terrain, DART + Bullet), `empty_ocean.launch.py`, the drone model `iris_075` (Isaac Sim drone: Iris scaled 0.75, 0.8 kg) and its build / export scripts |
| `ocean_sim2sim_description` | WAM-V (VRX hull, hydrodynamics, 2 aft thrusters, IMU) with a pluggable robot arm on the deck |
| `ur5_rh_p12_description` | UR5 (CB3) + ROBOTIS RH-P12-RN(A) gripper + RealSense D435i wrist camera as one URDF, its `ros2_control` / Gazebo xacros, controller and bridge configs, Isaac Sim convex collision hulls |
| `ocean_sim2sim_interface` | `interface_node`: real-robot / Isaac Sim topics on top of ros2_control (`/joint_command`, `/gripper/*`, `/joint_states`, `/protective_stop`); `scene_node`: the drone episode (`/sim/reset`, `/sim/drone_kill`, `/base/odom`); grasp judge, kinematics, observation helpers |
| `ocean_sim2sim_drone` | PX4 SITL drone: `DroneLockstep` Gazebo system (C++), `px4_bridge_node.py` (MAVLink HIL lockstep, virtual sensors, thrust model, Offboard commander, ported from the Isaac Sim / Pegasus code), `drone_cmd.py` CLI |
| `ocean_sim2sim_gz_plugins` | Gazebo system `ContactMonitor` (per-link contact forces of the arm for the protective stop, [`docs/interface.md`](docs/interface.md) 8) |
| `ocean_sim2sim_policy` | openpi policy client (port of the Isaac Sim client: observation pairing, action chunks at 60 Hz, safety limits), fake server, offline test |
| `ocean_sim2sim_teleop` | PICO VR arm teleop node (port of the Isaac Sim `xr_teleop_node`: One Euro filter, differential IK from `/robot_description`, controller buttons), episode recorder `record_toggle.py`, `plot_bag_imu.py`, headset-less test |

### Boat with an arm

`ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro` = WAM-V + one fixed joint `wamv/base_link -> <arm root link>`.
Any arm works if its xacro is a complete `<robot>` file whose args all have defaults and whose root link is not welded
to the world. Its `ros2_control` and Gazebo sensor xacros and the controller yaml are args too
(`arm_ros2_control_xacro`, `arm_controllers_yaml`, `arm_gazebo_xacro`; [`docs/ros2_control.md`](docs/ros2_control.md)).

```bash
xacro $(ros2 pkg prefix ocean_sim2sim_description)/share/ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro
xacro $(ros2 pkg prefix ur5_rh_p12_description)/share/ur5_rh_p12_description/urdf/ur5_rh_p12_d435i.urdf.xacro   # arm alone
```

Arm control: `gz_ros2_control` with `joint_trajectory_controller` (position) for the 6 arm joints and the gripper, two
`joint_state_broadcaster`s; physics at 240 Hz ([`docs/ros2_control.md`](docs/ros2_control.md),
[`docs/gain_tuning.md`](docs/gain_tuning.md)).

## Documentation

| Doc | Topic |
|---|---|
| [`docs/interface.md`](docs/interface.md) | ROS 2 topics shared with the real robot / Isaac Sim, interface node, protective stop |
| [`docs/ros2_control.md`](docs/ros2_control.md) | controllers, launch, swapping the arm |
| [`docs/gain_tuning.md`](docs/gain_tuning.md) | position gain choice and overshoot (study notes, `drive_test.py`) |
| [`docs/wrist_camera.md`](docs/wrist_camera.md) | D435i wrist camera: topics, intrinsics, frame timing |
| [`docs/grasp.md`](docs/grasp.md) | gripper force / friction / collision shapes vs Isaac Sim (`grasp_test.py`) |
| [`docs/scene.md`](docs/scene.md) | episode scene: drone model, drone positions, `/sim/reset`, `/sim/drone_kill` |
| [`docs/drone_px4.md`](docs/drone_px4.md) | PX4 SITL drone: structure, air start, grasp demo with PX4, boat IMU, collision detector comparison |
| [`docs/grasp_demo.md`](docs/grasp_demo.md) | scripted drone grasp demo and judge (`grasp_demo.py`), results |
| [`docs/waves.md`](docs/waves.md) | wave settings and measured base motion per wave gain, comparison with the Isaac Sim moving base |
| [`docs/policy.md`](docs/policy.md) | openpi policy client (`policy_client_node.py`) |
| [`docs/regression.md`](docs/regression.md) | one-command regression suite (`regression.py`), checks and expected values |
| [`docs/teleop.md`](docs/teleop.md) | PICO VR arm teleop, transmitter drone teleop, episode recording, IMU plots |

## Known sim2sim gaps

Same inputs as Isaac Sim; these result differences are kept and recorded, not tuned away:

- **Pull-out (slip) force** of the gripper (G3, 0.8 kg box): Gazebo **27 - 32 N** vs Isaac Sim **19.1 N** (+40 - 70 %);
  effective friction 0.52 - 0.61 vs 0.386 (both set to 0.6). Corrected on 2026-11-04; the older "9.4 N" was a
  measurement error ([`docs/grasp.md`](docs/grasp.md) 3).
- **No finger-finger contact** with the original meshes: an empty close stops at 1.134 rad (Isaac Sim 1.121 rad,
  fingers touch) ([`docs/grasp.md`](docs/grasp.md) 3, 5).
- **Drone tilt after motor stop** (Bullet contact between finger meshes and the convex drone hulls): 1 - 10 deg vs
  Isaac Sim 0.5 - 1.7 deg, so `offset_x` sometimes ends as slip; the `miss` case is grasped (Isaac Sim: drone pushed out);
  contact force 33 - 48 N vs 27 N ([`docs/drone_px4.md`](docs/drone_px4.md) 4, 7, 8).
- **RTF while the drone is held**: 0.24 - 0.47 (Isaac Sim 0.69); 1.00 otherwise. Lockstep, so only wall time is affected
  ([`docs/drone_px4.md`](docs/drone_px4.md) 7, 8). Bullet is kept by decision (FCL / DART detectors were worse elsewhere).
- **Boat drift**: the boat drifts with the waves, so the distance to the world-fixed drone changes during an episode.
  Kept on purpose ([`docs/waves.md`](docs/waves.md) 5).
- **Boat IMU**: stock VRX IMU (100 Hz, noise, other position) vs Isaac Sim `/base/imu` (120 Hz, no noise)
  ([`docs/drone_px4.md`](docs/drone_px4.md) 5).

## License

This repository: **Apache-2.0** (see [`LICENSE`](LICENSE)). Third-party parts keep their own licenses:

| Part | License | Notes |
|---|---|---|
| Code in `ocean_sim2sim_drone` ported (via the Isaac Sim project) from [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator): virtual sensors, geomagnetic model, PX4 MAVLink link, commander, thrust model (see each file header) | BSD-3-Clause, Copyright (c) 2023 Marcelo Fialho Jacinto | `ocean_sim2sim_drone/ocean_sim2sim_drone/LICENSE.PegasusSimulator` |
| Drone meshes from Isaac Sim (`ocean_sim2sim_gazebo/models/iris_075/meshes/visual_isaac/`, `collision_isaac/`): Pegasus Simulator Iris | BSD-3-Clause | see `models/iris_075/README.md` and `LICENSE.PegasusSimulator` |
| PX4 Iris meshes (`ocean_sim2sim_gazebo/models/iris_075/meshes/iris.stl`, `iris_prop_*.dae`): PX4 Gazebo-classic Iris (Lorenz Meier, Thomas Gubler) | BSD (PX4 BSD-3-Clause) | `ocean_sim2sim_gazebo/models/iris_075/LICENSE_PX4`, `README.md` |
| UR5 convex collision hulls (`ur5_rh_p12_description/meshes/collision_convex/ur5/`), made from `ur_description` meshes | BSD-3-Clause | `ur5_rh_p12_description/meshes/collision_convex/ur5/LICENSE_ur_description` |
| Gripper xacro and meshes (`ur5_rh_p12_description/urdf/rh_p12_rn_a.urdf.xacro`, `ur5_rh_p12_description/meshes/rh_p12_rn_a/`, and their convex hulls), from [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) | Apache-2.0 | `ur5_rh_p12_description/meshes/rh_p12_rn_a/LICENSE_open_manipulator` |

The UR5 visual meshes, the D435i model and the WAM-V are not copied: they come from the installed `ur_description`,
`realsense2_description` and VRX packages.
