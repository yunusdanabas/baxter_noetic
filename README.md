[![CI](https://github.com/yunusdanabas/baxter_noetic/actions/workflows/ci.yml/badge.svg)](https://github.com/yunusdanabas/baxter_noetic/actions/workflows/ci.yml)

# Baxter Noetic

ROS Noetic / Python 3 checkout for the Baxter Research Robot SDK, simulator, and MoveIt configuration. This repository is intended for Ubuntu 20.04 + ROS Noetic, or for the included Docker image when Noetic is not installed natively.

No pre-built image is published from this checkout yet. Build the local image before using Docker or Compose.

## Packages

| Path | Purpose |
|------|---------|
| `baxter/` | SDK metapackage and legacy `baxter.sh` helper |
| `baxter_common/` | URDF, meshes, end-effector descriptions, custom messages, and services |
| `baxter_interface/` | Python SDK interfaces and action servers |
| `baxter_tools/` | Robot operation and maintenance scripts |
| `baxter_examples/` | SDK example scripts |
| `baxter_simulator/` | Gazebo simulator, controllers, simulated hardware, IO, and kinematics |
| `baxter_moveit_config/` | MoveIt configuration and launch files |

## Docker Quick Start

Use this path if the host is not Ubuntu 20.04 with ROS Noetic.

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:n07 .
docker run --rm -it baxter-noetic:n07
```

Inside the container:

```bash
source /root/baxter_ws/install/setup.bash
roslaunch --nodes baxter_gazebo baxter_world.launch
roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true
roslaunch --nodes baxter_moveit_config move_group.launch
```

The `--nodes` form parses launches and prints node names without starting the full GUI stack.

## Native Setup

Native setup requires Ubuntu 20.04 and ROS Noetic. If `/opt/ros/noetic` is missing or `ROS_DISTRO` is not `noetic`, use Docker instead.

Install ROS Noetic first, then install the workspace dependencies:

```bash
cd ~/baxter_noetic_ws
source /opt/ros/noetic/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
```

Build with catkin tools:

```bash
cd ~/baxter_noetic_ws
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build --jobs 1 --limit-status-rate 0.2 --no-notify
source install/setup.bash
```

The Dockerfile is the source of truth for the minimal apt dependency set used by CI.

## Docker Compose

Compose builds the same local image and defines four services: `roscore`, `baxter_sim`, `moveit`, and `rviz`.

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker compose build
docker compose up roscore
```

In another terminal, start simulator and planning services as needed:

```bash
docker compose up baxter_sim
docker compose up moveit
docker compose up rviz
```

For non-GUI validation:

```bash
docker compose config
docker compose run --rm --no-deps roscore bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch'
```

Gazebo and RViz services request `gpus: all` and mount X11/WSLg sockets. If the host Docker runtime reports `failed to discover GPU vendor from CDI`, either fix host GPU/CDI support or temporarily remove/override `gpus: all` for software-rendered GUI testing.

## Simulator Basics

With the workspace sourced and a ROS master available:

```bash
roscore
roslaunch baxter_gazebo baxter_world.launch
```

Useful simulator commands in another terminal:

```bash
source ~/baxter_noetic_ws/install/setup.bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
```

The verified simulator launch path is `baxter_gazebo baxter_world.launch`.

## MoveIt Basics

For simulation or a robot session with `robot_description` available:

```bash
source ~/baxter_noetic_ws/install/setup.bash
roslaunch baxter_moveit_config planning_context.launch load_robot_description:=true
roslaunch baxter_moveit_config move_group.launch
```

Start RViz separately when a display is available:

```bash
roslaunch baxter_moveit_config moveit_rviz.launch config:=true
```

For a fake-controller/demo path that does not require Gazebo:

```bash
roslaunch baxter_moveit_config demo_dummy.launch
```

Before executing trajectories on a real or simulated Baxter through the action interface, start the trajectory action server:

```bash
rosrun baxter_interface joint_trajectory_action_server.py
```

## Tests And CI Checks

The GitHub Actions workflow builds the Docker image, runs the full smoke test, runs the kinematics gtest, and parses key Gazebo/MoveIt launch files.

Note: `test/test_smoke.py` is the automated smoke suite used by CI; the similarly named `baxter_tools/scripts/smoke_test.py` is the legacy Rethink interactive tool that requires a live robot.

Run the same checks locally after building the image:

```bash
docker run --rm baxter-noetic:n07 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

```bash
docker run --rm baxter-noetic:n07 bash -lc 'source /root/baxter_ws/devel/setup.bash; cd /root/baxter_ws/build/baxter_sim_kinematics; make test_kinematics; roscore >/tmp/roscore.log 2>&1 & roscore_pid=$!; trap "kill $roscore_pid || true" EXIT; sleep 5; timeout 120 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics'
```

```bash
docker run --rm baxter-noetic:n07 bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

For native work, the equivalent catkin targets are:

```bash
catkin run_tests --no-deps baxter_tools baxter_sim_kinematics --limit-status-rate 0.2 --no-notify
catkin_test_results build
```

## Physical Robot Notes

The legacy `baxter/baxter.sh` helper is still present, but it contains local network assumptions and must be reviewed before use. For a physical robot session, source ROS Noetic and set the robot networking variables for your robot and workstation before running `baxter_tools` or examples.

```bash
source /opt/ros/noetic/setup.bash
source ~/baxter_noetic_ws/install/setup.bash
export ROS_MASTER_URI=http://<robot-hostname-or-ip>:11311
export ROS_IP=<workstation-ip-on-robot-network>
rosrun baxter_tools enable_robot.py -s
```

Do not run motion examples until networking, e-stop state, and workspace safety are confirmed.

## Troubleshooting

| Symptom or warning | Meaning / action |
|--------------------|------------------|
| `The catkin CMake module was not found` | The shell is not using ROS Noetic. Source `/opt/ros/noetic/setup.bash` or use Docker. |
| `ROS_DISTRO=jazzy` while building this checkout | Wrong ROS distro for this repository. Use Noetic or Docker. |
| Gazebo Classic deprecation warnings | Expected on current Noetic/Focal packages; CI build still passes. |
| Qt/googletest `AUTOMOC` developer warnings | Known `baxter_sim_io` build warnings; CI build still passes. |
| `xacro: in-order processing became default` | Harmless Noetic warning from older launch arguments. |
| `Child elements of a <xacro:include> tag are ignored` | Known Baxter xacro warning; verified launch-parse checks still pass. |
| Python `DeprecationWarning` for invalid escape sequences | Existing warning noise in some scripts; smoke tests pass. |
| Kinematics gtest hangs or reports ROS master connection errors | Start `roscore` before running `test_kinematics`. |
| Docker Compose GPU/CDI error | Host Docker GPU discovery is not configured. Fix GPU/CDI or override `gpus: all`. |
| Kinect/Xtion/warehouse/CHOMP/STOMP launch paths | Not part of the verified CI quick path. Prefer `planning_context`, `move_group`, `moveit_rviz`, or `demo_dummy`. |

## More Docs

Package-level notes live beside each package. Offline Baxter reference PDFs are listed in [`docs/README.md`](docs/README.md).
