---
step: N07
title: Modern Docker Setup
agent_date: 2026-07-14
status: completed
previous_steps: [N01, N02, N03, N04, N05, N06]
---

# N07: Modern Docker Setup

## Task

Replace the obsolete Kinetic Docker setup with a Noetic Dockerfile that builds the Baxter Noetic catkin workspace and includes the simulator/Gazebo dependencies.

## Changes

- Replaced `src/baxter_noetic/baxter_simulator/Dockerfile` with a single-stage `ros:noetic-ros-base-focal` Dockerfile.
- Removed the old Kinetic `wstool` source fetch path; the image now builds the current local checkout copied into `/root/baxter_ws/src/baxter_noetic`.
- Added Noetic build/runtime dependencies through apt plus `rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y`.
- Added `rosdep update --include-eol-distros` because current rosdep skips Noetic by default after EOL.
- Added minimal source-root `.dockerignore` at `src/baxter_noetic/.dockerignore`.
- Fixed two build/launch blockers exposed by the Docker verification:
  - `baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp`: compute FK into `PoseStamped`, then copy pose/header into `EndpointState`.
  - `baxter_simulator/baxter_gazebo/launch/baxter_world.launch`: removed the comment before the XML declaration.

## Verification Results

### Docker availability

Command:

```bash
docker --version
```

Result:

```text
Docker version 29.6.1, build 8900f1d
```

### Whitespace/static check

Command:

```bash
git diff --check -- "baxter_simulator/Dockerfile" ".dockerignore" "baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp" "baxter_simulator/baxter_gazebo/launch/baxter_world.launch"
```

Result: passed with no output.

### Docker build

Command:

```bash
docker build -f "baxter_simulator/Dockerfile" -t baxter-noetic:n07 .
```

Working directory:

```text
~/baxter_noetic_ws/src/baxter_noetic
```

Result: passed. Final catkin build summary from the image build:

```text
[build] Summary: All 18 packages succeeded!
[build] Warnings: 5 packages succeeded with warnings.
[build] Failed: No packages failed.
[build] Runtime: 3 minutes and 37.7 seconds total.
```

Warnings observed:

- Gazebo Classic/`gazebo_msgs`/`gazebo_ros_control` deprecation warnings from current Noetic packages.
- Qt/googletest AUTOMOC developer warnings from `baxter_sim_io` CMake configuration.

### Container launch-parse smoke

Command:

```bash
docker run --rm baxter-noetic:n07 bash -lc 'source /root/baxter_ws/install/setup.bash && rospack find baxter_gazebo && roslaunch --nodes baxter_gazebo baxter_world.launch'
```

Result: passed. The image resolved `baxter_gazebo` and `roslaunch --nodes` listed the simulator nodes, including `/gazebo`, `/gazebo_gui`, `/urdf_spawner`, `/baxter_sim_kinematics_left`, `/baxter_sim_kinematics_right`, `/baxter_emulator`, `/robot_state_publisher`, and `/baxter_sim_io`.

Warnings observed:

```text
warning: Child elements of a <xacro:include> tag are ignored
when processing file: /root/baxter_ws/install/share/baxter_description/urdf/baxter.urdf.xacro
```

## Notes

- An initial Docker build hit the tool timeout during the first large apt install; reruns used cached layers.
- One build attempt failed on a temporary DNS lookup for `snapshots.ros.org`; retry succeeded without Dockerfile changes.
- Full Gazebo runtime was not launched in this step to avoid GUI/headless runtime assumptions; launch XML/package resolution was verified inside the built image.

## Artifacts

- `src/baxter_noetic/baxter_simulator/Dockerfile`
- `src/baxter_noetic/.dockerignore`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp`
- `src/baxter_noetic/baxter_simulator/baxter_gazebo/launch/baxter_world.launch`
- `logs/N07_docker_setup.log.md`
