---
step: N08
title: Docker Compose
agent_date: 2026-07-14
status: completed
previous_steps: [N01, N02, N03, N04, N05, N06, N07]
---

# N08: Docker Compose

## Task

Create a minimal Docker Compose setup for the Baxter Noetic simulator stack without starting N09 CI/CD.

## Changes

- Added `src/baxter_noetic/docker-compose.yml`.
- Reused the N07 image name and Dockerfile: `baxter-noetic:n07` from `baxter_simulator/Dockerfile`.
- Added four services only:
  - `roscore`: `roscore`
  - `baxter_sim`: `roslaunch baxter_gazebo baxter_world.launch`
  - `moveit`: loads `planning_context.launch`, then runs `move_group.launch`
  - `rviz`: `roslaunch baxter_moveit_config moveit_rviz.launch config:=true`
- Used host networking with `ROS_MASTER_URI=http://127.0.0.1:11311` and `ROS_IP=127.0.0.1` so ROS 1 dynamic node ports work across the compose services on Linux/WSL2.
- Added minimal GUI/GPU settings only to Gazebo/RViz services: `DISPLAY`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR`, `QT_X11_NO_MITSHM`, `NVIDIA_DRIVER_CAPABILITIES`, X11/WSLg bind mounts, and `gpus: all`.
- Did not add `devcontainer.json`; it would only duplicate the compose setup and does not materially help VS Code users for this step.

## Verification Results

Working directory for compose commands:

```text
~/baxter_noetic_ws/src/baxter_noetic
```

### Docker Compose availability

Command:

```bash
docker compose version
```

Result:

```text
Docker Compose version v5.1.4
```

### Compose config

Command:

```bash
docker compose config
```

Result: passed. The rendered config includes `roscore`, `baxter_sim`, `moveit`, and `rviz`; GPU/display settings are present only on `baxter_sim` and `rviz`.

### Compose build

Command:

```bash
docker compose build
```

Result: passed. The image built as `baxter-noetic:n07`; final catkin summary:

```text
[build] Summary: All 18 packages succeeded!
[build] Warnings: 5 packages succeeded with warnings.
[build] Failed: No packages failed.
[build] Runtime: 3 minutes and 2.3 seconds total.
```

Observed warnings matched N07-era warnings: Gazebo Classic deprecation warnings and Qt/googletest AUTOMOC developer warnings.

### Gazebo launch parse through compose

Command:

```bash
docker compose run --rm --no-deps roscore bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch'
```

Result: passed. Nodes listed included `/gazebo`, `/gazebo_gui`, `/urdf_spawner`, `/baxter_sim_kinematics_left`, `/baxter_sim_kinematics_right`, `/baxter_emulator`, `/robot_state_publisher`, and `/baxter_sim_io`.

Warnings observed:

```text
warning: Child elements of a <xacro:include> tag are ignored
when processing file: /root/baxter_ws/install/share/baxter_description/urdf/baxter.urdf.xacro
```

### MoveIt launch parse through compose

Command:

```bash
docker compose run --rm --no-deps roscore bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

Result: passed. Node listed:

```text
/move_group
```

Warnings observed:

```text
xacro: in-order processing became default in ROS Melodic. You can drop the option.
warning: Child elements of a <xacro:include> tag are ignored
when processing file: /root/baxter_ws/install/share/baxter_description/urdf/baxter.urdf.xacro
```

### RViz launch parse through compose

Command:

```bash
docker compose run --rm --no-deps roscore bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_moveit_config moveit_rviz.launch config:=true'
```

Result: passed. Node listed:

```text
/rviz_yunusdanabas_7_7501213405957841870
```

### GUI/GPU container start check

Command:

```bash
docker compose run --rm --no-deps rviz bash -lc 'true'
```

Result: blocked by host GPU/CDI discovery before the container command ran:

```text
Error response from daemon: failed to discover GPU vendor from CDI: no known GPU vendor found
```

Conclusion: GUI/GPU runtime could not be verified on this host. Non-GUI compose config, build, and launch-parse checks passed.

### Whitespace/static check

Command:

```bash
git diff --check -- docker-compose.yml
```

Result: passed with no output.

## Open Questions

- None for N08. Full Gazebo/RViz runtime should be retested on a host with working Docker GPU/CDI or after disabling `gpus: all` for software-rendered GUI use.

## Artifacts

- `src/baxter_noetic/docker-compose.yml`
- `logs/N08_docker_compose.log.md`
