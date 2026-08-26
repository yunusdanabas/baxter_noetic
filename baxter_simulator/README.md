# Baxter Simulator

Gazebo simulation, simulated hardware, simulated IO, controllers, and kinematics for Baxter on ROS Noetic.

## Packages

| Package | Purpose |
|---------|---------|
| `baxter_simulator` | Metapackage |
| `baxter_gazebo` | Gazebo world and robot spawn launch files |
| `baxter_sim_controllers` | Gazebo controller plugins |
| `baxter_sim_examples` | Simulation-specific examples and models |
| `baxter_sim_hardware` | Emulated Baxter hardware interfaces |
| `baxter_sim_io` | Qt-based simulated navigator IO |
| `baxter_sim_kinematics` | FK, IK, gravity compensation, and kinematics tests |

## Docker Build

Build the local Noetic image from the repository root:

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:local .
```

No pre-built image is published yet.

## Simulator Launch

After building and sourcing the workspace:

```bash
roslaunch baxter_gazebo baxter_world.launch
```

Parse the launch without starting Gazebo:

```bash
roslaunch --nodes baxter_gazebo baxter_world.launch
```

In Docker:

```bash
docker run --rm baxter-noetic:local bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch'
```

## Compose

The root `docker-compose.yml` defines `roscore`, `baxter_sim`, `moveit`, and `rviz` services.

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker compose build
docker compose up roscore
docker compose up baxter_sim
```

`baxter_sim` runs Gazebo headless by default. RViz needs a working host display; add a local Compose override for X11/WSLg/GPU mounts when you want GUI rendering.

## Kinematics Test

```bash
docker run --rm baxter-noetic:local bash -lc 'source /root/baxter_ws/devel/setup.bash; cd /root/baxter_ws/build/baxter_sim_kinematics; make test_kinematics; roscore >/tmp/roscore.log 2>&1 & roscore_pid=$!; trap "kill $roscore_pid || true" EXIT; sleep 5; timeout 120 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics'
```

`test_kinematics` needs a ROS master because the kinematics class owns a tf2 listener.

## Known Warnings

Gazebo Classic deprecation warnings and Qt/googletest `AUTOMOC` developer warnings are expected in the current Docker build and do not fail CI.
