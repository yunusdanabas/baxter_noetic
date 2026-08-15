---
step: N12
title: Fix Remaining Non-Kinematics Bugs
agent_date: 2026-07-16
status: completed
previous_steps: [N01, N02, N03, N04, N05, N06, N07, N08, N09, N09.5, N10, N11]
---

# N12: Fix Remaining Non-Kinematics Bugs

## Task

Fix the remaining non-kinematics P0/P1 bugs from the N02 audit before any release work.

## Code Changes

- Fixed `baxter_interface/src/baxter_interface/limb.py` Python 3 ROS message assignment by converting command dictionary views to lists.
- Fixed float-to-`UInt16` publish paths in `joint_trajectory_action.py` and `joint_velocity_wobbler.py` by publishing integer rates.
- Hardened `baxter_sim_hardware/src/baxter_emulator.cpp` against empty laser scans, unknown navigator light names, and sparse `JointState` arrays.
- Replaced Python 2 leftovers: `xrange` in `joint_velocity_puppet.py` and `joint_velocity_wobbler.py`, `operator.div` in `joint_trajectory_file_playback.py`.
- Made `_get_trajectory_parameters()` report failure after invalid-joint aborts, and made `_on_trajectory_action()` return immediately on that failure.
- Changed `head_action.py` command dispatch to call `set_pan(..., timeout=0.0)` so the action loop remains responsible for feedback, timeout, and preemption.
- Checked sub-controller `init(robot, joint_nh)` return values in all five simulator controllers.
- Removed the non-standard VLA in `baxter_effort_controller.cpp` while touching the effort controller init path.
- Added top-level `src/baxter_noetic/LICENSE` with the existing BSD license text used by the packages.
- Updated `docker-compose.yml` so default compose no longer requires host-specific X11/WSLg bind mounts or GPU reservations, uses `roslaunch --wait` against the shared master, and starts Gazebo headless by default.
- Updated README compose notes to match the new headless defaults and document RViz display forwarding as a GUI-only requirement.

## Verification

### Static Checks

Command:

```bash
git diff --check
```

Result: passed.

Command:

```bash
python3 -m py_compile baxter_interface/src/baxter_interface/limb.py baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py baxter_interface/src/head_action/head_action.py baxter_examples/scripts/joint_velocity_wobbler.py baxter_examples/scripts/joint_velocity_puppet.py baxter_examples/scripts/joint_trajectory_file_playback.py
```

Result: passed.

Command:

```bash
grep-equivalent static searches for xrange, operator.div, raw dict-view message assignments, uncast rate publishes, blocking head set_pan, emulator unchecked find/range/sparse array patterns, and effort-controller VLA
```

Result: passed. No stale matches remained except guarded `ranges[0]` accesses.

### Docker Build

Command:

```bash
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:n12 .
```

Result: passed. Catkin summary reported all 18 packages succeeded. Existing warning classes remained: Gazebo Classic deprecation and Qt/googletest AUTOMOC warnings.

### Smoke Suite

Command:

```bash
docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

Result: passed.

```text
Ran 6 tests in 4.727s
OK
```

Observed warnings: existing Python `DeprecationWarning` noise for invalid escape sequences in `robot_enable.py` and `update_robot.py`.

### Launch Parse Checks

Command:

```bash
docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

Result: passed. Nodes listed included Gazebo, emulator, kinematics, controller spawner, robot state publisher, and `/move_group`.

### Kinematics GTest

Command:

```bash
docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/devel/setup.bash && cd /root/baxter_ws/build/baxter_sim_kinematics && make test_kinematics && timeout 140 bash -lc '\''roscore >/tmp/roscore.log 2>&1 & pid=$!; trap "kill $pid || true" EXIT; sleep 5; /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics'\'''
```

Result: passed.

```text
[  PASSED  ] 4 tests.
```

### Direct P0 Exercises

Command:

```bash
docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 <monkeypatched P0 exercise script>'
```

Result: passed.

```text
limb message fields use lists
joint trajectory action publishes int rate
joint velocity wobbler publishes int rate
```

Command:

```bash
docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/install/setup.bash && timeout 180 bash -lc '\''set -e; roslaunch baxter_gazebo baxter_world.launch gui:=false headless:=true >/tmp/baxter_world.log 2>&1 & launch_pid=$!; trap "kill $launch_pid || true" EXIT; for i in {1..90}; do rostopic list 2>/dev/null | grep -q /robot/sim/started && break; sleep 1; done; timeout 30 rostopic echo -n1 /robot/sim/started >/tmp/sim_started.log; rostopic pub -1 /sim/laserscan/left_hand_range/state sensor_msgs/LaserScan "{range_min: 0.0, range_max: 10.0, ranges: []}" >/tmp/pub_left.log; rostopic pub -1 /sim/laserscan/right_hand_range/state sensor_msgs/LaserScan "{range_min: 0.0, range_max: 10.0, ranges: []}" >/tmp/pub_right.log; rostopic pub -1 /robot/digital_io/command baxter_core_msgs/DigitalOutputCommand "{name: not_a_light, value: true}" >/tmp/pub_nav.log; rostopic pub -1 /robot/joint_states sensor_msgs/JointState "{name: [head_pan, left_s0], position: [], velocity: [], effort: []}" >/tmp/pub_joint.log; sleep 2; rosnode ping -c1 /baxter_emulator; python3 -c "print(\"emulator_sparse_inputs_ok\")"'\'''
```

Result: passed. `/baxter_emulator` stayed alive after empty laser scans, an unknown nav-light command, and sparse joint-state arrays.

```text
emulator_sparse_inputs_ok
```

### Compose And Runtime Checks

Command:

```bash
docker compose build
```

Result: passed. Compose image `baxter-noetic:n07` rebuilt from the patched tree.

Command:

```bash
docker compose config
```

Result: passed after compose updates.

Command:

```bash
docker compose up -d
sleep 90
docker compose ps -a
docker compose exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rosnode list'
docker compose down
```

Result: partially passed in this host environment.

- `roscore`, `baxter_sim`, and `moveit` remained up.
- Live ROS nodes included `/gazebo`, `/baxter_emulator`, both kinematics nodes, `/robot_state_publisher`, controller spawners, and `/move_group`.
- `rviz` launched but exited because this Docker host has no usable display forwarding.

RViz log excerpt:

```text
Can't open default or :0 display. Try setting DISPLAY environment variable.
```

This is now documented as a GUI-only host setup requirement. The service is no longer blocked by mandatory `/mnt/wslg`, `/tmp/.X11-unix`, or `gpus: all` settings.

Command:

```bash
docker compose up -d roscore baxter_sim moveit
docker compose exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && timeout 30 rostopic echo -n1 /robot/sim/started >/tmp/sim_started.log && rosservice list | grep -q /gazebo/set_link_properties && timeout 30 rostopic echo -n1 /robot/limb/left/gravity_compensation_torques >/tmp/left_gravity.log && rosnode list'
docker compose down
```

Result: passed. This exercised the live Gazebo/emulator startup path, verified Gazebo link-property services are available, and observed gravity compensation output. Nodes listed included `/gazebo`, `/baxter_emulator`, both kinematics nodes, and `/move_group`.

## Deferred Items

- Stale shebangs, masked `baxter_tools` exceptions, `update_robot.py` `None` handling, and stale `.rosinstall` remain for a later cleanup pass.
- RViz live GUI validation still requires a host with working display forwarding.
- No package versions were changed; the future `v1.0.0` remains a git/project release tag decision.
- No release, tag, GHCR publish, or release notes work was started.

## Status

N12 is complete. The Final Step remains pending and user-gated.
