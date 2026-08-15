---
step: PX2
title: P2 Build Package Launch Polish
agent_date: 2026-07-16
status: completed
previous_steps: [N02, N12]
---

# PX2: P2 Build, Package, And Launch Polish

## Scope

- Picked one small P2 batch covering directly evidenced package metadata and install hygiene.
- Did not tag, publish images, create release notes, or add published-image documentation.
- Did not install the pick-and-place example scripts because `N02-P2-03` flags them as broken/unsafe.
- Did not chase optional MoveIt demo, warehouse, URDF/xacro, or simulator behavior cleanup in this pass.

## Code Changes

- Fixed `baxter_tools/CMakeLists.txt` so `share/` and `launch/` install under `${CATKIN_PACKAGE_SHARE_DESTINATION}` instead of `${CATKIN_PACKAGE_SHARE_DESTINATION}/share`.
- Added directly supported `baxter_tools` dependencies for `std_msgs` and `std_srvs` based on script imports.
- Added directly supported `baxter_interface` runtime dependency `python3-numpy` based on installed module imports.
- Added directly supported `baxter_examples` dependencies for `std_msgs`, `geometry_msgs`, `gazebo_msgs`, and `joy` based on script imports and joystick launch files.

## Verification

Command:

```bash
git diff --check
```

Result: passed.

Command:

```bash
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:px2 .
```

Result: passed. Catkin summary reported all 18 packages succeeded.

Observed existing warning classes:

- Gazebo Classic deprecation warnings.
- Qt/googletest AUTOMOC warnings.

Command:

```bash
docker run --rm baxter-noetic:px2 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

Result: passed.

```text
Ran 6 tests in 5.348s
OK
```

Observed existing warnings:

- Python `DeprecationWarning` noise for invalid escape sequences in `robot_enable.py` and `update_robot.py`.

Command:

```bash
docker run --rm baxter-noetic:px2 bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

Result: passed.

Nodes listed included:

```text
/gazebo
/gazebo_gui
/base_to_world
/urdf_spawner
/baxter_sim_kinematics_left
/baxter_sim_kinematics_right
/baxter_emulator
/robot/controller_spawner
/robot/controller_spawner_stopped
/robot/left_gripper_controller_spawner_stopped
/robot/right_gripper_controller_spawner_stopped
/robot_state_publisher
/baxter_sim_io
/move_group
```

Observed existing launch warnings:

- `xacro: in-order processing became default in ROS Melodic. You can drop the option.`
- `Child elements of a <xacro:include> tag are ignored` from `baxter.urdf.xacro`.

Additional install-path check:

```bash
docker run --rm baxter-noetic:px2 bash -lc 'test -d /root/baxter_ws/install/share/baxter_tools/launch && test ! -d /root/baxter_ws/install/share/baxter_tools/share/launch'
```

Result: passed.

## Deferred Items

- `baxter_examples/scripts/baxter_pnpcode.py` and `baxter_pnpcode2.py` remain uninstalled because they need behavior fixes before exposing them in installed workspaces.
- Remaining P2 simulator CMake/export, MoveIt optional launch, URDF/xacro, generated artifact, and example-model cleanup findings remain for later targeted passes.
- Kinematics gtest was not run because this pass did not touch simulator, URDF/xacro, or kinematics-adjacent code.

## Status

PX2 is complete.
