---
step: N10
title: tf -> tf2 Migration
agent_date: 2026-07-15
status: completed
previous_steps: [N01, N02, N03, N04, N05, N06, N07, N08, N09, N09.5]
---

# N10: tf -> tf2 Migration

## Task

Migrate remaining practical ROS Noetic `tf` usage to `tf2` without starting the N11 documentation overhaul.

## Pre-Change Review

- Reviewed `logs/N09_5_ci_hardening.log.md` before changing any `tf` usage.
- N09.5 confirmed the full smoke tests and full kinematics gtest passed before this migration, and that no tf2 migration had been done there.

## Search Findings

- Remaining C++ tf1 usage was limited to `baxter_sim_kinematics`:
  - `include/baxter_sim_kinematics/arm_kinematics.h`
  - `src/arm_kinematics.cpp`
  - `test/test_kinematics.cpp`
- Remaining `tf` static transform launch nodes were in:
  - `baxter_moveit_config/launch/baxter_moveit_sensor_manager.launch`
  - `baxter_moveit_config/launch/demo_dummy.launch`
- `baxter_sim_hardware` declared `tf` as a direct build dependency but had no direct tf usage; it only includes `baxter_sim_kinematics`.
- No Python `tf` imports were found.

## Changes

- Replaced `tf::TransformListener` in `Kinematics` with `tf2_ros::Buffer` plus `tf2_ros::TransformListener`.
- Replaced tf1 pose transform calls with `tf_buffer.transform(...)` on `geometry_msgs::PoseStamped`.
- Replaced `tf_conversions` KDL pose conversion calls with `tf2_kdl` conversions.
- Removed tf1 includes from the kinematics gtest.
- Replaced `tf` and `tf_conversions` CMake/package dependencies in `baxter_sim_kinematics` with `tf2_ros`, `tf2_geometry_msgs`, and `tf2_kdl`.
- Removed unused direct `tf` dependency from `baxter_sim_hardware`.
- Replaced MoveIt launch `pkg="tf" type="static_transform_publisher"` nodes with `pkg="tf2_ros" type="static_transform_publisher"` and tf2-compatible args.
- Added `tf2_ros` runtime dependency to `baxter_moveit_config` for those launch files.

## Verification Results

Working directory for source commands:

```text
~/baxter_noetic_ws/src/baxter_noetic
```

### Static whitespace check

Command:

```bash
git diff --check
```

Result: passed with no output.

### tf1 usage search

Check: searched the source checkout for tf1 C++ includes, `tf::` APIs, `tf_conversions`, and `pkg="tf"` launch nodes.

Result: passed; no matches remained.

### Docker build

Command:

```bash
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:n10 .
```

Result: passed after one compile fix. Final catkin summary:

```text
[build] Summary: All 18 packages succeeded!
[build] Warnings: 5 packages succeeded with warnings.
[build] Failed: No packages failed.
[build] Runtime: 3 minutes and 25.6 seconds total.
```

Notes:

- First build attempt caught a `tf2_kdl` conversion mistake: `tf2::toMsg(KDL::Frame)` returns `geometry_msgs::Pose`, not `geometry_msgs::Transform`. The code was corrected and the build passed.
- Warnings matched prior Docker runs: Gazebo Classic deprecation warnings and Qt/googletest AUTOMOC developer warnings.

### Full smoke check

Command:

```bash
docker run --rm baxter-noetic:n10 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

Result: passed.

```text
Ran 6 tests in 2.905s

OK
```

Observed warnings: existing Python `DeprecationWarning` messages for invalid escape sequences in `robot_enable.py` and `update_robot.py`.

### Full kinematics gtest

Command:

```bash
docker run --rm baxter-noetic:n10 bash -lc 'source /root/baxter_ws/devel/setup.bash; cd /root/baxter_ws/build/baxter_sim_kinematics; make test_kinematics; roscore >/tmp/roscore.log 2>&1 & roscore_pid=$!; trap "kill $roscore_pid || true" EXIT; sleep 5; timeout 120 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics'
```

Result: passed.

```text
[==========] Running 4 tests from 1 test suite.
[  PASSED  ] 4 tests.
```

Observed warnings: expected ROS error/fatal logs from the negative missing-limits test, plus `XmlRpcServer::acceptConnection` warnings while the local test node exits.

### Launch parse checks

Command:

```bash
docker run --rm baxter-noetic:n10 bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch && roslaunch --nodes baxter_moveit_config demo_dummy.launch && roslaunch --nodes baxter_moveit_config baxter_moveit_sensor_manager.launch'
```

Result: passed. Nodes listed included `/base_to_world`, `/move_group`, `/joint_state_publisher`, `/robot_state_publisher`, `/rviz_*`, and `/world_to_base`.

Observed warnings matched prior runs: xacro child include warnings and the in-order processing deprecation note.

### tf2 static publisher arg probe

Command:

```bash
docker run --rm baxter-noetic:n10 bash -lc 'source /root/baxter_ws/install/setup.bash; roscore >/tmp/roscore.log 2>&1 & roscore_pid=$!; trap "kill $roscore_pid || true" EXIT; sleep 5; timeout 5 rosrun tf2_ros static_transform_publisher 0.15 0.075 0.5 0.0 0.7854 0.0 torso camera_link; status=$?; test $status -eq 124'
```

Result: passed. The node started with the updated 6-DOF tf2 arg form and was stopped by the expected timeout.

## Open Questions

- None for N10.

## Artifacts

- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/arm_kinematics.h`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/test/test_kinematics.cpp`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/CMakeLists.txt`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/package.xml`
- `src/baxter_noetic/baxter_simulator/baxter_sim_hardware/CMakeLists.txt`
- `src/baxter_noetic/baxter_simulator/baxter_sim_hardware/package.xml`
- `src/baxter_noetic/baxter_moveit_config/launch/baxter_moveit_sensor_manager.launch`
- `src/baxter_noetic/baxter_moveit_config/launch/demo_dummy.launch`
- `src/baxter_noetic/baxter_moveit_config/package.xml`
- `logs/N10_tf2_migration.log.md`
