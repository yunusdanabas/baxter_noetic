---
step: N04
title: Fix Kinematics Bugs
agent_date: 2026-07-14
status: completed
previous_steps: [N01, N02, N03]
---

# N04: Fix Kinematics Bugs

## Task

Fix the audited correctness bugs in `baxter_sim_kinematics` without doing the N05 memory-ownership refactor.

## Changes

- Fixed `Kinematics::loadModel()` so URDF parse failure returns `false`, not truthy `-1`.
- Made `position_kinematics::init()` fail when `Kinematics::create(...)` returns an empty pointer.
- Changed `FilterJointState()` to reject joint states whose `name` and `position` sizes differ, and to reject missing arm joints instead of writing past the result buffers.
- Changed FK callback flow to publish endpoint state only when filtering and FK both succeed.
- Changed FK/IK seed handling in `arm_kinematics.cpp` to validate matching `JointState` vector sizes and require all model joints before indexing or solving.
- Replaced the broken `readJoints()` `for`/`while` traversal with a single parent-link traversal and added checks for parent joints, parent links, joint lookup, and non-continuous joint limits.
- Fixed the left-arm copy-paste fatal message in `position_kinematics.cpp`.
- Split gravity joint counts into `num_joints_l` and `num_joints_r`, and sized left/right gravity arrays independently.
- Removed the unnecessary IK service `ros::Rate` sleep.
- Guarded IK user seed access with `req_index < req.seed_angles.size()` before indexing `req.seed_angles[req_index]`.

## Verification Environment

- `/opt/ros/noetic` is missing.
- `ROS_DISTRO=jazzy`.
- The catkin profile still extends ROS Jazzy vendor paths, not ROS Noetic.

## Verification Results

### Whitespace/static diff check

Command:

```bash
git diff --check
```

Result: passed with no output.

### Kinematics package build attempt

Command:

```bash
catkin build --no-deps baxter_sim_kinematics
```

Result: failed before compiling `baxter_sim_kinematics` in `catkin_tools_prebuild` because the ROS 1 catkin CMake module is unavailable in this environment:

```text
The catkin CMake module was not found, but it is required to build a linked workspace.
```

Conclusion: native package build is blocked until ROS Noetic/catkin CMake is installed or sourced.

### Direct smoke test

Command:

```bash
python3 src/baxter_noetic/test/test_smoke.py
```

Result: ran `6` unittest cases; `2` passed and `4` failed. Failures match the N03 environment/audit blockers: missing ROS 1 Python modules such as `rospy`, existing invalid `baxter_world.launch` XML, `xacro_jade`, and Jazzy `xacro` package lookup failures.

## Open Questions

- None for N04.

## Artifacts

- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/arm_kinematics.h`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/position_kinematics.h`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp`
- `logs/N04_kinematics_bugs.log.md`
