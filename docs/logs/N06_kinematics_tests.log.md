---
step: N06
title: Kinematics Unit Tests
agent_date: 2026-07-14
status: completed
previous_steps: [N01, N02, N03, N04, N05]
---

# N06: Kinematics Unit Tests

## Task

Add package-local gtest coverage for `baxter_sim_kinematics` without starting N07 Docker work.

## Changes

- Added `baxter_simulator/baxter_sim_kinematics/test/test_kinematics.cpp`.
- Added a `catkin_add_gtest(test_kinematics ...)` hook to `baxter_simulator/baxter_sim_kinematics/CMakeLists.txt` under `CATKIN_ENABLE_TESTING`.
- Kept coverage local to `baxter_sim_kinematics`; no mocks, no fixtures, no parameterized tests, no tf2 migration.

## Test Coverage Added

- Loads a tiny one-joint URDF and verifies joint names and limits are read correctly.
- Rejects a non-continuous joint with missing limits.
- Computes known FK for a one-joint chain at `q=0`.
- Computes known IK for the same one-joint pose from a valid seed.
- Rejects incomplete FK joint-state requests.
- Rejects IK seeds that omit required model joints.
- Verifies disabled gravity output sizes left and right arms independently and zeros both effort arrays.

## Deliberate Limits

- Full `init_grav()` success is not unit-tested because the implementation blocks on `/gazebo/get_link_properties` and `/gazebo/set_link_properties`; adding fake Gazebo services or rostest launch plumbing would exceed the N06 scope.
- The test accesses private fields only inside the test translation unit to set up a tiny KDL model without ROS parameter server or Gazebo dependencies. Production code was not changed.

## Verification Environment

- `/opt/ros` contains only `jazzy/`; `/opt/ros/noetic` is missing.
- The catkin profile extends ROS Jazzy vendor paths, not ROS Noetic.

## Verification Results

### Tracked whitespace check

Command:

```bash
git diff --check
```

Result: passed with no output.

### New test whitespace check

Command:

```bash
git diff --check --no-index /dev/null baxter_simulator/baxter_sim_kinematics/test/test_kinematics.cpp
```

Result: reported no whitespace errors.

### Kinematics package build attempt

Command:

```bash
catkin build --no-deps baxter_sim_kinematics
```

Result: failed before compiling `baxter_sim_kinematics` in `catkin_tools_prebuild` because the ROS 1 catkin CMake module is unavailable in this environment:

```text
The catkin CMake module was not found, but it is required to build a linked workspace.
```

### Kinematics package test attempt

Command:

```bash
catkin run_tests --no-deps baxter_sim_kinematics
```

Result: failed because `baxter_sim_kinematics` is not built:

```text
Packages have to be built before they can be tested.
The following requested packages are not built yet:
 - baxter_sim_kinematics
```

Conclusion: the gtest target and test source are present in the working tree, but native build/test execution remains blocked until a ROS Noetic/catkin CMake environment is available.

## Open Questions

- None for N06.

## Artifacts

- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/CMakeLists.txt`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/test/test_kinematics.cpp`
- `logs/N06_kinematics_tests.log.md`
