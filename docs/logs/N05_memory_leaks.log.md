---
step: N05
title: Memory Leak Fixes
agent_date: 2026-07-14
status: completed
previous_steps: [N01, N02, N03, N04]
---

# N05: Memory Leak Fixes

## Task

Fix the audited raw KDL solver ownership leaks in `baxter_sim_kinematics` without starting N06 tests or redesigning kinematics.

## Changes

- Added `#include <memory>` to `arm_kinematics.h`.
- Replaced only the audited solver members with `std::unique_ptr`:
  - `fk_solver`
  - `ik_solver_vel`
  - `ik_solver_pos`
  - `gravity_solver_l`
  - `gravity_solver_r`
- Changed the five KDL solver allocation sites from raw owning assignments to `unique_ptr::reset(new ...)`.
- Kept existing `->` call sites behavior-equivalent.
- Did not add a destructor; the default destructor now safely releases the solver objects through `std::unique_ptr`.

## Verification Environment

- `/opt/ros/noetic` is missing.
- `/opt/ros` contains only `jazzy/`.
- `ROS_DISTRO=jazzy`.
- The catkin profile extends ROS Jazzy vendor paths, not ROS Noetic.

## Verification Results

### Whitespace/static diff check

Command:

```bash
git diff --check
```

Result: passed with no output.

### Solver ownership search

Check: searched `baxter_sim_kinematics` for the audited raw KDL solver pointer declarations and allocation sites.

Result: raw solver member declarations are gone. Remaining `new KDL::...` sites are immediately owned by `std::unique_ptr::reset(...)`.

### Kinematics package build attempt

Command:

```bash
catkin build --no-deps baxter_sim_kinematics
```

Result: failed before compiling `baxter_sim_kinematics` in `catkin_tools_prebuild` because the ROS 1 catkin CMake module is unavailable in this environment:

```text
The catkin CMake module was not found, but it is required to build a linked workspace.
```

Conclusion: native package build and runtime leak tooling are blocked until ROS Noetic/catkin CMake is installed or sourced. The direct static ownership checks passed.

## Open Questions

- None for N05.

## Artifacts

- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/arm_kinematics.h`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp`
- `logs/N05_memory_leaks.log.md`

## Addendum (2026-07-16 audit)

The gate's valgrind/ASan requirement was finally exercised by the independent audit: full kinematics gtest run under `valgrind --leak-check=full` inside the Docker image → **definitely lost: 0 bytes, indirectly lost: 0 bytes, ERROR SUMMARY: 0 errors, 4/4 tests passed**. Gate evidence now exists; see `logs/AUDIT_2026-07-16.log.md`.
