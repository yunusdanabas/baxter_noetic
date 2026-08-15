---
step: N03
title: Testing Infrastructure
agent_date: 2026-07-14
status: completed
previous_steps: [N01, N02]
---

# N03: Testing Infrastructure

## Task

Set up minimal catkin test infrastructure for the Baxter Noetic source checkout without fixing audited source bugs.

## Changes

- Added one shared smoke test entry point at `src/baxter_noetic/test/test_smoke.py`.
- Hooked the shared smoke test into catkin from `src/baxter_noetic/baxter_tools/CMakeLists.txt` with `catkin_add_nosetests(...)` under `CATKIN_ENABLE_TESTING`.
- Added test-only dependencies to `src/baxter_noetic/baxter_tools/package.xml`: `python3-nose`, `xacro`.
- Did not add a test runner script; direct `python3 .../test_smoke.py` and `catkin run_tests --no-deps baxter_tools` are enough.

## Smoke Coverage

- Verifies all 17 expected `package.xml` manifests parse and names match the audited package inventory.
- Compiles all Python files in memory to catch syntax errors without creating `__pycache__` artifacts.
- Statically checks Python imports across scripts/modules to catch missing import targets such as `xacro_jade`.
- Imports representative installable Python modules/packages without starting robot hardware.
- Parses all `.launch` files as XML.
- Runs `xacro` on selected top-level robot, gripper, and MoveIt xacro files.

## Decisions

- Used one package-group smoke test target instead of one file per package. The test itself enumerates all 17 packages.
- Left audited source failures untouched; this step only adds tests that expose them.
- Kept tests to Python stdlib plus existing ROS/catkin test plumbing.

## Verification Environment

- Current shell is not a Noetic environment: `ROS_DISTRO=jazzy`.
- `/opt/ros` contains `jazzy/` only; no `/opt/ros/noetic` was present.
- Active `python3` could not import `rospkg` before running tests.

## Verification Results

### Direct Smoke Test

Command:

```bash
python3 src/baxter_noetic/test/test_smoke.py
```

Result: executed the harness; `6` unittest cases ran, `4` failed.

Passing checks:

- `test_package_manifests`
- `test_python_syntax`

Failing checks documented expected/current blockers:

- `test_importable_modules`: failed on missing ROS1 Python modules such as `rospy` in this environment.
- `test_python_imports_are_resolvable`: failed on missing ROS1/generated/optional imports in this environment and exposed audited issues such as `baxter_examples/scripts/send_urdf_fragment.py:35: xacro_jade`.
- `test_launch_files_are_xml`: failed on audited issue `baxter_simulator/baxter_gazebo/launch/baxter_world.launch: XML or text declaration not at start of entity`.
- `test_selected_xacro_files_expand`: failed because the available Jazzy `xacro` uses ROS 2 package lookup and cannot find ROS 1 packages from `ROS_PACKAGE_PATH`.

### Catkin Test Attempt

Command:

```bash
catkin run_tests --no-deps baxter_tools
```

Initial result: failed before testing because `.catkin_tools/profiles/profiles.yaml` did not exist.

Command:

```bash
catkin init
```

Result: initialized workspace metadata. The profile extends ROS Jazzy vendor paths, confirming the wrong ROS distro for this Noetic checkout.

Command:

```bash
catkin run_tests --no-deps baxter_tools
```

Result: catkin_tools reached the workspace but refused because `baxter_tools` had not been built.

Command:

```bash
catkin build --no-deps baxter_tools
```

Result: failed in `catkin_tools_prebuild` before building `baxter_tools`:

```text
The catkin CMake module was not found, but it is required to build a linked workspace.
```

Conclusion: the catkin test hook is present in the working tree, but this machine cannot execute ROS Noetic catkin tests until a ROS 1 Noetic/catkin CMake environment is available.

## Open Questions

- None for N03. Later agents should fix the source failures exposed by the smoke tests in their assigned steps.

## Artifacts

- `src/baxter_noetic/test/test_smoke.py`
- `src/baxter_noetic/baxter_tools/CMakeLists.txt`
- `src/baxter_noetic/baxter_tools/package.xml`
- `logs/N03_testing_infrastructure.log.md`
