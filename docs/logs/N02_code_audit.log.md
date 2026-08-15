---
step: N02
title: Detailed Code Audit
agent_date: 2026-07-14
status: completed
previous_steps: [N01]
---

# N02: Detailed Code Audit

## Task

Perform a read-only code audit of the Baxter Noetic source checkout at `~/baxter_noetic_ws/src/baxter_noetic/`.

## Method

- Static inspection only. No source code was modified.
- No build, Docker build, catkin test, or roslaunch execution was run in this step.
- Searched for Python 2 leftovers, deprecated APIs, test targets, Docker/CI files, launch/config issues, generated artifacts, package metadata gaps, and C++ runtime risks.
- Package count verified from `package.xml` files: 17 packages.

## Package Inventory Audited

1. `baxter_moveit_config`
2. `baxter/baxter_sdk`
3. `baxter_interface`
4. `baxter_tools`
5. `baxter_examples`
6. `baxter_simulator/baxter_sim_kinematics`
7. `baxter_simulator/baxter_gazebo`
8. `baxter_simulator/baxter_simulator`
9. `baxter_simulator/baxter_sim_examples`
10. `baxter_common/baxter_description`
11. `baxter_simulator/baxter_sim_hardware`
12. `baxter_simulator/baxter_sim_io`
13. `baxter_common/rethink_ee_description`
14. `baxter_common/baxter_maintenance_msgs`
15. `baxter_simulator/baxter_sim_controllers`
16. `baxter_common/baxter_core_msgs`
17. `baxter_common/baxter_common`

## Severity Key

- P0: likely launch/build blocker, crash, or robot-control runtime breakage.
- P1: high-priority runtime, Python 3, memory, package, or simulator correctness issue.
- P2: medium-priority behavior, dependency, install, stale artifact, or optional-launch issue.
- P3: documentation, cleanup, deprecation warning, or low-risk maintainability issue.

## Recommended Implementation Order

1. Add minimal tests first in N03 so later fixes have a runnable safety net.
2. Fix P0 kinematics and Gazebo launch blockers before deeper simulator work.
3. Fix Python 3 runtime blockers in `baxter_interface`, `baxter_tools`, and `baxter_examples`.
4. Fix simulator memory/crash risks and C++ controller safety issues.
5. Fix package manifests, CMake install/export problems, and missing runtime dependencies.
6. Modernize Docker and add CI after native smoke tests exist.
7. Migrate deprecated `tf` usage to `tf2` after kinematics tests are in place.
8. Update docs after behavior and launch commands are verified.

## P0 Findings

### N02-P0-01: Main Gazebo launch file is invalid XML

- File: `baxter_simulator/baxter_gazebo/launch/baxter_world.launch:1-3`
- Issue: A comment appears before the XML declaration. XML declarations must be the first bytes in the document.
- Impact: `roslaunch baxter_gazebo baxter_world.launch` can fail before starting the simulator.
- Suggested order: Fix before Docker, launch smoke tests, or simulator demos.

### N02-P0-02: Kinematics model creation is unchecked and can crash later

- File: `baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp:103-104`
- Later dereferences: `position_kinematics.cpp:168`, `position_kinematics.cpp:192`, `position_kinematics.cpp:200`
- Issue: `arm_kinematics::Kinematics::create(...)` can return an empty pointer, but `init()` returns `true` unconditionally.
- Impact: Missing/invalid URDF or params can start the node with a null model and crash on FK/IK callbacks.
- Suggested order: Fix in N04.

### N02-P0-03: IK requests have unchecked vector indexing

- File: `baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp:189-192`
- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:450-456`
- Issue: `req.seed_angles[req_index]`, `seed.name[i]`, and `seed.position[i]` are indexed without validating vector sizes.
- Impact: Malformed or shorter IK seed data can crash the IK service.
- Suggested order: Fix in N04 before adding broad IK tests.

### N02-P0-04: FK path ignores failures and can read past incomplete joint states

- File: `baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp:128-130`
- File: `baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp:142-155`
- File: `baxter_simulator/baxter_sim_kinematics/src/position_kinematics.cpp:164-169`
- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:507-514`
- Issue: `FilterJointState()` copies by index without checking `msg->position` length, `FKCalc()` ignores the boolean return from `getPositionFK()`, and `getPositionFK()` indexes `joint_configuration.name/position` by `num_joints` without verifying length.
- Impact: Incomplete joint states or transform/FK failures can publish invalid endpoint state or crash.
- Suggested order: Fix in N04.

### N02-P0-05: Python 3 ROS message serialization bug in limb commands

- File: `baxter_interface/src/baxter_interface/limb.py:348-349`
- File: `baxter_interface/src/baxter_interface/limb.py:368-369`
- File: `baxter_interface/src/baxter_interface/limb.py:385-386`
- Issue: Python 3 `dict.keys()` and `dict.values()` views are assigned directly to ROS message array fields.
- Impact: Position, velocity, and torque commands can fail serialization under Noetic/Python 3.
- Suggested order: Fix after N03 import/message tests exist.

### N02-P0-06: Floats are published to `std_msgs/UInt16`

- File: `baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py:98`, `:115-119`
- File: `baxter_examples/scripts/joint_velocity_wobbler.py:60`, `:69`, `:129`
- Issue: Publish rate values default to floats such as `100.0` and `500.0` but are sent to `UInt16` messages.
- Impact: Publish/serialization can fail under Python 3.
- Suggested order: Fix with Python 3 runtime blockers.

### N02-P0-07: Emulator callbacks can crash on valid but sparse ROS messages

- File: `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp:392`
- File: `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp:412`
- File: `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp:429`
- File: `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp:513-515`
- File: `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp:520-522`
- Issue: Laser callbacks assume `ranges[0]`, nav light callback dereferences `find(...)->second` without checking `end()`, and joint-state callback assumes velocity/effort arrays are populated.
- Impact: Valid ROS messages with empty ranges, unknown names, or partial JointState arrays can crash the simulator hardware node.
- Suggested order: Fix after kinematics P0 issues.

## P1 Findings

### N02-P1-01: `loadModel()` returns success on URDF parse failure

- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:232-240`
- Issue: Function returns `-1` from a `bool` function, which converts to `true`.
- Impact: URDF initialization failure can be treated as success and cascade into invalid kinematics state.
- Suggested order: Fix in N04.

### N02-P1-02: `readJoints()` has a broken loop shape and unsafe joint-limit access

- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:264-288`
- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:296-325`
- Issue: A `for` loop wraps a `while` that mutates `link`, causing confusing repeated traversal behavior. Later code dereferences `link->parent_joint` and `joint->limits` without validating both.
- Impact: Bad or changed URDF structure can miscount joints or crash.
- Suggested order: Fix in N04 with focused kinematics tests.

### N02-P1-03: Raw KDL solvers leak and can be dereferenced uninitialized

- File: `baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/arm_kinematics.h:117-120`
- Allocation sites: `arm_kinematics.cpp:131`, `:176`, `:221-224`
- Dereference sites: `arm_kinematics.cpp:370`, `:372`, `:477`, `:518`
- File: `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp:275-276`, `:326`
- Issue: Raw solver pointers are never freed. `baxter_emulator` ignores `init_grav()` result and calls gravity torques every loop.
- Impact: Memory leaks and possible null/uninitialized dereference in gravity compensation.
- Suggested order: Fix in N05 after N04 correctness fixes.

### N02-P1-04: Gravity compensation shares one `num_joints` for both arms

- File: `baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/arm_kinematics.h:115`
- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:340-380`
- Issue: Left and right gravity chains share the same `num_joints` field and loop bounds.
- Impact: Different chains or failed initialization on one side can corrupt indexing for gravity output.
- Suggested order: Fix with N04/N05 kinematics work.

### N02-P1-05: Gazebo link-property service waits and calls are brittle

- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:73-83`
- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:117`, `:127`, `:162`, `:172`
- Issue: `waitForExistence()` has no timeout and service calls ignore return/success.
- Impact: Kinematics/gravity init can hang forever or use invalid service responses.
- Suggested order: Fix with gravity compensation hardening.

### N02-P1-06: Python 2 leftovers still break examples in Noetic

- File: `baxter_examples/scripts/joint_velocity_puppet.py:69`
- File: `baxter_examples/scripts/joint_velocity_wobbler.py:73`
- File: `baxter_examples/scripts/joint_trajectory_file_playback.py:35`, `:229`
- Issue: Uses `xrange` and removed `operator.div`.
- Impact: These examples crash under Python 3.
- Suggested order: Fix with Python 3 compatibility pass.

### N02-P1-07: Obsolete `xacro_jade` API is used

- File: `baxter_examples/scripts/send_urdf_fragment.py:35`, `:42-43`
- Issue: Imports `xacro_jade`, not the Noetic `xacro` API.
- Impact: URDF fragment example can fail immediately under Noetic.
- Suggested order: Fix with Python 3/deprecated API pass.

### N02-P1-08: Action-server scripts may run under the wrong interpreter

- File: `baxter_interface/CMakeLists.txt:41-44`
- File: `baxter_interface/scripts/gripper_action_server.py:1`
- File: `baxter_interface/scripts/head_action_server.py:1`
- Issue: Scripts are installed with `install(DIRECTORY ... USE_SOURCE_PERMISSIONS)`, so catkin does not rewrite shebangs. The script shebangs use `#!/usr/bin/env python`.
- Impact: On Noetic/Ubuntu 20.04, executable scripts may fail when `python` is absent or Python 2.
- Suggested order: Fix during Python 3 packaging cleanup.

### N02-P1-09: Dynamic reconfigure cfg and lower-priority Python shebangs are ambiguous

- Files: `baxter_interface/cfg/GripperActionServer.cfg:1`, `HeadActionServer.cfg:1`, `PositionFFJointTrajectoryActionServer.cfg:1`, `VelocityJointTrajectoryActionServer.cfg:1`, `PositionJointTrajectoryActionServer.cfg:1`
- File: `baxter_examples/cfg/JointSpringsExample.cfg:1`
- Files: `baxter_interface/setup.py:1`, `baxter_tools/setup.py:1`, `baxter_examples/setup.py:1`, `baxter_examples/src/baxter_external_devices/joystick.py:1`, `baxter_interface/src/joint_trajectory_action/minjerk.py:1`
- Issue: `#!/usr/bin/env python` remains in files used by build tools or installed modules.
- Impact: Lower-risk Python 3 compatibility hazard.
- Suggested order: Fix when normalizing Python packaging.

### N02-P1-10: Tool scripts mask original exceptions

- File: `baxter_tools/scripts/camera_control.py:63`
- File: `baxter_tools/scripts/enable_robot.py:79-80`
- File: `baxter_tools/scripts/calibrate_arm.py:104-105`
- File: `baxter_tools/scripts/tare.py:105-106`
- File: `baxter_tools/scripts/update_robot.py:169`, `:181`
- Issue: `camera_control.py` raises undefined `ROSTopicIOException`. Other scripts read `e.strerror`, which many exceptions do not have.
- Impact: Error handling can raise `NameError` or `AttributeError` and hide the real failure.
- Suggested order: Fix with Python 3 script cleanup.

### N02-P1-11: `update_robot.py` has scope and `None` bugs

- File: `baxter_tools/scripts/update_robot.py:195-212`
- Issue: `param_name` is local to `get_robot_version()` but used outside. `re.search(..., robot_version)` can receive `None`.
- Impact: Failed robot-version lookup can crash instead of reporting connectivity/version error.
- Suggested order: Fix with tool script cleanup.

### N02-P1-12: Trajectory action invalid-joint abort path continues

- File: `baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py:151-158`
- File: `baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py:410-415`
- Issue: `_get_trajectory_parameters()` aborts on invalid joint but caller ignores that and continues.
- Impact: Bad goals can proceed into trajectory generation after an abort.
- Suggested order: Fix with action-server tests.

### N02-P1-13: Head action server blocks inside its control loop

- File: `baxter_interface/src/head_action/head_action.py:81-82`
- File: `baxter_interface/src/head_action/head_action.py:110-123`
- Issue: The loop calls `Head.set_pan(... timeout=self._timeout)`, which can block while the loop is supposed to process feedback/preemption.
- Impact: Feedback and preemption can stall until timeout.
- Suggested order: Fix after action smoke tests.

### N02-P1-14: Camera integer math changed under Python 3

- File: `baxter_interface/src/baxter_interface/camera.py:258`
- File: `baxter_interface/src/baxter_interface/camera.py:277-278`
- Issue: `/` now returns float for window and limit values that look integer-like.
- Impact: Camera control values may be wrong type/range under Python 3.
- Suggested order: Fix with Python 3 compatibility pass.

### N02-P1-15: Realtime controllers ignore sub-controller init failures

- File: `baxter_simulator/baxter_sim_controllers/src/baxter_position_controller.cpp:101-102`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_velocity_controller.cpp:101-102`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_effort_controller.cpp:98-99`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_head_controller.cpp:90-91`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_gripper_controller.cpp:90-91`
- Issue: `init(robot, joint_nh)` return values are ignored.
- Impact: Controllers can report successful initialization while internal sub-controllers failed.
- Suggested order: Fix with controller smoke tests.

### N02-P1-16: Realtime controllers share command flags across callback/update threads unsafely

- File: `baxter_simulator/baxter_sim_controllers/src/baxter_position_controller.cpp:172-215`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_velocity_controller.cpp:175-215`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_head_controller.cpp:156-178`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_gripper_controller.cpp:158-209`
- Issue: Non-realtime callbacks write `new_command` flags while realtime `update()` reads/writes them.
- Impact: Data race and missed/duplicated command processing risk.
- Suggested order: Fix after functional blockers.

### N02-P1-17: Gripper controller assumes mimic and limits exist

- File: `baxter_simulator/baxter_sim_controllers/src/baxter_gripper_controller.cpp:96`
- File: `baxter_simulator/baxter_sim_controllers/src/baxter_gripper_controller.cpp:190-199`
- Issue: Dereferences mimic/limits without validation.
- Impact: Bad URDF/controller config can crash controller initialization or updates.
- Suggested order: Fix with controller safety issues.

### N02-P1-18: Effort controller uses non-standard variable-length array

- File: `baxter_simulator/baxter_sim_controllers/src/baxter_effort_controller.cpp:88`
- File: `baxter_simulator/baxter_sim_controllers/include/baxter_sim_controllers/baxter_effort_controller.h:78`
- Issue: Uses `std::string joint_name[n_joints_]`, which is not standard C++, and the header hard-codes publisher storage.
- Impact: Compiler portability and build reliability risk.
- Suggested order: Fix with simulator C++ cleanup.

### N02-P1-19: Simulator Dockerfile is obsolete and likely broken

- File: `baxter_simulator/Dockerfile:1`, `:4`, `:15-17`, `:29-31`, `:39-40`
- Issue: Uses Kinetic base image and branch, Python 2 package names, calls `wstool` before installing it, uses deprecated `MAINTAINER`, and replaces `/bin/sh` instead of Docker `SHELL`.
- Impact: Docker build is unlikely to work for Noetic.
- Suggested order: Replace in N07 after native tests exist.

### N02-P1-20: `.rosinstall` still tracks Kinetic/development branches

- File: `baxter_simulator/baxter_simulator.rosinstall:1-24`
- Issue: Pins `baxter_simulator` and `baxter_common` to `kinetic-devel`, others to `development`.
- Impact: Recreates a non-Noetic workspace if used by docs or Docker.
- Suggested order: Fix with Docker/documentation modernization.

## P2 Findings

### N02-P2-01: No automated tests or CI found

- Search result: no `test/`, `tests/`, `.rostest`, `catkin_add_gtest`, `catkin_add_nosetests`, `add_rostest`, `<test_depend>`, or `.github` workflow files found.
- Existing manual smoke scripts: `baxter_tools/scripts/smoke_test.py`, `baxter_tools/src/baxter_tools/smoketests.py`
- Impact: Later fixes have no automated regression safety net.
- Suggested order: Address in N03.

### N02-P2-02: `baxter_examples` installs omit scripts and depend on missing runtime packages

- File: `baxter_examples/CMakeLists.txt:39-64`
- Present but omitted: `baxter_examples/scripts/baxter_pnpcode.py`, `baxter_examples/scripts/baxter_pnpcode2.py`
- File: `baxter_examples/package.xml:24-44`
- Imports/launch use undeclared packages: `geometry_msgs`, `std_msgs`, `gazebo_msgs`, `joy`
- Example refs: `baxter_examples/scripts/ik_service_client.py:40-46`, `baxter_examples/scripts/baxter_pnpcode.py:11-20`, `baxter_examples/launch/joint_position_joystick.launch:10-11`
- Impact: Installed package may miss scripts and rosdep may not install required runtime packages.
- Suggested order: Fix after Python 3 example cleanup.

### N02-P2-03: Pick-and-place examples are broken/unfinished

- File: `baxter_examples/scripts/baxter_pnpcode.py:24-31`, `:111-160`
- File: `baxter_examples/scripts/baxter_pnpcode2.py:17`, `:41`, `:165-208`
- Issue: One script enables robot/moves at import time and has no `main()` guard. The other waits for `/solve_ik` and treats joint-angle dictionary entries as Cartesian pose values.
- Impact: Unsafe/unusable examples if installed or copied.
- Suggested order: Either remove from install/docs or fix as examples after core SDK tests.

### N02-P2-04: `baxter_tools` installs launch files under an extra `share` level

- File: `baxter_tools/CMakeLists.txt:35-38`
- Issue: Installs `share launch` to `${CATKIN_PACKAGE_SHARE_DESTINATION}/share`.
- Impact: Launch files can land under `share/share` or `share/launch` instead of the standard package launch path.
- Suggested order: Fix with package install cleanup.

### N02-P2-05: `baxter_tools` package metadata omits used packages

- File: `baxter_tools/package.xml:25-39`
- File: `baxter_tools/scripts/camera_control.py:37`, `:71`
- File: `baxter_tools/scripts/description_publisher.py:5`
- Issue: Uses `std_msgs` and `std_srvs` but package deps omit them.
- Impact: rosdep/package install can miss runtime dependencies.
- Suggested order: Fix with package manifests.

### N02-P2-06: `baxter_interface` package metadata omits NumPy

- File: `baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py:36`
- File: `baxter_interface/package.xml:25-42`
- Issue: Imports NumPy but package deps omit it.
- Impact: Fresh installs may fail to run trajectory action server.
- Suggested order: Fix with manifests.

### N02-P2-07: Simulator CMake exports/install paths are inconsistent

- File: `baxter_simulator/baxter_sim_hardware/CMakeLists.txt:21-33`, `:42`, `:52-55`
- File: `baxter_simulator/baxter_sim_io/CMakeLists.txt:24-29`, `:70`, `:78-81`
- File: `baxter_simulator/baxter_sim_controllers/CMakeLists.txt:23-31`, `:57-60`
- Issue: `baxter_sim_hardware` and `baxter_sim_io` export `${PROJECT_NAME}` libraries but build executables only. Several packages export include dirs but install headers into package share instead of include destination.
- Impact: Downstream package exports and installed workspaces can be broken.
- Suggested order: Fix with CMake/install cleanup.

### N02-P2-08: Simulator package dependencies are incomplete or inconsistent

- File: `baxter_simulator/baxter_gazebo/CMakeLists.txt:4-11`, `:19-25`
- File: `baxter_simulator/baxter_gazebo/src/baxter_gazebo_ros_control_plugin.cpp:45`
- Issue: Uses `controller_manager_msgs` headers but CMake does not list that component in `find_package(catkin ...)`.
- File: `baxter_simulator/baxter_sim_hardware/launch/baxter_sdk_control.launch:25`
- File: `baxter_simulator/baxter_sim_hardware/package.xml:28-49`
- Issue: Launch uses `baxter_moveit_config` but manifest lacks a runtime dependency.
- File: `baxter_simulator/baxter_sim_io/CMakeLists.txt:18`
- File: `baxter_simulator/baxter_sim_io/package.xml:13-19`
- Issue: Requires Qt5 but package metadata omits a system dependency.
- File: `baxter_simulator/baxter_sim_examples/scripts/ik_pick_and_place_demo.py:39`, `:45`, `:51`
- File: `baxter_simulator/baxter_sim_examples/package.xml:23-39`
- Issue: Uses `rospkg`, `geometry_msgs`, and `std_msgs`, but metadata is incomplete and declares `rospack` instead.
- Suggested order: Fix with manifests after basic tests expose imports.

### N02-P2-09: `baxter_sim_io` overwrites compiler flags

- File: `baxter_simulator/baxter_sim_io/CMakeLists.txt:12`
- Issue: `SET(CMAKE_CXX_FLAGS "-fPIC")` replaces existing compiler flags.
- Impact: Drops workspace/toolchain flags such as debug, warnings, sanitizers, or distro defaults.
- Suggested order: Fix with CMake cleanup.

### N02-P2-10: `baxter_sim_io` shares static ROS messages across GUI/ROS threads without locking

- File: `baxter_simulator/baxter_sim_io/src/qnode.cpp:183-238`
- File: `baxter_simulator/baxter_sim_io/src/baxter_io.cpp:110-308`
- Issue: ROS thread publishes/mutates static message state while Qt callbacks mutate the same fields.
- Impact: Data race and inconsistent simulated IO state.
- Suggested order: Fix only after higher-priority runtime blockers.

### N02-P2-11: Gazebo plugin caches mode before successful controller switch

- File: `baxter_simulator/baxter_gazebo/src/baxter_gazebo_ros_control_plugin.cpp:232-239`, `:251-258`
- Issue: Cached command mode is updated before confirming `switchController()` succeeded.
- Impact: If switch fails, later identical command can be ignored.
- Suggested order: Fix with Gazebo controller tests.

### N02-P2-12: Head controller processes commands only every 100 updates

- File: `baxter_simulator/baxter_sim_controllers/src/baxter_head_controller.cpp:140-146`
- Issue: `updateCommands()` is gated by `update_counter % 100 == 0`.
- Impact: Adds avoidable command latency.
- Suggested order: Fix with controller behavior cleanup.

### N02-P2-13: MoveIt Kinect/Xtion demo launches pass undeclared args

- File: `baxter_moveit_config/launch/demo_kinect.launch:20-31`
- File: `baxter_moveit_config/launch/demo_xtion.launch:20-31`
- File: `baxter_moveit_config/launch/demo_baxter.launch:3-7`, `:10-26`
- File: `baxter_moveit_config/launch/move_group.launch:28-36`
- Issue: Wrapper launches pass args such as `kinect`, `xtion`, `planner`, `rviz_config`, gripper args, and tip args to `demo_baxter.launch`, but those args are not declared or passed through.
- Impact: Optional demo launches can fail with unused arg errors and cannot enable sensors as intended.
- Suggested order: Fix with launch XML tests.

### N02-P2-14: MoveIt gripper/warehouse launch dependency mismatch

- File: `baxter_moveit_config/launch/baxter_grippers.launch:37`
- File: `baxter_moveit_config/package.xml:24-25`
- File: `baxter_moveit_config/launch/warehouse.launch:10`
- File: `baxter_moveit_config/launch/default_warehouse_db.launch:5`, `:13`
- Issue: `baxter_grippers.launch` unconditionally includes the default warehouse DB while `warehouse_ros_mongo` dependency is commented out. Default DB path is inside package share.
- Impact: Launch can fail on missing package or unwritable install path.
- Suggested order: Fix with MoveIt launch cleanup.

### N02-P2-15: URDF/SRDF gripper configuration is inconsistent

- File: `baxter_common/baxter_description/urdf/baxter.urdf.xacro:18-22`
- File: `baxter_common/baxter_description/urdf/left_end_effector.urdf.xacro:3-12`
- File: `baxter_common/baxter_description/urdf/right_end_effector.urdf.xacro:3-12`
- File: `baxter_moveit_config/launch/demo_dummy.launch:20-28`
- File: `baxter_moveit_config/config/baxter.srdf.xacro:10-21`
- Issue: URDF always includes electric end effectors, while some MoveIt launch defaults disable electric-gripper SRDF collision entries.
- Impact: MoveIt semantic model can miss collision disables for links present in URDF.
- Suggested order: Validate with xacro/check_urdf and MoveIt launch tests.

### N02-P2-16: Pneumatic gripper xacro has bad include and invalid geometry

- File: `baxter_common/rethink_ee_description/urdf/pneumatic_gripper/example_end_effector.urdf.xacro:3`
- File: `baxter_common/rethink_ee_description/urdf/pneumatic_gripper/pneumatic_gripper_base.xacro:17-20`
- Issue: Example includes `urdf/electric_gripper/rethink_pneumatic_gripper.xacro` instead of the pneumatic path. Collision geometry has two shapes inside one `<geometry>`, and the second `<box>` uses `length`/`radius` attributes.
- Impact: Pneumatic gripper xacro expansion/check_urdf can fail.
- Suggested order: Fix with description validation tests.

### N02-P2-17: Top-level Baxter xacro has misleading gazebo arg propagation

- File: `baxter_common/baxter_description/urdf/baxter.urdf.xacro:4-15`
- File: `baxter_common/baxter_description/urdf/pedestal/pedestal.xacro:4`, `:32-38`
- Issue: Top-level xacro appears to pass `gazebo` into includes with nested `<xacro:arg>`, while pedestal declares its own default.
- Impact: Gazebo world-fixing joint behavior can be misleading or wrong.
- Suggested order: Validate with xacro tests.

### N02-P2-18: Description package metadata omits xacro and simulator plugin dependencies

- File: `baxter_common/baxter_description/package.xml:23-24`
- File: `baxter_common/rethink_ee_description/package.xml:23`
- File: `baxter_common/baxter_description/urdf/baxter_base/baxter_base.gazebo.xacro:5`
- Issue: Description packages ship xacro files but do not declare `xacro`. `baxter_description` references `libbaxter_gazebo_ros_control.so` without declaring the simulator dependency.
- Impact: Standalone consumers can miss required runtime packages.
- Suggested order: Fix with manifest cleanup.

### N02-P2-19: Generated/copy artifacts are tracked and installed

- File: `baxter_moveit_config/config/gazebo_controllers (copy).yaml:1`
- File: `baxter_moveit_config/config/gazebo_baxter (copy).urdf:3`
- File: `baxter_moveit_config/CMakeLists.txt:9`
- File: `baxter_simulator/baxter_sim_examples/scripts/ik_pick_and_place_demo (copy).py:1`
- Issue: Divergent scratch/copy files exist. MoveIt config directory is installed wholesale, so copy files are installed too.
- Impact: Confusing installed artifacts and stale generated URDF/config content.
- Suggested order: Remove or exclude during cleanup; no new feature needed.

### N02-P2-20: Cafe table model config references missing SDF

- File: `baxter_simulator/baxter_sim_examples/models/cafe_table/model.config:6-7`
- Issue: References `model-1_4.sdf`, which was not found, while the available `model.sdf` declares a different version.
- Impact: Gazebo model lookup/version metadata can fail.
- Suggested order: Fix with simulator examples cleanup.

### N02-P2-21: Example model cleanup is disabled

- File: `baxter_simulator/baxter_sim_examples/scripts/ik_pick_and_place_demo.py:306`
- Issue: `rospy.on_shutdown(delete_gazebo_models)` is commented out.
- Impact: Demo-spawned Gazebo models can be left behind after shutdown.
- Suggested order: Fix after simulator launch tests.

## P3 Findings

### N02-P3-01: Deprecated `tf` API remains

- File: `baxter_simulator/baxter_sim_kinematics/include/baxter_sim_kinematics/arm_kinematics.h:39-41`, `:125`
- File: `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp:440-475`, `:505-532`
- File: `baxter_simulator/baxter_sim_kinematics/CMakeLists.txt:9-10`
- File: `baxter_simulator/baxter_sim_kinematics/package.xml:28`, `:38-40`
- File: `baxter_moveit_config/launch/baxter_moveit_sensor_manager.launch:12-13`, `:30-31`
- File: `baxter_moveit_config/launch/demo_dummy.launch:75-76`
- Issue: Uses tf1 C++ APIs and `tf` static transform publisher.
- Impact: Deprecated under Noetic; slated for N10 migration.
- Suggested order: Migrate after kinematics tests exist.

### N02-P3-02: `baxter_sim_hardware` declares unused `tf`

- File: `baxter_simulator/baxter_sim_hardware/CMakeLists.txt:12`
- File: `baxter_simulator/baxter_sim_hardware/package.xml:29`
- Issue: Declares `tf` but no matching usage was found in that package.
- Impact: Unnecessary dependency.
- Suggested order: Remove during tf2/dependency cleanup.

### N02-P3-03: MoveIt kinematics config uses deprecated attempts field

- File: `baxter_moveit_config/config/kinematics.yaml:5`, `:12`
- Issue: `kinematics_solver_attempts` commonly logs deprecation warnings in modern MoveIt.
- Impact: Warning/noise unless MoveIt still consumes it.
- Suggested order: Clean up during MoveIt config pass.

### N02-P3-04: CHOMP/STOMP config is incomplete

- File: `baxter_moveit_config/launch/chomp_planning_pipeline.launch:4`
- File: `baxter_moveit_config/config/stomp_planning.yaml:1`
- File: `baxter_moveit_config/package.xml:17-28`
- Issue: CHOMP launch exists without manifest dependency. STOMP config exists without matching launch found.
- Impact: Optional planners are partially wired.
- Suggested order: Either wire fully or remove stale config during docs/config cleanup.

### N02-P3-05: Metapackage repository URL typo

- File: `baxter_simulator/baxter_simulator/package.xml:15`
- Issue: URL path contains `baxter_simultor` typo.
- Impact: Documentation/package metadata quality issue.
- Suggested order: Fix with docs cleanup.

### N02-P3-06: Documentation references missing and stale setup helpers

- File: `README.md:5-12`, `:17-23`
- File: `baxter/baxter.sh:5`, `:21-31`, `:68-72`
- Issue: README references Carol's `run_baxter` wrapper, but no `run_baxter*` file was found. README says run `./baxter.sh` from workspace root, while the script is inside `baxter/` and explicitly says it must be moved. `baxter.sh` contains local machine values.
- Impact: New users will follow commands that do not work in this checkout.
- Suggested order: Fix in N11 after commands are validated.

### N02-P3-07: Package READMEs and external links are stale

- File: `baxter_interface/README.rst:10-16`
- File: `baxter_tools/README.rst:16-24`, `:34-42`
- File: `baxter_examples/README.rst:6-15`, `:24-46`
- File: `baxter_common/README.rst:6-15`
- File: `baxter_simulator/README.rst:9-14`
- File: `baxter_moveit_config/README.md:8-17`
- Issue: Docs mostly point to old Rethink wiki/repository pages and omit Noetic-specific install/test/Docker/launch caveats. Script lists omit newer local files such as `head_cam_display.py`, `baxter_pnpcode.py`, and `baxter_pnpcode2.py`.
- Impact: Poor onboarding and stale troubleshooting guidance.
- Suggested order: Fix in N11.

### N02-P3-08: Working-copy generated/ignored artifacts are present

- Present examples: `.cursor/`, `conductor/`, multiple `__pycache__/` directories under source packages.
- Relevant ignore rule: `.gitignore:14`
- Issue: Generated/untracked workspace artifacts are present in the checkout area.
- Impact: Can confuse audits and future packaging if accidentally tracked.
- Suggested order: Clean only if requested; N02 did not remove anything.

### N02-P3-09: Nested `.gitignore` rules can hide future source

- File: `baxter_interface/.gitignore:51-53`
- File: `baxter/.gitignore:50-52`
- File: `baxter_tools/.gitignore:50-52`
- File: `baxter_simulator/.gitignore:50-52`
- File: `baxter_examples/.gitignore:50-52`
- Issue: Overbroad nested ignore patterns may hide future files.
- Impact: Low-priority repository hygiene risk.
- Suggested order: Review during cleanup.

## Message Generation Review

- `baxter_common/baxter_core_msgs` has message/service generation wiring in `CMakeLists.txt:4-60` and runtime deps in `package.xml:25-33`. No blocking generation issue found.
- `baxter_common/baxter_maintenance_msgs` has message generation wiring in `CMakeLists.txt:4-26` and runtime deps in `package.xml:25-29`. No blocking generation issue found.
- No `.action` files were found in the audited packages.

## Package Coverage Summary

- `baxter_moveit_config`: Launch arg mismatches, warehouse dependency mismatch, stale/generated config, deprecated MoveIt config, missing optional deps, minimal docs.
- `baxter/baxter_sdk`: Metapackage only; no package-specific code issue found. Docs/setup script path issues noted through root README and `baxter.sh`.
- `baxter_interface`: Python 3 message serialization bugs, action-server behavior bugs, shebang/install issues, missing NumPy dep, no tests.
- `baxter_tools`: Bad install destination, missing deps, undefined exception, fragile exception handling, `update_robot.py` bugs, only manual smoke tests.
- `baxter_examples`: Python 2 leftovers, obsolete xacro API, broken pick-and-place scripts, missing deps, omitted installs, no tests.
- `baxter_sim_kinematics`: Multiple crash/correctness bugs, raw pointer leaks, deprecated tf1, no tests.
- `baxter_gazebo`: Invalid main launch XML, CMake dependency inconsistency, plugin mode-cache issue, no tests.
- `baxter_simulator`: Obsolete Dockerfile and Kinetic `.rosinstall`; metapackage URL typo.
- `baxter_sim_examples`: Missing deps, stale duplicate script, disabled cleanup, bad cafe table model config, no tests.
- `baxter_description`: xacro dependency gap, Gazebo plugin dependency gap, gazebo arg ambiguity, no URDF validation tests.
- `baxter_sim_hardware`: Emulator callback crash risks, ignores gravity init failure, CMake export/install issues, missing runtime deps, no tests.
- `baxter_sim_io`: Qt dep gap, CMake flag/export/install issues, GUI/ROS data race risk, no tests.
- `rethink_ee_description`: Pneumatic gripper include/geometry bugs, xacro dependency gap, no URDF validation tests.
- `baxter_maintenance_msgs`: Message generation appears wired; no tests.
- `baxter_sim_controllers`: Init failure handling, command flag races, gripper mimic/limit assumptions, non-standard VLA, no tests.
- `baxter_core_msgs`: Message/service generation appears wired; no tests.
- `baxter_common`: Metapackage only; docs are stale/minimal.

## N03 Test Targets Suggested By Audit

- Python import smoke tests for installable modules: `baxter_interface`, `baxter_dataflow`, `baxter_control`, `joint_trajectory_action`, `head_action`, `gripper_action`, `baxter_tools`, `baxter_examples`.
- Python syntax/compile smoke for scripts to catch `xrange`, `operator.div`, and bad imports without needing a robot.
- Catkin-level checks that every package has at least one test target.
- XML parse tests for all `.launch` files to catch invalid XML declarations.
- Xacro/check_urdf validation for Baxter, electric grippers, pneumatic gripper examples, and MoveIt generated robot descriptions.
- CMake/package metadata smoke via `catkin_lint` if already available; otherwise keep to simple package manifest/import checks.

## Decisions

- N02 did not fix any code, delete artifacts, run builds, or run launch files.
- N02 marks the audit complete because all 17 packages were inspected and findings are categorized with file/line references where available.

## Open Questions

- Should broken scratch/demo files such as `baxter_pnpcode.py`, `baxter_pnpcode2.py`, and `* (copy).*` be fixed, documented as unsupported, or removed from install/docs in a later cleanup step?
- Should optional Kinect/Xtion/warehouse/CHOMP/STOMP launch support be kept, or should unsupported optional paths be removed to keep the Noetic SDK minimal?

## Artifacts

- `~/baxter_noetic_ws/logs/N02_code_audit.log.md`
