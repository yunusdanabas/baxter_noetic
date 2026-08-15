---
step: N09.5
title: CI Hardening & Test Blocker Cleanup
agent_date: 2026-07-15
status: completed
previous_steps: [N01, N02, N03, N04, N05, N06, N07, N08, N09]
---

# N09.5: CI Hardening & Test Blocker Cleanup

## Task

Fix the known N09 CI/test blockers without starting the N10 `tf` to `tf2` migration.

## Changes

- Replaced obsolete `xacro_jade` usage in `baxter_examples/scripts/send_urdf_fragment.py` with Noetic `xacro.process_file(...)`.
- Changed `baxter_dataflow.signals` to import `WeakSet` directly from Python 3 `weakref` instead of falling back to the bundled Python 2-era `weakrefset` module.
- Fixed the pneumatic gripper example xacro include path to use `urdf/pneumatic_gripper/rethink_pneumatic_gripper.xacro`.
- Fixed `baxter_examples/cfg/JointSpringsExample.cfg` to generate dynamic-reconfigure output for package `baxter_examples`, matching `from baxter_examples.cfg import ...`.
- Narrowed the static import smoke checker to ignore only generated dynamic-reconfigure modules: `baxter_examples.cfg` and `baxter_interface.cfg`.
- Fixed `Kinematics::getPositionIK()` by sizing `jnt_pos_out` before calling KDL `CartToJnt()`.
- Removed N09 CI filters so `.github/workflows/ci.yml` now runs full smoke tests and the full kinematics gtest.

## Findings

- `baxter_interface.cfg` generation was already correctly configured and imports in the built Docker image.
- `baxter_examples.cfg` needed the package-name fix in `JointSpringsExample.cfg`; after that, generated cfg imports passed in Docker.
- `KinematicsTest.ComputesKnownFkAndIk` was exposing a real production IK setup bug, not an unsolvable test pose: the KDL IK output array was never resized.
- No `tf` to `tf2` migration was done.

## Verification Results

Working directory for source commands:

```text
~/baxter_noetic_ws/src/baxter_noetic
```

### Docker build

Command:

```bash
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:n09-5 .
```

Result: passed. Final catkin build summary:

```text
[build] Summary: All 18 packages succeeded!
[build] Warnings: 5 packages succeeded with warnings.
[build] Failed: No packages failed.
[build] Runtime: 4 minutes and 0.3 seconds total.
```

Warnings matched prior Docker work: Gazebo Classic deprecation warnings and Qt/googletest AUTOMOC developer warnings.

### Generated cfg import probe

Command:

```bash
docker run --rm baxter-noetic:n09-5 bash -lc 'source /root/baxter_ws/install/setup.bash && python3 -c "from baxter_examples.cfg import JointSpringsExampleConfig; from baxter_interface.cfg import GripperActionServerConfig, HeadActionServerConfig, PositionJointTrajectoryActionServerConfig, VelocityJointTrajectoryActionServerConfig, PositionFFJointTrajectoryActionServerConfig; print(\"cfg imports ok\")"'
```

Result: passed.

```text
cfg imports ok
```

### Full smoke check

Command:

```bash
docker run --rm baxter-noetic:n09-5 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

Result: passed.

```text
Ran 6 tests in 4.160s

OK
```

Observed warnings: Python `DeprecationWarning` messages for existing invalid escape sequences in `robot_enable.py` and `update_robot.py`.

### Full kinematics gtest

Command:

```bash
docker run --rm baxter-noetic:n09-5 bash -lc '
            source /root/baxter_ws/devel/setup.bash
            cd /root/baxter_ws/build/baxter_sim_kinematics
            make test_kinematics
            roscore >/tmp/roscore.log 2>&1 &
            roscore_pid=$!
            trap "kill $roscore_pid || true" EXIT
            sleep 5
            timeout 120 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics
          '
```

Result: passed.

```text
[==========] Running 4 tests from 1 test suite.
[  PASSED  ] 4 tests.
```

Observed warnings: expected ROS error/fatal logs from the negative missing-limits test, plus `XmlRpcServer::acceptConnection` warnings while the local test node exits. They did not fail the gtest.

### Launch parse checks

Command:

```bash
docker run --rm baxter-noetic:n09-5 bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

Result: passed. Nodes listed included `/gazebo`, `/gazebo_gui`, `/base_to_world`, `/urdf_spawner`, `/baxter_sim_kinematics_left`, `/baxter_sim_kinematics_right`, `/baxter_emulator`, `/robot_state_publisher`, `/baxter_sim_io`, and `/move_group`.

Observed warnings matched prior runs: xacro child include warnings and the in-order processing deprecation note.

### Static whitespace check

Command:

```bash
git diff --check
```

Result: passed with no output.

## Open Questions

- None for N09.5.

## Artifacts

- `src/baxter_noetic/.github/workflows/ci.yml`
- `src/baxter_noetic/test/test_smoke.py`
- `src/baxter_noetic/baxter_examples/scripts/send_urdf_fragment.py`
- `src/baxter_noetic/baxter_interface/src/baxter_dataflow/signals.py`
- `src/baxter_noetic/baxter_common/rethink_ee_description/urdf/pneumatic_gripper/example_end_effector.urdf.xacro`
- `src/baxter_noetic/baxter_examples/cfg/JointSpringsExample.cfg`
- `src/baxter_noetic/baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp`
- `logs/N09_5_ci_hardening.log.md`
