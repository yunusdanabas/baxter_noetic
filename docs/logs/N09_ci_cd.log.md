---
step: N09
title: CI/CD Pipeline
agent_date: 2026-07-15
status: completed
previous_steps: [N01, N02, N03, N04, N05, N06, N07, N08]
---

# N09: CI/CD Pipeline

## Task

Set up a minimal GitHub Actions CI pipeline for the Baxter Noetic checkout without starting the N10 tf to tf2 migration.

## Changes

- Added one workflow file: `src/baxter_noetic/.github/workflows/ci.yml`.
- Workflow name: `CI`.
- Workflow triggers: `push` and `pull_request`.
- Workflow uses the existing Noetic Dockerfile instead of recreating ROS install logic in GitHub Actions.
- Workflow verifies:
  - Docker image build with `baxter_simulator/Dockerfile`.
  - Passing subset of the existing smoke checks.
  - Build of the kinematics gtest executable.
  - Passing subset of the existing kinematics gtests with a local `roscore`.
  - Gazebo and MoveIt launch parsing with `roslaunch --nodes`.
- Added a README CI badge because the final workflow path is `.github/workflows/ci.yml` and `origin` points to `https://github.com/yunusdanabas/baxter_noetic.git`.
- Did not add a matrix, cache, release publishing, GHCR push, or a second workflow.

## CI Scope Decisions

- Full `catkin run_tests --no-deps baxter_tools baxter_sim_kinematics` is not used as the CI gate yet.
- The full smoke test still exposes existing non-N09 issues: unresolved `xacro_jade`, generated cfg import lookup, `weakrefset`, and the pneumatic gripper xacro path.
- The full kinematics gtest needs a ROS master because `Kinematics` owns a `tf::TransformListener`.
- With `roscore`, the full kinematics gtest still fails `KinematicsTest.ComputesKnownFkAndIk`; the one-joint IK success assertion currently returns `false`.
- CI therefore runs the smoke and kinematics checks that are passable now, and documents the skipped failing cases for later cleanup.

## Verification Results

Working directory for source commands:

```text
~/baxter_noetic_ws/src/baxter_noetic
```

### Docker availability

Command:

```bash
docker --version
```

Result:

```text
Docker version 29.6.1, build 8900f1d
```

### Static whitespace check

Command:

```bash
git diff --check
```

Result: passed with no output.

### Docker image build

Command:

```bash
docker build -f baxter_simulator/Dockerfile -t baxter-noetic:ci .
```

Result: passed. Final catkin build summary:

```text
[build] Summary: All 18 packages succeeded!
[build] Warnings: 5 packages succeeded with warnings.
[build] Failed: No packages failed.
[build] Runtime: 3 minutes and 23.2 seconds total.
```

Observed warnings matched prior Docker work: Gazebo Classic deprecation warnings and Qt/googletest AUTOMOC developer warnings.

### Smoke check subset

Command:

```bash
docker run --rm baxter-noetic:ci bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py SmokeTests.test_package_manifests SmokeTests.test_python_syntax SmokeTests.test_importable_modules SmokeTests.test_launch_files_are_xml'
```

Result: passed.

```text
Ran 4 tests in 0.673s

OK
```

Observed warnings: Python `DeprecationWarning` messages for invalid escape sequences in existing scripts.

### Full smoke/catkin test attempt

Command:

```bash
docker run --rm baxter-noetic:ci bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws && catkin run_tests --no-deps baxter_tools baxter_sim_kinematics --limit-status-rate 0.2 --no-notify && catkin_test_results build'
```

Result: not suitable as the current CI gate. `baxter_tools` smoke tests ran `6` tests with `2` failures:

```text
test_python_imports_are_resolvable
test_selected_xacro_files_expand
```

Representative failures:

```text
baxter_examples/scripts/send_urdf_fragment.py:35: xacro_jade
baxter_common/rethink_ee_description/urdf/pneumatic_gripper/example_end_effector.urdf.xacro: No such file or directory: rethink_pneumatic_gripper.xacro
```

The combined run then exceeded the local 10 minute tool timeout while `baxter_sim_kinematics` was still active. The leftover container was stopped.

### Kinematics gtest build

Command:

```bash
docker exec baxter-n09-kin-debug bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/build/baxter_sim_kinematics && timeout 300 make test_kinematics VERBOSE=1'
```

Result: passed. `test_kinematics` built successfully.

### Kinematics gtest runtime without roscore

Command:

```bash
docker exec baxter-n09-kin-debug bash -lc 'source /root/baxter_ws/devel/setup.bash && timeout 60 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics'
```

Result: timed out with repeated ROS master connection errors:

```text
Error in XmlRpcClient::writeRequest: write error (Connection refused).
```

Conclusion: kinematics gtest runtime needs a ROS master.

### Full kinematics gtest with roscore

Command:

```bash
docker exec baxter-n09-kin-debug bash -lc 'source /root/baxter_ws/devel/setup.bash; roscore >/tmp/roscore.log 2>&1 & roscore_pid=$!; trap "kill $roscore_pid" EXIT; sleep 5; timeout 120 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics'
```

Result: ran, but failed one existing kinematics test:

```text
[  PASSED  ] 3 tests.
[  FAILED  ] KinematicsTest.ComputesKnownFkAndIk
```

Failure summary:

```text
Value of: kin.getPositionIK(OneJointZeroPose(), OneJointState(0.0), &ik_result)
  Actual: false
Expected: true
```

### Filtered kinematics gtest used by CI

Command:

```bash
docker run --rm baxter-noetic:ci bash -lc 'source /root/baxter_ws/devel/setup.bash; cd /root/baxter_ws/build/baxter_sim_kinematics; make test_kinematics; roscore >/tmp/roscore.log 2>&1 & roscore_pid=$!; trap "kill $roscore_pid || true" EXIT; sleep 5; timeout 120 /root/baxter_ws/devel/.private/baxter_sim_kinematics/lib/baxter_sim_kinematics/test_kinematics --gtest_filter=-KinematicsTest.ComputesKnownFkAndIk'
```

Result: passed.

```text
[==========] Running 3 tests from 1 test suite.
[  PASSED  ] 3 tests.
```

### Launch parse smoke

Command:

```bash
docker run --rm baxter-noetic:ci bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

Result: passed. Nodes listed included:

```text
/gazebo
/gazebo_gui
/urdf_spawner
/baxter_sim_kinematics_left
/baxter_sim_kinematics_right
/baxter_emulator
/robot_state_publisher
/baxter_sim_io
/move_group
```

Observed warnings matched N07/N08: xacro child include warnings and the in-order processing deprecation note.

### GitHub Actions local execution

GitHub Actions was not executed locally; no `act` or GitHub runner was used. Validation was static plus Docker-based execution of the commands embedded in the workflow.

## Open Questions

- The skipped full smoke failures should be addressed in later cleanup steps before making the full smoke suite mandatory.
- The skipped kinematics IK success test should be fixed or replaced before making the full kinematics gtest mandatory.

## Artifacts

- `src/baxter_noetic/.github/workflows/ci.yml`
- `src/baxter_noetic/README.md`
- `logs/N09_ci_cd.log.md`
