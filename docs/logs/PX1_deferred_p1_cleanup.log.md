# PX1: Deferred P1 Cleanup

Date: 2026-07-16

Status: completed

## Scope

Fixed only the cheap P1 cleanup items deferred by N12. No release work was started: no tags, GHCR publish, release notes, or published-image docs.

## Changes

- Updated remaining exact `#!/usr/bin/env python` shebangs to `#!/usr/bin/env python3` in executable scripts, setup files, and dynamic-reconfigure cfg files.
- Fixed masked exceptions in `baxter_tools/scripts/*.py` by replacing undefined `ROSTopicIOException` with `rospy.ROSException` and replacing `e.strerror` reads with `str(e)`.
- Fixed `baxter_tools/scripts/update_robot.py` so missing `rethink/software_version` returns before `re.search(pattern, robot_version)` can receive `None`.

## Rosinstall Review

- `baxter/baxter_sdk.rosinstall` remains unchanged. It pins vendor `release-1.2.0` refs and is already documented in `baxter/README.md` as historical; the current workspace vendors the packages directly.
- `baxter_simulator/baxter_simulator.rosinstall` remains unchanged. It is stale (`kinetic-devel` and `development` refs), but upstream Rethink repos do not provide a clear Noetic branch target. Changing these refs would be an unverified package-selection change, not a cheap P1 bug fix.

## Verification

Command:

```bash
git diff --check
```

Result: passed.

Command:

```bash
PYTHONPYCACHEPREFIX=/tmp/opencode/pycache python3 -m py_compile <changed Python and cfg files>
```

Result: passed.

Command:

```bash
docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

Result: passed.

```text
Ran 6 tests in 4.053s
OK
```

Observed existing warnings: Python `DeprecationWarning` for invalid escape sequences in `robot_enable.py` and `update_robot.py` inside the `baxter-noetic:n12` image.

## Deferred

- No `.rosinstall` refs were changed because there is no verified Noetic upstream target.
- Lower-priority warning cleanup remains outside PX1 scope.

## Recheck

Second pass requested after implementation:

- Exact search for `#!/usr/bin/env python`: no matches.
- Search for `e.strerror` and `ROSTopicIOException` in `baxter_tools/scripts/*.py`: no matches.
- Remaining `raise` sites in `baxter_tools/scripts/*.py` use defined exceptions: `rospy.ROSException` and `OSError`.
- `update_robot.py` now returns before `re.search(pattern, robot_version)` when the rosparam is missing.
- `git diff --check`: passed.
- `python3 -m py_compile <changed Python and cfg files>`: passed.
- Docker smoke suite: passed, `Ran 6 tests in 3.035s`, `OK`.
