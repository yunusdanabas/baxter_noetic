# Baxter Noetic Improvements — Master Plan

## Overview

Improvement plan for the existing Baxter ROS Noetic SDK. This plan focuses on bug fixes (especially kinematics), modern Docker setup, CI/CD, and code quality improvements. The goal is to make the Noetic codebase more robust and easier for students to use while the ROS 2 Jazzy version is being developed separately.

**Scope boundary:** Bug fixes, testing, Docker modernization, CI/CD, and code quality. No ROS 2 migration (that's a separate project). No new features unless they directly support the improvements.

**Starting point:** Existing Noetic SDK at `~/Yunus Portfolio/ROS1/catkin_ws/src/baxter_noetic/` (or reference copy at `~/baxter_ros2_jazzy/plan/baxter_noetic_ref/`). Active workspace: `~/baxter_noetic_ws/`, source checkout: `~/baxter_noetic_ws/src/baxter_noetic/`.

---

## Steps

### N01: Create Working Copy & Git Initialization

- **Status:** `completed`
- **Type:** Setup
- **Description:** Create a catkin workspace at `~/baxter_noetic_ws/`, clone the Noetic SDK into `src/baxter_noetic/`, preserve git history, carry local source changes, and move planning docs into the workspace root.
- **Gate:** `~/baxter_noetic_ws/src/baxter_noetic/` exists with all 17 packages, git initialized with existing history, and planning docs moved into `~/baxter_noetic_ws/`.
- **Log:** `logs/N01_working_copy.log.md`

### N02: Detailed Code Audit

- **Status:** `completed`
- **Type:** Research
- **Description:** Perform comprehensive code audit expanding on initial findings. Document all bugs, code quality issues, Python 2/3 compatibility issues, deprecated API usage, and missing tests. Create prioritized issue list.
- **Gate:** Audit document complete with all identified issues categorized and prioritized.
- **Log:** `logs/N02_code_audit.log.md`

### N03: Testing Infrastructure

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Set up minimal test infrastructure: one shared smoke test target covering package manifests, Python syntax/imports, launch XML, and selected xacro expansion.
- **Gate:** Smoke test infrastructure is present and verification results are documented; full `catkin run_tests` execution requires a ROS Noetic catkin environment.
- **Log:** `logs/N03_testing_infrastructure.log.md`

### N04: Fix Kinematics Bugs

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Fix all 8 identified bugs in `baxter_sim_kinematics`:
  1. `loadModel()` return value bug
  2. `FilterJointState()` buffer overflow
  3. `FKCalc()` ignoring failures
  4. Broken loop in `readJoints()`
  5. Copy-paste error message
  6. Shared `num_joints` variable
  7. Unnecessary delay in IK service
  8. Add bounds checking to IK seed validation
- **Gate:** All bugs fixed, kinematics package builds, existing functionality preserved.
- **Note (2026-07-16 audit):** build gate unverifiable at sign-off (no Noetic on host); first successful build N07. One IK bug (`getPositionIK` output sizing) survived until N09.5. Verified green retroactively — see `logs/AUDIT_2026-07-16.log.md`.
- **Log:** `logs/N04_kinematics_bugs.log.md`

### N05: Memory Leak Fixes

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Fix memory leaks in `baxter_sim_kinematics` by replacing raw `new`/`delete` with `std::unique_ptr` for all KDL solvers: `fk_solver`, `ik_solver_vel`, `ik_solver_pos`, `gravity_solver_l`, `gravity_solver_r`. Add proper destructor.
- **Gate:** No memory leaks (verify with valgrind or address sanitizer), package builds and runs.
- **Note (2026-07-16 audit):** valgrind first run by the audit (clean: 0 leaks, 0 errors); gate evidence did not exist at sign-off. See N05 log addendum.
- **Log:** `logs/N05_memory_leaks.log.md`

### N06: Kinematics Unit Tests

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Add comprehensive unit tests for kinematics: FK/IK round-trip tests, known pose validation, joint limit tests, gravity compensation tests. Use `gtest` framework.
- **Gate:** Unit tests pass, coverage includes all major kinematics functions.
- **Note (2026-07-16 audit):** tests never executed at sign-off; first run (N09) had 1 failure, green since N09.5. Audit re-confirmed 4/4 unfiltered.
- **Log:** `logs/N06_kinematics_tests.log.md`

### N07: Modern Docker Setup

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Replace outdated Kinetic Dockerfile with modern Noetic-based Dockerfile. Base image: `ros:noetic-ros-base-focal`. Include Gazebo support, all dependencies, workspace build. Add `.dockerignore`.
- **Gate:** `docker build` succeeds, simulator launch parsing works in the image, all packages build.
- **Log:** `logs/N07_docker_setup.log.md`

### N08: Docker Compose

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Create `docker-compose.yml` with services: `roscore`, `baxter_sim` (Gazebo), `moveit`, `rviz`. Add minimal GPU/display support configuration.
- **Gate:** Compose file is present, non-GUI compose checks pass, and GUI/GPU runtime blocker is documented.
- **Log:** `logs/N08_docker_compose.log.md`

### N09: CI/CD Pipeline

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Set up GitHub Actions CI/CD: build workflow, test workflow, Docker build workflow. Use `industrial_ci` or custom workflow. Add status badges to README.
- **Gate:** CI runs on push/PR, builds successfully, runs tests, builds Docker image.
- **Log:** `logs/N09_ci_cd.log.md`

### N09.5: CI Hardening & Test Blocker Cleanup

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Fix the known blockers documented during N09 so CI can run the full smoke and kinematics checks instead of filtered subsets. Address `xacro_jade`, generated cfg import handling, `weakrefset`, pneumatic gripper xacro path, and the failing one-joint IK gtest/setup.
- **Gate:** Full Docker-backed smoke and kinematics checks either pass unfiltered or any remaining exclusions are narrowly justified; CI workflow is updated to remove obsolete filters where possible.
- **Log:** `logs/N09_5_ci_hardening.log.md`

### N10: tf → tf2 Migration

- **Status:** `completed`
- **Type:** Implementation
- **Description:** Migrate from deprecated `tf` library to `tf2_ros` in `baxter_sim_kinematics` and any other packages using `tf`. Update includes, API calls, and launch files.
- **Gate:** No `tf` dependencies remain, all packages build, kinematics tests still pass.
- **Log:** `logs/N10_tf2_migration.log.md`

### N11: Documentation Overhaul

- **Status:** `completed`
- **Type:** Documentation
- **Description:** Overhaul Markdown documentation for the current Noetic checkout: native setup, Docker, Compose, tests/CI, simulator basics, MoveIt basics, and troubleshooting.
- **Gate:** Documentation changes are present, stale quick-start/package docs are updated, and documentation checks are recorded.
- **Log:** `logs/N11_documentation.log.md`

### N12: Fix Remaining Non-Kinematics Bugs (P0/P1)

- **Status:** `completed`
- **Type:** Implementation
- **Description:** N04 only fixed the 8 bugs inside `baxter_sim_kinematics`. N02's original audit found P0 (crash-severity) and P1 bugs elsewhere in the SDK that were never in scope for N04–N11. Fix: `limb.py` dict-view assigned to ROS message fields (P0-05), float rate published to `UInt16` (P0-06), `baxter_emulator.cpp` crashes on sparse ROS messages (P0-07), `xrange`/`operator.div` in example scripts (P1-06), trajectory abort-continue bug (P1-12), blocking `set_pan` call in head action server (P1-13), ignored controller `init()` return values (P1-15). Lower-effort items (stale shebangs, masked exceptions, `update_robot.py` `None` bug, non-standard VLA, stale `.rosinstall`) ride along only if trivial.
- **Gate:** P0 items fixed and verified by direct exercise, not just inspection; full smoke suite and kinematics gtest still pass unfiltered; no regressions in already-fixed kinematics code.
- **Log:** `logs/N12_non_kinematics_bugs.log.md`
- **Findings source:** `logs/AUDIT_2026-07-16.log.md` — "Addendum: non-kinematics scope gap" section.

---

## Final Step: Release (Pre-built Docker Images & Publish)

- **Status:** `pending — not started, awaiting explicit user approval`
- **Type:** Release
- **Description:** Set up automated Docker image builds to GitHub Container Registry (GHCR). Tag a release (v1.0.0). Create release notes. Update documentation with `docker pull` instructions.
- **Gate:** Docker images available on GHCR, release tagged, documentation updated.
- **Log:** `logs/RELEASE.log.md`
- **Note:** Deliberately kept outside the numbered N01–N12 sequence, not a step in the chain. This runs only after the user has independently reviewed the codebase and explicitly says to start it — no agent starts this on its own initiative, regardless of what else is marked `completed`.

---

## Dependencies Between Steps

```text
N01 → N02 → N03 → N04 → N05 → N06 → N07 → N08 → N09 → N09.5 → N10 → N11 → N12
```

All numbered steps are sequential. Each step builds on the previous. **Release is not part of this chain** — it starts only on explicit user sign-off, whenever that happens.

## Rules

- Preserve existing functionality — no breaking changes
- All fixes must include tests where applicable
- Docker images must work on both Linux and WSL2
- CI must pass before marking step complete
- Documentation must be updated with each change
- No ROS 2 migration (separate project)
- Keep changes minimal and focused

## Ponytail Mode (Active During All Implementation Steps)

All code-writing agents follow ponytail principles:

1. **Shortest working diff wins** — no unrequested abstractions, no scaffolding "for later"
2. **Stdlib first** — use existing ROS/KDL/Python stdlib before adding dependencies
3. **Deletion over addition** — if a bug fix removes code, that's better than adding code
4. **No comments unless asked** — code should be self-documenting
5. **One-line fixes when possible** — don't wrap a one-liner in a class
6. **`ponytail:` comments for deliberate shortcuts** — mark known ceilings: `// ponytail: global lock, per-joint locks if throughput matters`
7. **No over-engineering** — no factory for one product, no interface with one implementation
8. **Boring over clever** — the simplest fix that works is the right fix

### Ponytail Rules Per Step

| Step | Application |
|------|-------------|
| N04 (bugs) | Minimal diff per bug. Don't refactor surrounding code. Fix the bug, move on. |
| N05 (memory) | `std::unique_ptr` replacement only. Don't redesign the class hierarchy. |
| N06 (tests) | gtest only. No test fixtures, no mocking framework, no parameterized tests unless needed. |
| N07 (Docker) | Single Dockerfile, multi-stage. No custom entrypoint scripts unless required. |
| N08 (compose) | Minimal services. No optional profiles, no health checks unless something breaks. |
| N09 (CI) | One workflow file. No matrix builds, no caching optimization unless build is slow. |
| N09.5 (CI hardening) | Fix only blockers documented by N09. Remove filters only after full checks pass. |
| N10 (tf2) | Mechanical replacement. Don't refactor the node structure. |
| N11 (docs) | Markdown only. No Sphinx, no doc generators. |
| N12 (non-kinematics bugs) | Same discipline as N04: minimal diff per bug, don't refactor surrounding code. |
| Release (final step) | GitHub release + GHCR. No custom release scripts. Does not start without explicit user approval. |

### What Ponytail Does NOT Simplify

- Input validation at trust boundaries (IK seed validation stays)
- Error handling that prevents silent failures (FK failure check stays)
- Safety measures (robot enable/disable logic untouched)
- The actual bug fixes (fix them properly, just minimally)

---

## Success Criteria

1. All 8 kinematics bugs fixed
2. No memory leaks in C++ code
3. Unit test coverage for kinematics
4. Modern Docker setup (Noetic + Gazebo)
5. docker-compose for easy startup
6. CI/CD pipeline running
7. tf2 migration complete
8. Comprehensive documentation
9. Remaining non-kinematics P0/P1 bugs resolved (N12)
10. Pre-built Docker images on GHCR — Final Step, user-gated, separate from the numbered pipeline
