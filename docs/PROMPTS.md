# Agent Prompts

This file contains prompts for each sequential agent in the multi-agent workflow. Each agent reads this file, executes its step, then appends the next prompt.

## Global: Ponytail Mode

All agents operate in ponytail mode. This means:

- Shortest working diff wins
- No unrequested abstractions or refactoring
- Stdlib and existing dependencies first
- Deletion over addition
- No comments unless the code is genuinely unclear
- Mark deliberate shortcuts with `// ponytail: <reason>`
- If the fix is one line, it's one line — don't wrap it

Do NOT:
- Refactor surrounding code while fixing a bug
- Add new dependencies for what existing ones handle
- Create interfaces, factories, or abstractions for single implementations
- Add scaffolding for "future use"
- Write comments explaining what the code does (only why, if non-obvious)

---

## N01: Create Working Copy & Git Initialization

```
You are agent N01 for the Baxter Noetic improvements project.

## Your Task

Create a working copy of the Baxter Noetic SDK and initialize it as a git repository.

## Context

This is the first step in a 12-step improvement plan for the Baxter ROS Noetic SDK. The goal is to fix bugs, add tests, modernize Docker, and improve code quality. You are setting up the foundation for all subsequent work.

## Source Location

The Noetic SDK reference copy is at:
  ~/baxter_ros2_jazzy/plan/baxter_noetic_ref/

The original working copy (if it exists) is at:
  ~/Yunus Portfolio/ROS1/catkin_ws/src/baxter_noetic/

## What To Do

1. **Check for original git history:**
   - Check if `~/Yunus Portfolio/ROS1/catkin_ws/src/baxter_noetic/` has a `.git` directory
   - If yes, clone it to `~/baxter_noetic_ws/src/baxter_noetic/` to preserve history
   - If no, copy the reference copy to `~/baxter_noetic_ws/src/baxter_noetic/`

2. **Create working copy:**
   ```bash
   # If original has git history:
   cd ~/Yunus\ Portfolio/ROS1/catkin_ws/src/baxter_noetic/
   mkdir -p ~/baxter_noetic_ws/src
   git clone . ~/baxter_noetic_ws/src/baxter_noetic/
   
   # If no git history:
   mkdir -p ~/baxter_noetic_ws/src
   cp -r ~/baxter_ros2_jazzy/plan/baxter_noetic_ref/ ~/baxter_noetic_ws/src/baxter_noetic/
   cd ~/baxter_noetic_ws/src/baxter_noetic/
   git init
   ```

3. **Set up .gitignore:**
   Create `.gitignore` with:
   ```
   # Build artifacts
   build/
   devel/
   install/
   logs/
   .catkin_workspace
   .catkin_tools/
   
   # IDE
   .vscode/
   .idea/
   *.swp
   *.swo
   
   # Python
   __pycache__/
   *.pyc
   *.pyo
   *.egg-info/
   dist/
   
   # OS
   .DS_Store
   Thumbs.db
   
   # Docker
   .docker/
   
   # Temporary
   *.log
   *.tmp
   ```

4. **Create initial commit:**
   ```bash
   git add .
   git commit -m "Initial commit: Baxter Noetic SDK"
   ```

5. **Verify structure:**
   - Confirm all 17 packages are present
   - Confirm git is initialized
   - Confirm .gitignore is working

## Output

Write your findings to:
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md

Include:
- Source used (original vs reference copy)
- Git history status (preserved vs fresh)
- Package count verification
- Any issues encountered

Then update the step status in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Change N01 status from `pending` to `completed`.

Then append the N02 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Rules

- Do not modify any source code yet
- Do not build the workspace yet
- Only create the working copy and initialize git
- If original has git history, preserve it
- If no git history, start fresh

## Ponytail Mode

This project uses ponytail mode. For N01 (setup), this means:
- Minimal .gitignore (only what's needed)
- No README badges, no CI config, no pre-commit hooks yet
- Just the working copy, git init, and .gitignore
- Don't add CONTRIBUTING.md, CODE_OF_CONDUCT.md, or other boilerplate
```

---

## N02: Detailed Code Audit

```
You are agent N02 for the Baxter Noetic improvements project.

## Your Task

Perform a detailed code audit of the Noetic source checkout. Do not modify source code.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md

## What To Do

1. Inspect all 17 packages for bugs, Python 2/3 issues, deprecated APIs, missing tests, Docker/build issues, and documentation gaps.
2. Prioritize findings by severity and implementation order.
3. Record exact file paths and line references where possible.
4. Do not build, refactor, or fix code in this step.

## Output

Write findings to:
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md

Then update N02 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N02 `completed` if the audit log is complete.

Then append the N03 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Audit only. No speculative fixes, no scaffolding, no new files except the N02 log and next prompt.
```

---

## N03: Testing Infrastructure

```
You are agent N03 for the Baxter Noetic improvements project.

## Your Task

Set up minimal testing infrastructure for the Noetic source checkout. Do not fix audited source bugs in this step except where a tiny test harness issue blocks the tests themselves.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md

## What To Do

1. Add the smallest useful catkin test infrastructure for the 17 packages.
2. Add smoke tests that catch import/syntax/package/launch/XML/xacro failures without requiring a physical Baxter.
3. Prefer one small test entry point per package or package group over large test suites.
4. Add a minimal test runner script only if it materially reduces repeated commands.
5. Run the smallest feasible verification command and record results.
6. Do not refactor or fix the N02 findings yet; failing tests can document known failures.

## Output

Write findings and test results to:
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md

Then update N03 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N03 `completed` if the test infrastructure is present in the working tree and the verification results are documented.

Then append the N04 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Minimal tests only. No fixtures, no mocks, no broad framework, no source fixes unless the test harness itself cannot run. Prefer boring catkin/pytest/unittest basics already available in ROS Noetic.
```

---

## N04: Fix Kinematics Bugs

```
You are agent N04 for the Baxter Noetic improvements project.

## Your Task

Fix the audited kinematics correctness bugs in `baxter_sim_kinematics`. Do not do the N05 memory-leak refactor yet.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md

## What To Do

1. Fix the `loadModel()` bool return bug in `baxter_simulator/baxter_sim_kinematics/src/arm_kinematics.cpp`.
2. Add bounds checks for incomplete joint states in `FilterJointState()` and FK paths.
3. Make FK code honor and report FK failures instead of publishing invalid results.
4. Fix the broken `readJoints()` traversal/loop shape and unsafe joint-limit access.
5. Fix the copy-paste error message noted in N02.
6. Split shared gravity joint-count state so left/right arms cannot corrupt each other.
7. Remove the unnecessary IK service delay.
8. Add IK seed vector-size validation before indexing request seed data.
9. Run the smallest feasible build/test command and record results. If this machine still lacks ROS Noetic, document that blocker and run any direct static checks that do work.

## Output

Write findings and test results to:
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md

Then update N04 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N04 `completed` if the kinematics fixes are present in the working tree and verification results are documented.

Then append the N05 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Minimal bug fixes only. No broad refactors, no memory-ownership rewrite, no tf2 migration, no new test framework. Keep each fix local to the audited failure.
```

---

(N05-N12 prompts will be appended by each agent)

---

## N05: Memory Leak Fixes

```
You are agent N05 for the Baxter Noetic improvements project.

## Your Task

Fix the audited memory leaks in `baxter_sim_kinematics`. Do not start N06 kinematics tests yet.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md

## What To Do

1. Fix the raw KDL solver ownership leaks in `baxter_simulator/baxter_sim_kinematics`.
2. Replace only these solver pointers with `std::unique_ptr`: `fk_solver`, `ik_solver_vel`, `ik_solver_pos`, `gravity_solver_l`, `gravity_solver_r`.
3. Keep call sites behavior-equivalent; do not redesign the class.
4. Add only the destructor or initialization needed for safe ownership.
5. Run the smallest feasible build/test command and record results. If this machine still lacks ROS Noetic, document that blocker and run any direct static checks that do work.

## Output

Write findings and test results to:
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md

Then update N05 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N05 `completed` if the memory-ownership fixes are present in the working tree and verification results are documented.

Then append the N06 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Minimal memory fix only. No kinematics redesign, no tf2 migration, no new tests beyond the smallest check needed. Keep raw pointer replacement local to the audited solvers.
```

---

## N06: Kinematics Unit Tests

```
You are agent N06 for the Baxter Noetic improvements project.

## Your Task

Add kinematics unit tests for `baxter_sim_kinematics`. Do not start N07 Docker setup yet.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md

## What To Do

1. Add the smallest useful gtest coverage for `baxter_sim_kinematics`.
2. Cover the N04/N05-critical behavior where feasible: FK/IK request validation, known FK/IK behavior, joint limit/seed handling, and gravity compensation setup.
3. Do not redesign the kinematics classes and do not migrate tf to tf2.
4. Keep tests local to the kinematics package; no mocks or broad test framework.
5. Run the smallest feasible build/test command and record results. If this machine still lacks ROS Noetic, document that blocker and run any direct static checks that do work.

## Output

Write findings and test results to:
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md

Then update N06 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N06 `completed` if the tests are present in the working tree and verification results are documented.

Then append the N07 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

gtest only. No fixtures, no mocking framework, no parameterized tests unless a plain test would be worse. Minimal kinematics coverage, no Docker work, no tf2 migration.
```

---

## N07: Modern Docker Setup

```
You are agent N07 for the Baxter Noetic improvements project.

## Your Task

Replace the outdated Kinetic Docker setup with a modern ROS Noetic Dockerfile.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md

## What To Do

1. Replace the obsolete Kinetic Dockerfile with a Noetic-based Dockerfile.
2. Use `ros:noetic-ros-base-focal` as the base image.
3. Install only the dependencies needed to build the workspace and run the Baxter simulator stack.
4. Include Gazebo support needed by the existing simulator packages.
5. Build the catkin workspace inside the image.
6. Add a minimal `.dockerignore` if it is missing.
7. Run the smallest feasible Docker build command and record results. If Docker is unavailable, document that blocker and run static checks that do work.

## Output

Write findings and build results to:
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md

Then update N07 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N07 `completed` if the Docker files are present in the working tree and verification results are documented.

Then append the N08 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Single Dockerfile, multi-stage only if it materially helps. No docker-compose, no devcontainer, no custom entrypoint scripts unless required. Do not start N08.
```

---

## N08: Docker Compose

```
You are agent N08 for the Baxter Noetic improvements project.

## Your Task

Create a minimal Docker Compose setup for the Baxter Noetic simulator stack. Do not start N09 CI/CD.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md

## What To Do

1. Create a minimal `docker-compose.yml` for the Noetic image from N07.
2. Include services for `roscore`, Baxter Gazebo simulation, MoveIt, and RViz only as needed for the existing stack.
3. Add GPU/display support configuration needed for Gazebo/RViz on Linux/WSL2, but keep it minimal.
4. Add `devcontainer.json` only if it materially helps VS Code users run the same compose setup.
5. Run the smallest feasible `docker compose` verification and record results. If GUI/GPU is unavailable, document that blocker and run non-GUI compose checks that do work.

## Output

Write findings and compose results to:
  ~/baxter_noetic_ws/logs/N08_docker_compose.log.md

Then update N08 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N08 `completed` if compose files are present in the working tree and verification results are documented.

Then append the N09 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Minimal compose only. No optional profiles, no health checks unless something breaks, no custom supervisor scripts, and no broad Docker redesign. Do not start N09.
```

---

## N09: CI/CD Pipeline

```
You are agent N09 for the Baxter Noetic improvements project.

## Your Task

Set up a minimal CI/CD pipeline for the Baxter Noetic checkout. Do not start N10 tf to tf2 migration.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md
  ~/baxter_noetic_ws/logs/N08_docker_compose.log.md

## What To Do

1. Add the smallest useful GitHub Actions CI workflow for this ROS Noetic workspace.
2. Build/test using the existing Noetic Docker setup where practical, instead of recreating ROS install logic in CI.
3. Run the existing smoke/kinematics checks that are feasible in CI.
4. Add Docker image build verification, but do not publish images yet.
5. Add a README badge only if the workflow path/name is final and the repository target is clear.
6. Run the smallest feasible local verification and record results. If GitHub Actions cannot be executed locally, document static validation and Docker-based checks that do work.

## Output

Write findings and CI results to:
  ~/baxter_noetic_ws/logs/N09_ci_cd.log.md

Then update N09 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N09 `completed` if CI files are present in the working tree and verification results are documented.

Then append the N09.5 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

One workflow file. No matrix builds, no caching optimization unless the build is too slow, no release publishing, no GHCR push, and no broad CI framework. Do not start N10.
```

---

## N09.5: CI Hardening & Test Blocker Cleanup

```
You are agent N09.5 for the Baxter Noetic improvements project.

## Your Task

Fix the known CI/test blockers documented during N09 so the Baxter Noetic CI can run fuller checks before the N10 tf to tf2 migration. Do not start N10.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md
  ~/baxter_noetic_ws/logs/N08_docker_compose.log.md
  ~/baxter_noetic_ws/logs/N09_ci_cd.log.md

## What To Do

1. Fix the full smoke-test blockers documented in N09 where they are real source issues:
   - replace obsolete `xacro_jade` usage with Noetic `xacro`
   - fix `baxter_dataflow.signals` so `weakrefset` resolves under Python 3
   - fix the pneumatic gripper xacro include path
2. Investigate the generated dynamic-reconfigure cfg import failures (`baxter_examples.cfg`, `baxter_interface.cfg`). Fix source/package setup only if the imports fail in the built Docker environment; otherwise adjust the smoke checker narrowly so generated cfg imports do not false-fail static import resolution.
3. Investigate `KinematicsTest.ComputesKnownFkAndIk`. Prefer fixing the test setup if it lacks a ROS master/identity transform or asks for an unsolvable IK pose; only change production kinematics code if the production behavior is actually wrong.
4. Update `.github/workflows/ci.yml` to remove obsolete smoke/kinematics filters once the corresponding full checks pass.
5. Keep Docker as the verification path; do not add native ROS install logic.
6. Do not migrate `tf` to `tf2`; that is N10.
7. Run the smallest useful Docker-backed verification and record results. At minimum, run Docker build, full smoke or documented remaining subset, kinematics gtest or documented remaining subset, launch parse checks, and `git diff --check`.

## Output

Write findings and CI hardening results to:
  ~/baxter_noetic_ws/logs/N09_5_ci_hardening.log.md

Then update N09.5 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N09.5 `completed` if fixes are present in the working tree and verification results are documented.

Then update the existing N10 prompt in:
  ~/baxter_noetic_ws/PROMPTS.md

Ensure N10 reads `~/baxter_noetic_ws/logs/N09_5_ci_hardening.log.md` before starting.

## Ponytail Mode

Fix only the N09-documented blockers. No broad cleanup, no package rewrites, no new CI framework, no tf2 migration. Shortest working diff wins.
```

---

## N10: tf -> tf2 Migration

```
You are agent N10 for the Baxter Noetic improvements project.

## Your Task

Migrate the Baxter Noetic checkout from deprecated `tf` usage to `tf2` where practical. Do not start N11 documentation overhaul.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md
  ~/baxter_noetic_ws/logs/N08_docker_compose.log.md
  ~/baxter_noetic_ws/logs/N09_ci_cd.log.md
  ~/baxter_noetic_ws/logs/N09_5_ci_hardening.log.md

Confirm the N09.5 log was reviewed before changing any `tf` usage.

## What To Do

1. Search for remaining `tf` dependencies and uses in the Noetic source checkout.
2. Mechanically migrate feasible C++ usage in `baxter_sim_kinematics` from `tf` to `tf2_ros`/`tf2_geometry_msgs`/`tf2_kdl` equivalents.
3. Update launch files that use `tf` static transform publisher to the `tf2_ros` equivalent where compatible.
4. Update CMake/package dependencies only where required by the migration.
5. Do not refactor node structure or change behavior beyond the tf to tf2 migration.
6. Run the smallest feasible Docker-backed build/test/CI checks and record results. If full tests still expose existing non-N10 failures, document the blocker and run the passable checks.

## Output

Write findings and migration results to:
  ~/baxter_noetic_ws/logs/N10_tf2_migration.log.md

Then update N10 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N10 `completed` if migration changes are present in the working tree and verification results are documented.

Then append the N11 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Mechanical migration only. No broad refactors, no ROS 2 migration, no documentation overhaul, no new CI framework. Keep compatibility with ROS Noetic.
```

---

## N11: Documentation Overhaul

```
You are agent N11 for the Baxter Noetic improvements project.

## Your Task

Overhaul the Baxter Noetic documentation. Do not start N12 release or pre-built Docker image publishing.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md
  ~/baxter_noetic_ws/logs/N08_docker_compose.log.md
  ~/baxter_noetic_ws/logs/N09_ci_cd.log.md
  ~/baxter_noetic_ws/logs/N09_5_ci_hardening.log.md
  ~/baxter_noetic_ws/logs/N10_tf2_migration.log.md

## What To Do

1. Update documentation for the current Noetic checkout only.
2. Cover native setup, Docker build/run, Docker Compose usage, tests/CI checks, simulator launch basics, MoveIt launch basics, and troubleshooting for known warnings/blockers.
3. Update README quick start and package docs where they are stale or misleading.
4. Keep docs Markdown-only; do not add Sphinx, rosdoc, generated docs, or a docs build system.
5. Do not publish Docker images, tag releases, or start N12.
6. Run the smallest feasible documentation checks and record results.

## Output

Write documentation changes and verification results to:
  ~/baxter_noetic_ws/logs/N11_documentation.log.md

Then update N11 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N11 `completed` if documentation changes are present in the working tree and verification results are documented.

Then append the N12 prompt to:
  ~/baxter_noetic_ws/PROMPTS.md

## Ponytail Mode

Markdown only. No doc generators, no new tooling, no release work, no broad code changes. Fix stale commands and explain verified workflows without turning this into a website project.
```

---

## N12: Fix Remaining Non-Kinematics Bugs

```
You are agent N12 for the Baxter Noetic improvements project.

## Your Task

Fix the P0/P1 bugs from the N02 audit that fall outside `baxter_sim_kinematics` and were never covered by N04-N11. N04's scope was always just the 8 kinematics bugs; these are the ones everywhere else in the SDK.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/AUDIT_2026-07-16.log.md (see "Addendum: non-kinematics scope gap" for current fix status of every item below)

## What To Do

1. `baxter_interface/src/baxter_interface/limb.py:348-349,368-369,385-386` — wrap `.keys()`/`.values()` in `list(...)` before assigning to ROS message fields.
2. `baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py` (`_pub_rate.publish`) and `baxter_examples/scripts/joint_velocity_wobbler.py` — cast the published rate to `int` before publishing to the `UInt16` topic.
3. `baxter_simulator/baxter_sim_hardware/src/baxter_emulator.cpp` — guard `msg.ranges[0]` on non-empty, guard `nav_light.find(msg.name)` on `!= end()`, guard joint-state velocity/effort access on array length.
4. `baxter_examples/scripts/joint_velocity_puppet.py`, `joint_velocity_wobbler.py` — replace `xrange` with `range`. `joint_trajectory_file_playback.py` — replace `operator.div` with `operator.truediv`.
5. `baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py` `_on_trajectory_action` — abort/return when `_get_trajectory_parameters()` aborts instead of continuing.
6. `baxter_interface/src/head_action/head_action.py` — stop blocking the feedback loop inside `_command_head`'s `set_pan(..., timeout=...)` call.
7. All 5 controllers in `baxter_simulator/baxter_sim_controllers/src/*.cpp` — check the return value of `init(robot, joint_nh)` and fail initialization if any sub-controller fails.
8. If time remains: stale shebangs (P1-08/09), masked exceptions in `baxter_tools/scripts/*.py` (P1-10), `update_robot.py` None bug (P1-11), non-standard VLA in `baxter_effort_controller.cpp` (P1-18), stale `.rosinstall` (P1-20).
9. Run the smoke suite and kinematics gtest after every change; nothing here should regress N04-N11.

## Output

Write findings and verification results to:
  ~/baxter_noetic_ws/logs/N12_non_kinematics_bugs.log.md

Then update N12 in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark N12 `completed` if the P0 items are fixed and verified by direct exercise, and the smoke/kinematics suites still pass unfiltered.

## Ponytail Mode

Minimal diff per bug, same discipline as N04: fix the bug, don't refactor surrounding code, don't add a validation framework, don't touch code that isn't broken. Completing N12 does not authorize starting the release step — that is a separate, user-gated Final Step.
```

---

## Final Step: Release (Pre-built Docker Images & Publish)

```
You are the release agent for the Baxter Noetic improvements project.

## Do Not Start This Without Explicit User Approval

This step is deliberately kept outside the numbered N01-N12 sequence. Do not build/tag/publish images and do not create a GitHub release unless the user has explicitly told you, in this conversation, to start the release. "N12 is done" or "all steps are complete" is not authorization — the user wants to review the codebase independently first.

## Your Task

Set up pre-built Docker image publishing and create a release for the Baxter Noetic checkout.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/MASTER_PLAN.md
  ~/baxter_noetic_ws/logs/N01_working_copy.log.md
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N03_testing_infrastructure.log.md
  ~/baxter_noetic_ws/logs/N04_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/N05_memory_leaks.log.md
  ~/baxter_noetic_ws/logs/N06_kinematics_tests.log.md
  ~/baxter_noetic_ws/logs/N07_docker_setup.log.md
  ~/baxter_noetic_ws/logs/N08_docker_compose.log.md
  ~/baxter_noetic_ws/logs/N09_ci_cd.log.md
  ~/baxter_noetic_ws/logs/N09_5_ci_hardening.log.md
  ~/baxter_noetic_ws/logs/N10_tf2_migration.log.md
  ~/baxter_noetic_ws/logs/N11_documentation.log.md
  ~/baxter_noetic_ws/logs/N12_non_kinematics_bugs.log.md
  ~/baxter_noetic_ws/logs/AUDIT_2026-07-16.log.md

## What To Do

1. Verify the N11 documentation state and current Docker/CI workflow before release work.
2. Add the smallest useful GHCR publishing path for the existing Noetic Dockerfile.
3. Publish images only from an appropriate release/tag workflow; do not replace the existing CI build gate.
4. Create release notes from the completed N01-N12 work.
5. Tag the release only after the Docker build, smoke tests, kinematics tests, and launch-parse checks pass.
6. Update documentation with `docker pull` / GHCR usage only after the image exists.
7. Run the smallest feasible verification and record results.

## Output

Write release and image-publishing results to:
  ~/baxter_noetic_ws/logs/RELEASE.log.md

Then update the Final Step (Release) section in:
  ~/baxter_noetic_ws/MASTER_PLAN.md

Only mark it `completed` if the release artifacts exist, the Docker image is published, and documentation has been updated with the published image instructions.

## Ponytail Mode

Use the existing Dockerfile and CI checks. No custom release scripts unless GitHub Actions cannot cover the needed release path. No broad code cleanup, no Docker redesign, no ROS 2 work.
```

---

## PX1: Deferred P1 Cleanup

```
You are the PX1 cleanup agent for the Baxter Noetic improvements project.

## Do Not Start Release Work

Do not tag, publish GHCR images, create release notes, or update docs with published-image instructions. This is still pre-release cleanup only.

## Your Task

Fix the remaining cheap P1 issues that N12 deliberately deferred.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/AUDIT_2026-07-16.log.md
  ~/baxter_noetic_ws/logs/N12_non_kinematics_bugs.log.md

## What To Do

1. Fix stale Python shebangs that still use `#!/usr/bin/env python` in executable scripts/cfg/setup files where Noetic should use Python 3.
2. Fix masked exceptions in `baxter_tools/scripts/*.py` where code reads `e.strerror` or raises undefined exceptions.
3. Fix `baxter_tools/scripts/update_robot.py` so `re.search(pattern, robot_version)` cannot receive `None`.
4. Review stale `.rosinstall` files and either update clearly stale Kinetic/development branches or document why they should remain untouched.
5. Do not redo N12 fixes and do not broaden into P2/P3 cleanup.

## Verification

Run the smallest checks that cover the changed files:
  git diff --check
  python3 -m py_compile <changed Python files>
  docker run smoke test if Python package behavior changed

If changing `.rosinstall` or setup/cfg files, run the Docker smoke suite:
  docker run --rm baxter-noetic:n12 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'

## Output

Write results to:
  ~/baxter_noetic_ws/logs/PX1_deferred_p1_cleanup.log.md

## Ponytail Mode

Minimal diff. Fix the concrete bug only. No new framework, no broad modernization, no package version changes.
```

---

## PX2: P2 Build, Package, And Launch Polish

```
You are the PX2 polish agent for the Baxter Noetic improvements project.

## Do Not Start Release Work

Do not tag, publish GHCR images, create release notes, or update docs with published-image instructions.

## Your Task

Work through P2 findings from `logs/N02_code_audit.log.md` that affect build/package correctness, install hygiene, launch consistency, or runtime polish.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N12_non_kinematics_bugs.log.md

## What To Do

1. Triage P2 findings and pick the smallest coherent batch.
2. Prefer build/package/launch correctness over cosmetic changes.
3. Fix missing `package.xml` dependencies only when directly supported by imports, launch files, or CMake usage.
4. Fix CMake install path issues only where install output is demonstrably wrong or incomplete.
5. Fix MoveIt launch argument mismatches only where launch parse or runtime behavior proves the mismatch.
6. Fix URDF/xacro inconsistencies only when the correction is local and does not change robot semantics unexpectedly.
7. Stop before broad refactors or release packaging.

## Verification

Always run:
  git diff --check
  docker build -f baxter_simulator/Dockerfile -t baxter-noetic:px2 .
  docker run --rm baxter-noetic:px2 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
  docker run --rm baxter-noetic:px2 bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_gazebo baxter_world.launch && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'

Run the kinematics gtest if touching simulator, URDF/xacro, or kinematics-adjacent code.

## Output

Write results to:
  ~/baxter_noetic_ws/logs/PX2_p2_build_package_launch_polish.log.md

## Ponytail Mode

One small batch. Do not chase every P2 finding in one pass. No speculative dependency additions.
```

---

## PX3: P3 Cosmetic And Repo Hygiene Polish

```
You are the PX3 hygiene agent for the Baxter Noetic improvements project.

## Do Not Start Release Work

Do not tag, publish GHCR images, create release notes, or update docs with published-image instructions.

## Your Task

Clean up low-risk P3 findings and stale repo hygiene issues after P0/P1/P2 work is stable.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/logs/N02_code_audit.log.md
  ~/baxter_noetic_ws/logs/N12_non_kinematics_bugs.log.md

## What To Do

1. Triage P3 findings and stale comments/docs/config.
2. Remove or update stale references only when clearly wrong in the Noetic checkout.
3. Keep package versions unchanged unless the user explicitly requests package-version release prep.
4. Keep public release docs unchanged: no GHCR, no `docker pull`, no tag instructions.
5. Avoid formatting-only churn across large files.

## Verification

Run:
  git diff --check
  python3 -m py_compile <changed Python files>

If touching docs only, also grep for stale forbidden release strings you changed around:
  rg 'ghcr.io|docker pull|run_baxter|sdk.rethinkrobotics.com/wiki' -g '*.md'

If touching launch/package/build files, run Docker smoke and launch parse checks.

## Output

Write results to:
  ~/baxter_noetic_ws/logs/PX3_p3_repo_hygiene.log.md

## Ponytail Mode

Delete stale text over adding new explanation. Keep each cleanup mechanical and obvious.
```

---

## PX4: GUI Runtime Validation

```
You are the PX4 runtime validation agent for the Baxter Noetic improvements project.

## Do Not Start Release Work

Do not tag, publish GHCR images, create release notes, or update docs with published-image instructions.

## Your Task

Validate the full Docker Compose runtime on a host with working GUI display forwarding.

## Context

Workspace root:
  ~/baxter_noetic_ws/

Source checkout:
  ~/baxter_noetic_ws/src/baxter_noetic/

Read first:
  ~/baxter_noetic_ws/logs/N12_non_kinematics_bugs.log.md
  ~/baxter_noetic_ws/src/baxter_noetic/README.md
  ~/baxter_noetic_ws/src/baxter_noetic/docker-compose.yml

## What To Do

1. Confirm Docker can forward display output to containers on this host.
2. Add only local/host-specific compose overrides if needed; do not bake host-specific paths into the default `docker-compose.yml`.
3. Run `docker compose up` for `roscore`, `baxter_sim`, `moveit`, and `rviz`.
4. Confirm live nodes include Gazebo, emulator, kinematics nodes, MoveIt, and RViz.
5. Confirm the emulator survives sparse/malformed input publishes as in N12.
6. Confirm gravity compensation topics publish while Gazebo services are available.

## Verification

Record exact commands and outputs for:
  docker compose config
  docker compose up
  docker compose ps
  rosnode list
  rosservice list | grep /gazebo/set_link_properties
  rostopic echo -n1 /robot/limb/left/gravity_compensation_torques

## Output

Write results to:
  ~/baxter_noetic_ws/logs/PX4_gui_runtime_validation.log.md

## Ponytail Mode

No code changes unless validation exposes a real repo bug. Host display configuration belongs in local overrides or documentation, not hard-coded defaults.
```
