# Changelog

This file records repository-level changes to the ROS Noetic/Python 3 port. The
historical package changelogs remain alongside their respective packages.

## 2026-08-25 - Public repository hardening

### Changed

- Reworked the root and package documentation around a simulation-first Docker
  and Compose workflow, native Noetic setup, MoveIt, testing, troubleshooting,
  and the ROS 2 Jazzy sibling repository.
- Standardized the locally built container image name as `baxter-noetic:local`.
- Replaced the robot hostname, workstation hostname, and workstation address in
  `baxter/baxter.sh` with deliberately invalid configuration placeholders.
- Expanded ignore rules for generated catkin/colcon output, bags, and local
  planning or agent-workflow material.

### Removed

- Removed project-specific pick-and-place scratch scripts from
  `baxter_examples` because they were not part of the verified SDK examples.
- Removed internal plans, prompts, audit logs, and workspace notes from the
  public tree.
- Removed redistributed Baxter manuals and thesis PDFs; documentation now
  directs readers to the original publisher or institutional source.

### Validation

- Built the complete Noetic image from `baxter_simulator/Dockerfile`.
- Passed all six repository smoke tests and all four simulator kinematics tests.
- Parsed the supported Gazebo and MoveIt launch paths successfully.
- Validated the Docker Compose configuration and whitespace-clean diff.
