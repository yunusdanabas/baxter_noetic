# Baxter SDK Metapackage

This directory contains the SDK metapackage and legacy setup helper for the ROS Noetic Baxter checkout.

## Contents

| Path | Purpose |
|------|---------|
| `baxter_sdk/` | Catkin metapackage that depends on the Baxter SDK package set |
| `baxter_sdk.rosinstall` | Historical rosinstall file; the current workspace already vendors the packages |
| `baxter.sh` | Legacy environment helper that must be configured for the local robot network |

## Current Setup Path

Use the repository root [README](../README.md) for current native, Docker, Compose, simulator, MoveIt, and test commands.

`baxter.sh` is not the primary quick start for this checkout. It ships with
deliberately invalid host/IP placeholders and expects a devel-space workspace;
the repository's documented default uses install space. Configure and review the
helper before using it with a physical robot.

```bash
source /opt/ros/noetic/setup.bash
source ~/baxter_noetic_ws/install/setup.bash
export ROS_MASTER_URI=http://<robot-hostname-or-ip>:11311
export ROS_IP=<workstation-ip-on-robot-network>
rosrun baxter_tools enable_robot.py -s
```

Do not run robot motion commands until networking, e-stop state, and workcell safety are confirmed.
