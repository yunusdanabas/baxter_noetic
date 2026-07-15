# Baxter Interface

Python 3 / ROS Noetic interface classes and action servers for Baxter.

## Contents

| Path | Purpose |
|------|---------|
| `src/baxter_interface/` | Component APIs for limbs, grippers, head, cameras, IO, navigator, and robot enable state |
| `src/baxter_control/` | Generic controller utilities |
| `src/baxter_dataflow/` | Timing and flow helpers |
| `src/joint_trajectory_action/` | FollowJointTrajectory action implementation |
| `src/gripper_action/` | Gripper action implementation |
| `src/head_action/` | Head action implementation |
| `scripts/` | Action-server executables |
| `cfg/` | Dynamic reconfigure configs |

## Common Commands

Start the joint trajectory action server before executing MoveIt trajectories through the Baxter action interface:

```bash
source ~/baxter_noetic_ws/install/setup.bash
rosrun baxter_interface joint_trajectory_action_server.py
```

Other action servers:

```bash
rosrun baxter_interface gripper_action_server.py
rosrun baxter_interface head_action_server.py
```

## Tests

The repository CI runs the full smoke test and kinematics test in Docker. The smoke test imports representative `baxter_interface` modules and generated dynamic-reconfigure configs.

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker run --rm baxter-noetic:n07 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

See the root [README](../README.md) for setup and troubleshooting.
