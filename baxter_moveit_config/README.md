# Baxter MoveIt Config

MoveIt configuration for Baxter on ROS Noetic.

## Verified Launch Paths

Load the planning context and start `move_group`:

```bash
source ~/baxter_noetic_ws/install/setup.bash
roslaunch baxter_moveit_config planning_context.launch load_robot_description:=true
roslaunch baxter_moveit_config move_group.launch
```

Start RViz when a display is available:

```bash
roslaunch baxter_moveit_config moveit_rviz.launch config:=true
```

Fake-controller demo path:

```bash
roslaunch baxter_moveit_config demo_dummy.launch
```

The CI launch-parse check verifies `planning_context.launch`, `move_group.launch`, `demo_dummy.launch`, and the tf2 static-publisher sensor-manager path.

## Docker Check

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
docker run --rm baxter-noetic:local bash -lc 'source /root/baxter_ws/install/setup.bash && roslaunch --nodes baxter_moveit_config planning_context.launch load_robot_description:=true && roslaunch --nodes baxter_moveit_config move_group.launch'
```

## Trajectory Execution

For execution through Baxter's action interface, start the joint trajectory action server first:

```bash
rosrun baxter_interface joint_trajectory_action_server.py
```

Use RViz's MotionPlanning panel to plan and execute only after the target robot or simulator is enabled and safe.

## Regenerating SRDF

The tracked SRDF is generated from `config/baxter.srdf.xacro`. To inspect regenerated output without overwriting the tracked file:

```bash
rosrun xacro xacro $(rospack find baxter_moveit_config)/config/baxter.srdf.xacro left_electric_gripper:=true right_electric_gripper:=true left_tip_name:=left_gripper right_tip_name:=right_gripper >/tmp/baxter.srdf
```

## Notes

Optional Kinect, Xtion, warehouse, CHOMP, and STOMP paths are not the primary verified quick-start path. Prefer the launch commands above unless you are specifically validating those optional integrations.
