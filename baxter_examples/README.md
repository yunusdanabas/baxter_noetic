# Baxter Examples

Example scripts for using the Baxter SDK on ROS Noetic.

## Common Examples

| Script | Purpose |
|--------|---------|
| `joint_position_keyboard.py` | Keyboard joint-position control |
| `joint_position_waypoints.py` | Move through configured joint waypoints |
| `joint_recorder.py` | Record joint and gripper data to CSV |
| `joint_position_file_playback.py` | Replay a recorded joint-position CSV |
| `joint_torque_springs.py` | Virtual spring torque example |
| `joint_velocity_wobbler.py` | Sinusoidal velocity command example |
| `joint_velocity_puppet.py` | Mirror one arm onto the other |
| `joint_trajectory_client.py` | Send a FollowJointTrajectory goal |
| `ik_service_client.py` | Call the Baxter IK service |
| `gripper_keyboard.py` | Keyboard gripper control |
| `head_wobbler.py` | Head pan/nod example |
| `digital_io_blink.py` | Navigator LED blink example |
| `analog_io_rampup.py` | Analog IO ramp example |
| `xdisplay_image.py` | Show an image on the head display |
| `send_urdf_fragment.py` | Send a Noetic xacro-generated URDF fragment |

## Running Examples

Source the Noetic workspace first:

```bash
source ~/baxter_noetic_ws/install/setup.bash
```

For simulator or robot sessions, confirm the robot is enabled before motion examples:

```bash
rosrun baxter_tools enable_robot.py -s
rosrun baxter_tools enable_robot.py -e
```

Example commands:

```bash
rosrun baxter_examples joint_position_keyboard.py
rosrun baxter_examples ik_service_client.py -l left
rosrun baxter_examples gripper_keyboard.py
```

Do not run motion examples on a physical robot until networking, e-stop state, and workcell safety are confirmed.
