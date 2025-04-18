Here’s a quick reference “cheat‐sheet” for the most common Baxter example commands, both via Carol’s  
`run_baxter` wrapper and the underlying `rosrun` calls.  

> **Prerequisites**  
> ```bash
> cd ~/ros_ws           # your ROS workspace  
> ./baxter.sh           # sets ROS_MASTER_URI and ROS_IP to talk to Baxter  
> ```  

---

## Robot Enable / Disable / State  
| **run_baxter**    | **rosrun**                                 | Description                       |
|-------------------|--------------------------------------------|-----------------------------------|
| `run_baxter enable`   | `rosrun baxter_tools enable_robot.py -e`     | Power on & enable motors          |
| `run_baxter disable`  | `rosrun baxter_tools enable_robot.py -d`     | Disable (power off)               |
| `run_baxter state`    | `rosrun baxter_tools enable_robot.py -s`     | Print current robot state         |
| `run_baxter reset`    | `rosrun baxter_tools enable_robot.py -r`     | Reset (clear faults)              |
| `run_baxter stop`     | `rosrun baxter_tools enable_robot.py -S`     | Stop all motion immediately       |

---

## Arms Tuck/Untuck (zero‑G mode)  
| **run_baxter**        | **rosrun**                               | Description                         |
|-----------------------|------------------------------------------|-------------------------------------|
| `run_baxter tuck`     | `rosrun baxter_tools tuck_arms.py -t`    | Tuck both arms into safe stow pose  |
| `run_baxter untuck`   | `rosrun baxter_tools tuck_arms.py -u`    | Untuck arms into zero‑G mode        |

---

## Joint Position Control  
| **run_baxter**               | **rosrun**                                                        | Description                                       |
|------------------------------|-------------------------------------------------------------------|---------------------------------------------------|
| `run_baxter arms_keyboard`   | `rosrun baxter_examples joint_position_keyboard.py`               | Keyboard control of individual joints             |
| `rosrun … joint_position_keyboard.py` then press `?`                  |                                                                   |
| `run_baxter record <file>`   | `rosrun baxter_examples joint_recorder.py -f <file>`              | Record time‑stamped joint + gripper data to CSV   |
| `run_baxter playback <file>` | `rosrun baxter_examples joint_position_file_playback.py -f <file>`| Play back a joint‑position recording              |
| `run_baxter springs <limb>`  | `rosrun baxter_examples joint_torque_springs.py -l <limb>`        | Virtual “spring” torque around current pose       |
| `run_baxter arms_wobbler`    | `rosrun baxter_examples joint_velocity_wobbler.py`                | Sinusoidal velocity “wobble” of both arms         |
| `run_baxter puppet <limb>`   | `rosrun baxter_examples joint_velocity_puppet.py -l <limb>`       | Mirror one arm’s velocity onto the other          |

---

## Joint Trajectories & IK  
| **run_baxter**                         | **rosrun**                                                        | Description                                               |
|----------------------------------------|-------------------------------------------------------------------|-----------------------------------------------------------|
| _start server_                         | `rosrun baxter_interface joint_trajectory_action_server.py`       | Enable FollowJointTrajectory action interface             |
| `run_baxter joint_trajectory <limb>`   | _(wrapper)_ runs action server + client on `<limb>`               | Send a simple predefined trajectory to `<limb>`            |
| `run_baxter ik <limb>`                 | `rosrun baxter_examples ik_service_client.py -l <limb>`           | Call on‑robot IK to solve for a Cartesian pose            |

---

## Head, Gripper & LEDs  
| **run_baxter**                   | **rosrun**                                 | Description                               |
|----------------------------------|--------------------------------------------|-------------------------------------------|
| `run_baxter head_wobbler`        | `rosrun baxter_examples head_wobbler.py`   | Random pan/nod of Baxter’s head           |
| `run_baxter gripper_keyboard`    | `rosrun baxter_examples gripper_keyboard.py` | Keyboard‑driven open/close                |
| `run_baxter digital_io`          | `rosrun baxter_examples digital_io_blink.py` | Blink the left‑arm navigator LEDs         |
| `run_baxter analog_io`           | `rosrun baxter_examples analog_io_rampup.py`| Ramp experiment on analog I/O (fans)      |

---

## Cameras & Displays  
| **run_baxter**                                                                                                                           | **rosrun**                                                                    | Description                                     |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|-------------------------------------------------|
| `run_baxter camera open <right|left|head> res <wide\|medium\|narrow>`<br/>`run_baxter camera close <...>` | `rosrun baxter_tools camera_control.py -o <name> -r <WxH>`<br/>`-c <name>` | Open/close/configure hand/head cameras          |
| &nbsp;&nbsp;then view with…<br/>`rosrun image_view image_view image:=/cameras/<name>/image`                                                                                   |                                                                               |                                                   |
| `run_baxter head_display <file>`                                                                                                           | `rosrun baxter_examples xdisplay_image.py --f <file>`                         | Show a PNG/JPG on Baxter’s face display          |

---

### Simulation & Planning  
- **Gazebo**  
  ```bash
  ./baxter.sh sim
  roslaunch baxter_gazebo baxter_world.launch
  rosrun baxter_tools enable_robot.py -e
  . run_baxter untuck
  ```
- **MoveIt!**  
  ```bash
  rosrun baxter_interface joint_trajectory_action_server.py
  roslaunch baxter_moveit_config demo_baxter.launch
  ```
  — in RViz use the “Planning” panel to pick goal poses and **Execute** to send to the real robot.

---

**Tip:** if you ever get lost, just run  
```bash
. run_baxter                # no args ➔ prints the full list of commands
```