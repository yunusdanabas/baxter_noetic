---
step: PX4
title: GUI Runtime Validation
agent_date: 2026-07-16
status: passed
source_checkout: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
---

# PX4: GUI Runtime Validation

## Summary

Full Docker Compose runtime passed with `roscore`, `baxter_sim`, `moveit`, and `rviz` running together.

Docker GUI forwarding required a host-local override outside the repo because this Docker Desktop backend denied direct `/tmp/.X11-unix` bind mounts. The final validation used `/tmp/opencode/px4-gui.override.yml`, a local TCP X11 proxy, and a workspace-local copied Xauthority file. The default `docker-compose.yml` was not modified for host display paths.

No release work was started. No tags, GHCR images, release notes, or published-image docs were created.

## Host GUI Preflight

Command:

```bash
printenv DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR QT_QPA_PLATFORM XAUTHORITY
```

Output:

```text
:0
wayland-0
/run/user/1000
/run/user/1000/.mutter-Xwaylandauth.H77RS3
```

Command:

```bash
docker compose version
```

Output:

```text
Docker Compose version v5.3.0
```

Command:

```bash
docker version --format '{{.Client.Version}} {{.Server.Version}}'
```

Output:

```text
29.6.1 29.6.1
```

Command:

```bash
test -S /tmp/.X11-unix/X0 && test -r "$XAUTHORITY" && printf 'x11 socket and xauthority readable\n'
```

Output:

```text
x11 socket and xauthority readable
```

## Local GUI Override

Initial direct X11 socket mounting failed with this host-specific Docker Desktop error:

```text
Error response from daemon: mounts denied: 
The path /tmp/.X11-unix is not shared from the host and is not known to Docker.
You can configure shared paths from Docker -> Preferences... -> Resources -> File Sharing.
See https://docs.docker.com/ for more info.
```

Final local override used for validation:

```yaml
services:
  rviz:
    environment:
      DISPLAY: host.docker.internal:0
      QT_QPA_PLATFORM: xcb
      XAUTHORITY: /tmp/.docker.xauth
      LIBGL_ALWAYS_SOFTWARE: "1"
    volumes:
      - /home/yunusdanabas/baxter_noetic_ws/.px4-x11/docker.xauth:/tmp/.docker.xauth:ro
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml run --rm --no-deps rviz bash -lc 'python3 - <<"PY"
import os, socket
host = "host.docker.internal"
print("DISPLAY", os.environ.get("DISPLAY"))
print("XAUTHORITY", os.environ.get("XAUTHORITY"), os.path.exists(os.environ.get("XAUTHORITY", "")))
print("addr", socket.gethostbyname(host))
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(5)
try:
    s.connect((host, 6000))
    print("tcp connect ok")
except Exception as e:
    print("tcp connect failed", repr(e))
finally:
    s.close()
PY'
```

Output:

```text
 Container baxter_noetic-rviz-run-7614b94dbab8 Creating 
 Container baxter_noetic-rviz-run-7614b94dbab8 Created 
DISPLAY host.docker.internal:0
XAUTHORITY /tmp/.docker.xauth True
addr 192.168.65.2
tcp connect ok
```

## docker compose config

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml config
```

Output:

```yaml
name: baxter_noetic
services:
  baxter_sim:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash && roslaunch --wait baxter_gazebo baxter_world.launch gui:=false headless:=true
    depends_on:
      roscore:
        condition: service_started
        required: true
    environment:
      DISPLAY: :0
      NVIDIA_DRIVER_CAPABILITIES: all
      QT_QPA_PLATFORM: offscreen
      QT_X11_NO_MITSHM: "1"
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
      WAYLAND_DISPLAY: wayland-0
      XDG_RUNTIME_DIR: /run/user/1000
    image: baxter-noetic:n07
    init: true
    network_mode: host
  moveit:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash; roslaunch --wait baxter_moveit_config planning_context.launch load_robot_description:=true & sleep 5; exec roslaunch --wait baxter_moveit_config move_group.launch
    depends_on:
      baxter_sim:
        condition: service_started
        required: true
      roscore:
        condition: service_started
        required: true
    environment:
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
    image: baxter-noetic:n07
    init: true
    network_mode: host
  roscore:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash && roscore
    environment:
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
    image: baxter-noetic:n07
    init: true
    network_mode: host
  rviz:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash && roslaunch --wait baxter_moveit_config moveit_rviz.launch config:=true
    depends_on:
      moveit:
        condition: service_started
        required: true
      roscore:
        condition: service_started
        required: true
    environment:
      DISPLAY: host.docker.internal:0
      LIBGL_ALWAYS_SOFTWARE: "1"
      NVIDIA_DRIVER_CAPABILITIES: all
      QT_QPA_PLATFORM: xcb
      QT_X11_NO_MITSHM: "1"
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
      WAYLAND_DISPLAY: wayland-0
      XAUTHORITY: /tmp/.docker.xauth
      XDG_RUNTIME_DIR: /run/user/1000
    image: baxter-noetic:n07
    init: true
    network_mode: host
    volumes:
      - type: bind
        source: /home/yunusdanabas/baxter_noetic_ws/.px4-x11/docker.xauth
        target: /tmp/.docker.xauth
        read_only: true
        bind: {}
x-baxter:
  build:
    context: .
    dockerfile: baxter_simulator/Dockerfile
  image: baxter-noetic:n07
  init: true
  network_mode: host
x-gui-env:
  DISPLAY: :0
  NVIDIA_DRIVER_CAPABILITIES: all
  QT_QPA_PLATFORM: offscreen
  QT_X11_NO_MITSHM: "1"
  WAYLAND_DISPLAY: wayland-0
  XDG_RUNTIME_DIR: /run/user/1000
x-ros-env:
  ROS_IP: 127.0.0.1
  ROS_MASTER_URI: http://127.0.0.1:11311
```

## docker compose up

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml up -d roscore baxter_sim moveit rviz
```

Output:

```text
 Container baxter_noetic-baxter_sim-1 Running 
 Container baxter_noetic-moveit-1 Running 
 Container baxter_noetic-roscore-1 Running 
 Container baxter_noetic-rviz-1 Recreate 
 Container baxter_noetic-rviz-1 Recreated 
 Container baxter_noetic-rviz-1 Starting 
 Container baxter_noetic-rviz-1 Started 
```

## docker compose ps

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml ps
```

Output:

```text
NAME                         IMAGE               COMMAND                  SERVICE      CREATED              STATUS              PORTS
baxter_noetic-baxter_sim-1   baxter-noetic:n07   "/ros_entrypoint.sh …"   baxter_sim   5 minutes ago        Up 5 minutes        
baxter_noetic-moveit-1       baxter-noetic:n07   "/ros_entrypoint.sh …"   moveit       5 minutes ago        Up 5 minutes        
baxter_noetic-roscore-1      baxter-noetic:n07   "/ros_entrypoint.sh …"   roscore      5 minutes ago        Up 5 minutes        
baxter_noetic-rviz-1         baxter-noetic:n07   "/ros_entrypoint.sh …"   rviz         About a minute ago   Up About a minute   
```

## rosnode list

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rosnode list'
```

Output:

```text
/base_to_world
/baxter_emulator
/baxter_sim_kinematics_left
/baxter_sim_kinematics_right
/gazebo
/move_group
/robot/controller_spawner
/robot/controller_spawner_stopped
/robot_state_publisher
/rosout
/rviz_docker_desktop_7_6512401007516196352
```

Confirmed live nodes include Gazebo, emulator, left/right kinematics nodes, MoveIt, and RViz.

## Gazebo Service

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rosservice list | grep /gazebo/set_link_properties'
```

Output:

```text
/gazebo/set_link_properties
```

## Gravity Compensation Topic

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rostopic echo -n1 /robot/limb/left/gravity_compensation_torques'
```

Output:

```yaml
header: 
  seq: 19964
  stamp: 
    secs: 209
    nsecs: 642000000
  frame_id: "base"
name: 
  - left_s0
  - left_s1
  - left_e0
  - left_e1
  - left_w0
  - left_w1
  - left_w2
commanded_position: []
commanded_velocity: []
commanded_acceleration: []
commanded_effort: []
actual_position: [0.18818256844216918, 1.0470048273179149, 0.017947578533145325, 0.5036315403115514, -0.8705051438116325, -0.030444165161946923, -0.09713405631485905]
actual_velocity: [-1.9383756926983535e-05, 3.26570356838878e-09, 5.225545141270044e-05, 9.115423011150235e-06, -0.0021837438102555043, -7.782082588072443e-06, -0.0002225933792384758]
actual_effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gravity_model_effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gravity_only: []
hysteresis_model_effort: []
crosstalk_model_effort: []
hystState: 0.0
---
```

Confirmed gravity compensation publishes while `/gazebo/set_link_properties` is available.

## Emulator Sparse/Malformed Input Check

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rostopic pub -1 /sim/laserscan/left_hand_range/state sensor_msgs/LaserScan "{range_min: 0.0, range_max: 10.0, ranges: []}" && rostopic pub -1 /sim/laserscan/right_hand_range/state sensor_msgs/LaserScan "{range_min: 0.0, range_max: 10.0, ranges: []}" && rostopic pub -1 /robot/digital_io/command baxter_core_msgs/DigitalOutputCommand "{name: not_a_light, value: true}" && rostopic pub -1 /robot/joint_states sensor_msgs/JointState "{name: [head_pan, left_s0], position: [], velocity: [], effort: []}" && sleep 2 && rosnode ping -c1 /baxter_emulator'
```

Output:

```text
publishing and latching message for 3.0 seconds
publishing and latching message for 3.0 seconds
publishing and latching message for 3.0 seconds
publishing and latching message for 3.0 seconds
rosnode: node is [/baxter_emulator]
pinging /baxter_emulator with a timeout of 3.0s
xmlrpc reply from http://127.0.0.1:58991/	time=0.871897ms
```

Confirmed `/baxter_emulator` survived empty laser scans, an unknown navigator light command, and sparse `JointState` arrays.

## Final Status

PX4 GUI runtime validation passed on this host using local display forwarding configuration only.

## Recheck: 2026-07-16

Status: passed again.

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml config
```

Output:

```yaml
name: baxter_noetic
services:
  baxter_sim:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash && roslaunch --wait baxter_gazebo baxter_world.launch gui:=false headless:=true
    depends_on:
      roscore:
        condition: service_started
        required: true
    environment:
      DISPLAY: :0
      NVIDIA_DRIVER_CAPABILITIES: all
      QT_QPA_PLATFORM: offscreen
      QT_X11_NO_MITSHM: "1"
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
      WAYLAND_DISPLAY: wayland-0
      XDG_RUNTIME_DIR: /run/user/1000
    image: baxter-noetic:n07
    init: true
    network_mode: host
  moveit:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash; roslaunch --wait baxter_moveit_config planning_context.launch load_robot_description:=true & sleep 5; exec roslaunch --wait baxter_moveit_config move_group.launch
    depends_on:
      baxter_sim:
        condition: service_started
        required: true
      roscore:
        condition: service_started
        required: true
    environment:
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
    image: baxter-noetic:n07
    init: true
    network_mode: host
  roscore:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash && roscore
    environment:
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
    image: baxter-noetic:n07
    init: true
    network_mode: host
  rviz:
    build:
      context: /home/yunusdanabas/baxter_noetic_ws/src/baxter_noetic
      dockerfile: baxter_simulator/Dockerfile
    command:
      - bash
      - -lc
      - source /root/baxter_ws/install/setup.bash && roslaunch --wait baxter_moveit_config moveit_rviz.launch config:=true
    depends_on:
      moveit:
        condition: service_started
        required: true
      roscore:
        condition: service_started
        required: true
    environment:
      DISPLAY: host.docker.internal:0
      LIBGL_ALWAYS_SOFTWARE: "1"
      NVIDIA_DRIVER_CAPABILITIES: all
      QT_QPA_PLATFORM: xcb
      QT_X11_NO_MITSHM: "1"
      ROS_IP: 127.0.0.1
      ROS_MASTER_URI: http://127.0.0.1:11311
      WAYLAND_DISPLAY: wayland-0
      XAUTHORITY: /tmp/.docker.xauth
      XDG_RUNTIME_DIR: /run/user/1000
    image: baxter-noetic:n07
    init: true
    network_mode: host
    volumes:
      - type: bind
        source: /home/yunusdanabas/baxter_noetic_ws/.px4-x11/docker.xauth
        target: /tmp/.docker.xauth
        read_only: true
        bind: {}
x-baxter:
  build:
    context: .
    dockerfile: baxter_simulator/Dockerfile
  image: baxter-noetic:n07
  init: true
  network_mode: host
x-gui-env:
  DISPLAY: :0
  NVIDIA_DRIVER_CAPABILITIES: all
  QT_QPA_PLATFORM: offscreen
  QT_X11_NO_MITSHM: "1"
  WAYLAND_DISPLAY: wayland-0
  XDG_RUNTIME_DIR: /run/user/1000
x-ros-env:
  ROS_IP: 127.0.0.1
  ROS_MASTER_URI: http://127.0.0.1:11311
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml up -d roscore baxter_sim moveit rviz
```

Output:

```text
 Container baxter_noetic-roscore-1 Creating 
 Container baxter_noetic-roscore-1 Created 
 Container baxter_noetic-baxter_sim-1 Creating 
 Container baxter_noetic-baxter_sim-1 Created 
 Container baxter_noetic-moveit-1 Creating 
 Container baxter_noetic-moveit-1 Created 
 Container baxter_noetic-rviz-1 Creating 
 Container baxter_noetic-rviz-1 Created 
 Container baxter_noetic-roscore-1 Starting 
 Container baxter_noetic-roscore-1 Started 
 Container baxter_noetic-baxter_sim-1 Starting 
 Container baxter_noetic-baxter_sim-1 Started 
 Container baxter_noetic-moveit-1 Starting 
 Container baxter_noetic-moveit-1 Started 
 Container baxter_noetic-rviz-1 Starting 
 Container baxter_noetic-rviz-1 Started 
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml ps
```

Output after startup settling:

```text
NAME                         IMAGE               COMMAND                  SERVICE      CREATED              STATUS              PORTS
baxter_noetic-baxter_sim-1   baxter-noetic:n07   "/ros_entrypoint.sh …"   baxter_sim   About a minute ago   Up About a minute   
baxter_noetic-moveit-1       baxter-noetic:n07   "/ros_entrypoint.sh …"   moveit       About a minute ago   Up About a minute   
baxter_noetic-roscore-1      baxter-noetic:n07   "/ros_entrypoint.sh …"   roscore      About a minute ago   Up About a minute   
baxter_noetic-rviz-1         baxter-noetic:n07   "/ros_entrypoint.sh …"   rviz         About a minute ago   Up About a minute   
```

Output after emulator malformed-input check:

```text
NAME                         IMAGE               COMMAND                  SERVICE      CREATED         STATUS         PORTS
baxter_noetic-baxter_sim-1   baxter-noetic:n07   "/ros_entrypoint.sh …"   baxter_sim   2 minutes ago   Up 2 minutes   
baxter_noetic-moveit-1       baxter-noetic:n07   "/ros_entrypoint.sh …"   moveit       2 minutes ago   Up 2 minutes   
baxter_noetic-roscore-1      baxter-noetic:n07   "/ros_entrypoint.sh …"   roscore      2 minutes ago   Up 2 minutes   
baxter_noetic-rviz-1         baxter-noetic:n07   "/ros_entrypoint.sh …"   rviz         2 minutes ago   Up 2 minutes   
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rosnode list'
```

Output:

```text
/base_to_world
/baxter_emulator
/baxter_sim_kinematics_left
/baxter_sim_kinematics_right
/gazebo
/move_group
/robot/controller_spawner
/robot/controller_spawner_stopped
/robot_state_publisher
/rosout
/rviz_docker_desktop_7_7794141944650936512
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rosservice list | grep /gazebo/set_link_properties'
```

Output:

```text
/gazebo/set_link_properties
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rostopic echo -n1 /robot/limb/left/gravity_compensation_torques'
```

Output:

```yaml
header: 
  seq: 4910
  stamp: 
    secs: 59
    nsecs: 107000000
  frame_id: "base"
name: 
  - left_s0
  - left_s1
  - left_e0
  - left_e1
  - left_w0
  - left_w1
  - left_w2
commanded_position: []
commanded_velocity: []
commanded_acceleration: []
commanded_effort: []
actual_position: [0.43303720948901425, 1.0470021344809535, -0.2183290089417591, 0.5485133231485282, -0.09195375398487737, 0.005429814343171024, 0.016806279870215768]
actual_velocity: [0.0016239599408204909, 1.956266234141575e-06, -0.000561922707099064, -0.00023128463389787847, 0.0026287640595674014, 0.0006945650923478556, 0.0010125550938410734]
actual_effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gravity_model_effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gravity_only: []
hysteresis_model_effort: []
crosstalk_model_effort: []
hystState: 0.0
---
```

Command:

```bash
docker compose -f docker-compose.yml -f /tmp/opencode/px4-gui.override.yml exec -T roscore bash -lc 'source /root/baxter_ws/install/setup.bash && rostopic pub -1 /sim/laserscan/left_hand_range/state sensor_msgs/LaserScan "{range_min: 0.0, range_max: 10.0, ranges: []}" && rostopic pub -1 /sim/laserscan/right_hand_range/state sensor_msgs/LaserScan "{range_min: 0.0, range_max: 10.0, ranges: []}" && rostopic pub -1 /robot/digital_io/command baxter_core_msgs/DigitalOutputCommand "{name: not_a_light, value: true}" && rostopic pub -1 /robot/joint_states sensor_msgs/JointState "{name: [head_pan, left_s0], position: [], velocity: [], effort: []}" && sleep 2 && rosnode ping -c1 /baxter_emulator'
```

Output:

```text
publishing and latching message for 3.0 seconds
publishing and latching message for 3.0 seconds
publishing and latching message for 3.0 seconds
publishing and latching message for 3.0 seconds
rosnode: node is [/baxter_emulator]
pinging /baxter_emulator with a timeout of 3.0s
xmlrpc reply from http://127.0.0.1:55833/	time=1.278639ms
```

Recheck confirmed the full compose stack stayed live, including RViz, after the emulator sparse/malformed input exercise.
