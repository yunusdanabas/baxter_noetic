# Baxter Tools

Operational and maintenance tools for Baxter on ROS Noetic.

## Scripts

| Script | Purpose |
|--------|---------|
| `enable_robot.py` | Query, enable, disable, reset, or stop the robot |
| `tuck_arms.py` | Tuck or untuck both arms |
| `camera_control.py` | Open, close, and configure cameras |
| `calibrate_arm.py` | Run arm calibration routines |
| `tare.py` | Run tare routines |
| `update_robot.py` | Robot software update helper for supported robot setups |
| `description_publisher.py` | Publish `robot_description` on a latched topic |
| `smoke_test.py` | Historical manual smoke test entry point |

## Common Commands

With the workspace sourced:

```bash
rosrun baxter_tools enable_robot.py -s
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools enable_robot.py -d
rosrun baxter_tools tuck_arms.py -u
rosrun baxter_tools tuck_arms.py -t
```

Camera example:

```bash
rosrun baxter_tools camera_control.py -o head_camera -r 1280x800
rosrun baxter_tools camera_control.py -c head_camera
```

## Automated Smoke Test

The current automated smoke test lives at repository root `test/test_smoke.py` and is hooked into catkin through this package.

```bash
cd ~/baxter_noetic_ws/src/baxter_noetic
python3 test/test_smoke.py
```

Run it in Docker for the verified Noetic environment:

```bash
docker run --rm baxter-noetic:n07 bash -lc 'source /root/baxter_ws/install/setup.bash && cd /root/baxter_ws/src/baxter_noetic && python3 test/test_smoke.py'
```

See the root [README](../README.md) for setup, Docker, and CI details.
