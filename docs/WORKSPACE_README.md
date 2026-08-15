# Baxter Noetic Workspace

Local catkin workspace for Baxter ROS Noetic improvements. Git tracking lives in the SDK checkout only.

## Layout

```
baxter_noetic_ws/
├── src/baxter_noetic/   # Baxter Noetic SDK (git repo → github.com/yunusdanabas/baxter_noetic)
├── logs/                # Local agent logs (not tracked)
├── MASTER_PLAN.md       # Local planning (not tracked)
├── PROMPTS.md           # Local agent prompts (not tracked)
├── build/               # Catkin build (generated)
└── devel/               # Catkin devel space (generated)
```

## Source And Docs

SDK packages and reference PDFs are under [`src/baxter_noetic/`](src/baxter_noetic/). See the source [README](src/baxter_noetic/README.md) for native setup, Docker, Compose, simulator, MoveIt, tests, and troubleshooting. See [docs/](src/baxter_noetic/docs/) for the documentation index and offline Baxter manuals.

## Native Build

Native setup requires Ubuntu 20.04 and ROS Noetic. If this host is not using Noetic, use the Docker quick start in [`src/baxter_noetic/README.md`](src/baxter_noetic/README.md).

```bash
cd ~/baxter_noetic_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin config --extend /opt/ros/noetic --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build --jobs 1 --limit-status-rate 0.2 --no-notify
source install/setup.bash
```
