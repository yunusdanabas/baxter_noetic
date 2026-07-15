FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
    CATKIN_WS=/root/baxter_ws \
    ROS_PYTHON_VERSION=3 \
    PYTHONIOENCODING=UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-catkin-tools \
    python3-nose \
    python3-numpy \
    python3-rosdep \
    python3-rospkg \
    qtbase5-dev \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update --include-eol-distros

WORKDIR ${CATKIN_WS}
COPY . src/baxter_noetic

RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/noetic/setup.bash && \
    catkin config --extend /opt/ros/noetic --install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --jobs 1 --limit-status-rate 0.2 --no-notify

RUN echo "source ${CATKIN_WS}/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
