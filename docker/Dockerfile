# ROS 2 Jazzy + Gazebo Harmonic + TurtleBot 4 simulator, Nav2 (incl. docking), RTAB-Map, AprilTag.
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-turtlebot4-simulator \
      ros-jazzy-turtlebot4-navigation \
      ros-jazzy-turtlebot4-viz \
      ros-jazzy-nav2-bringup \
      ros-jazzy-opennav-docking \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-apriltag-ros \
      mesa-utils python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && dpkg-query -W -f='${Package}=${Version}\n' 'ros-jazzy-*' > /opt/ros-package-versions.txt

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ws
