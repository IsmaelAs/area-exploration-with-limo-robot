FROM ros:noetic

RUN apt-get update && apt-get install -y curl git \
      libeigen3-dev libgoogle-glog-dev libudev-dev libuvc-dev \
      ros-noetic-tf ros-noetic-cv-bridge \
      ros-noetic-image-geometry ros-noetic-image-transport \
      ros-noetic-camera-info-manager ros-noetic-libuvc-ros \
      ros-noetic-gmapping ros-noetic-amcl \
      ros-noetic-costmap-2d ros-noetic-global-planner \
      ros-noetic-dwa-local-planner ros-noetic-navigation \
      ros-noetic-rosbridge-server xterm\
      ros-${ROS_DISTRO}-multirobot-map-merge ros-${ROS_DISTRO}-explore-lite

# For some reason, libusb is missing some pkg-config files
RUN curl -LO https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2 && \
      tar xvf libusb-1.0.26.tar.bz2 && cd libusb-1.0.26 && \
      ./configure && make install && cd .. && \
      rm -rf libusb-1.0.26*

# For some reason, libuvc is missing some pkg-config files
RUN git clone https://github.com/libuvc/libuvc && cd libuvc && \
      cmake . && make install && cd .. \
      rm -rf libuvc

# Install YDLidar SDK
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK && cd YDLidar-SDK && \
      cmake . && make install && cd .. && rm -rf YDLidar-SDK


ADD .rosinstall agx_ws/.rosinstall
ADD ./packages/src /agx_ws/src
RUN cd agx_ws && . /opt/ros/noetic/setup.sh  && rosinstall . ; catkin_make


RUN chmod +x agx_ws/src/explore_control/src/control_explore.py

COPY . /home/ros-package
WORKDIR /home/ros-package
RUN chmod +x ./entrypoint.sh

ENTRYPOINT [ "./entrypoint.sh" ]