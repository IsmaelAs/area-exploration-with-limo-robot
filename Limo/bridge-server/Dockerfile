FROM ros:noetic

RUN apt-get update
RUN apt-get install ros-noetic-rosbridge-suite -y
CMD ["roslaunch", "rosbridge_server", "rosbridge_websocket.launch"]