FROM ros:noetic

RUN apt-get update
RUN apt-get install ros-noetic-rosbridge-suite -y
COPY ./ros_entrypoint.sh  .
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
