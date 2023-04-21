FROM ros:melodic
RUN apt-get update && apt-get install -y ros-melodic-libfranka ros-melodic-franka-ros

WORKDIR /panda_ws/src
RUN git clone https://repo.ijs.si/hcr/franka/robot_module_msgs
RUN git clone https://github.com/abr-ijs/ijs_controllers
WORKDIR /panda_ws
RUN . /opt/ros/melodic/setup.sh && catkin_make
 
