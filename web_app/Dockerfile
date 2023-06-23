FROM osrf/ros:noetic-desktop-xenial

MAINTAINER Rizwin Antony <antonyrizwin@gmail.com> 

RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-noetic-desktop-full=1.3.2-0* \
    ros-noetic-navigation\
    ros-noetic-slam-gmapping\
    ros-noetic-rosbridge-suite\
    ros-noetic-tf2-web-republisher\
    ros-noetic-robot-pose-publisher\
    python-pip\
    python-catkin-tools\
    python-rosdep\
    python-rosinstall\
    python-rosinstall-generator\
    python-wstool\
    build-essential\
    python-rosdep\
    &&rm -rf /var/lib/apt/lists/*

COPY requirements.txt /root/
RUN pip install -r /root/requirements.txt


 
# clone ros package repo
ENV ROS_WS /root/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git 
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
WORKDIR $ROS_WS


COPY ["turtlebot3_bringup.launch", "turtlebot3_navigation.launch","/root/ros_ws/src/turtlebot3/turtlebot3_navigation/launch/"]

COPY ["turtlebot3_slam.launch", "/root/ros_ws/src/turtlebot3/turtlebot3_slam/launch/"]

# install ros package dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y \
      --from-paths \
        src/turtlebot3/turtlebot3 \
        src/turtlebot3/turtlebot3_bringup \
        src/turtlebot3/turtlebot3_description \
        src/turtlebot3/turtlebot3_example \
        src/turtlebot3/turtlebot3_navigation \
        src/turtlebot3/turtlebot3_slam\
        src/turtlebot3/turtlebot3_teleop\
        src/turtlebot3_msgs\
        src/turtlebot3_simulations/turtlebot3_fake\
        src/turtlebot3_simulations/turtlebot3_gazebo\
        src/turtlebot3_simulations/turtlebot3_simulations\
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build \
      turtlebot3\
      turtlebot3_bringup\
      turtlebot3_description\
      turtlebot3_example\
      turtlebot3_navigation\
      turtlebot3_slam\
      turtlebot3_teleop\
      turtlebot3_msgs\
      turtlebot3_fake\
      turtlebot3_gazebo\
      turtlebot3_simulations

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

WORKDIR /root/

COPY static /root/static/
COPY templates /root/templates
COPY app.py /root

ENV FLASK_APP=app.py

ENV TURTLEBOT3_MODEL=waffle

EXPOSE 5000

CMD ["flask", "run"]