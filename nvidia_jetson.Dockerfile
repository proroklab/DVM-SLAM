FROM dustynv/ros:humble-ros-base-l4t-r35.3.1
# FROM dustynv/ros:humble-pytorch-l4t-r35.3.1 # when using pytorch (image is larger though)

# Add system packages to be installed here
RUN apt-get update && apt-get install -y git vim

#RUN mkdir -p /opt/robomaster/src
#COPY src /opt/root_ws/src

#WORKDIR /opt/root_ws

#RUN . /opt/ros/humble/install/setup.sh && colcon build --symlink-install --packages-select robomaster_msgs freyja_msgs state_manager lqg_control robomaster_handler sensing_msgs control waypoint_manager

#COPY cyclone_dds_profile.xml /opt/robomaster

WORKDIR /opt/root_ws

# Install Pangolin
RUN mkdir thirdparty && cd thirdparty
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git --depth 1 \
    && cd Pangolin \
    && yes | ./scripts/install_prerequisites.sh required \
    && cmake -B build \
    && cmake --build build -j 4 \
    && cmake --install build

# Install cv_bridge
RUN . /opt/ros/humble/install/setup.sh \
    && cd /opt/root_ws/thirdparty \
    && git clone https://github.com/ros-perception/vision_opencv.git -b humble \
    && cd vision_opencv \
    && MAKEFLAGS="-j 4" colcon build --symlink-install

# install interactive_markers
RUN . /opt/ros/humble/install/setup.sh \
    && cd /opt/root_ws/thirdparty \
    && git clone https://github.com/ros-visualization/interactive_markers.git -b humble \
    && cd interactive_markers \
    && MAKEFLAGS="-j 4" colcon build --symlink-install

# Build part II project
RUN apt-get update && yes | apt-get install libboost-all-dev
RUN cd /opt/root_ws \
    && . /opt/ros/humble/install/setup.sh \
    && git clone https://github.com/jyjblrd/part_II_project \
    && cd part_II_project \
    && git config --global --add safe.directory /opt/root_ws/part_II_project
# && MAKEFLAGS="-j 4" colcon build

RUN tar -xvzf /opt/root_ws/part_II_project/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.tar.gz -C /opt/root_ws/part_II_project/src/orb_slam3_ros/orb_slam3/Vocabulary/

# ros2 run orb_slam3_ros ros_mono --ros-args -p agentId:=1 -p config:=/opt/root_ws/part_II_project/src/orb_slam3_ros/configs/robomaster.yaml -p imageTopic:=/robomaster_0/camera_0/image_raw -p vocFile:=/opt/root_ws/part_II_project/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt -p useViewer:=false
# ros2 run motion_controller follow_the_leader --ros-args -p agentId:=2 -p positionOffsetX:=0.0 -p positionOffsetY:=0.0 -p rotationOffset:=0.0 -p linearGain:=1.0 -p angularGain:=1.0 -p cmdVelTopic:=/robomaster_1/cmd_vel
# ros2 run motion_controller collision_avoidance --ros-args -p agentId:=2 -p linearGain:=.5 -p angularGain:=0.1 -p agentRadius:=0.05 -p cmdVelTopic:=/robomaster_1/cmd_vel -p Qc:=4.0 -p kappa:=15.0 -p staticKappa:=60.0

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#ENV CYCLONEDDS_URI=/opt/robomaster/cyclone_dds_profile.xml

#COPY ros_entrypoint.sh /opt/robomaster/ros_entrypoint.sh
#COPY launch /opt/robomaster/launch

#ENTRYPOINT ["/opt/robomaster/ros_entrypoint.sh"]
#CMD ["/bin/bash"]

