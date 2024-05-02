#!/bin/bash

cd /media/psf/ubuntuSharedFolder/part_II_project

tab=" --tab"
options=()

cmds[1]="source install/setup.bash && source /home/joshuabird/Documents/webots_ros2/install/setup.bash && ros2 launch webots_sim robot_launch.py"

cmds[2]="source install/setup.bash && ros2 run central_management_interface my_node"

cmds[3]="source install/setup.bash && ros2 run slam_system ros_mono --ros-args -p agentId:=1 -p config:=/home/joshuabird/Desktop/orbslam_configs/configs/euroc.yaml"

cmds[4]="source install/setup.bash && ros2 run slam_system ros_mono --ros-args -p agentId:=2 -p config:=/home/joshuabird/Desktop/orbslam_configs/configs/euroc.yaml"


for i in 1 2 3 4; do
options+=($tab -e "bash -c '${cmds[i]} ; bash'" )
done

gnome-terminal "${options[@]}"

exit 0