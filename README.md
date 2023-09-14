# body_planner

## install dependencies
1.sudo apt install ros-humble-tf2

2.sudo apt install ros-humble-tf2-geometry-msgs

3.sudo apt install ros-humble-grid-map

4.sudo apt install ros-humble-rviz

## build 
git clone git@github.com:wgx-chanshi/body_planner.git

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

## run
### run planner
source install/setup.bash

ros2 launch global_body_planner global_body_planner_launch.py 

### run rviz
source install/setup.bash

ros2 launch robot_visualization rviz.launch.py
