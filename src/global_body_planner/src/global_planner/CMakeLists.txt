find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(planner_msg REQUIRED)

add_library(body_planner SHARED
    fast_terrain_map.cpp
    fast_terrain_map.h
    planning_utils.cpp
    planning_utils.h
    graph_class.cpp
    graph_class.h
    planner_class.cpp
    planner_class.h
    rotate_grf.h
    rrt.cpp
    rrt.h
    rrt_connect.cpp
    rrt_connect.h
    rrt_star_connect.cpp
    rrt_star_connect.h
    terrain_map_publisher.cpp
    terrain_map_publisher.h
    global_body_planner.cpp
    global_body_planner.h
    rviz_interface.cpp
    rviz_interface.h
)

ament_target_dependencies(body_planner rclcpp std_msgs tf2_geometry_msgs planner_msg grid_map_core grid_map_ros)
