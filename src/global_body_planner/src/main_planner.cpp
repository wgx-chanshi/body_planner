#include "global_planner/global_body_planner.h"
#include "global_planner/rviz_interface.h"
#include "global_planner/terrain_map_publisher.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto pub_terrain_map_node = std::make_shared<TerrainMapPublisher>();
  auto pub_body_planner_node = std::make_shared<GlobalBodyPlanner>();
  auto rviz_interface_node = std::make_shared<RVizInterface>();

  executor.add_node(pub_terrain_map_node);
  executor.add_node(pub_body_planner_node);
  executor.add_node(rviz_interface_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}