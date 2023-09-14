#ifndef GLOBAL_BODY_PLANNER_H
#define GLOBAL_BODY_PLANNER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fast_terrain_map.h"
#include "planner_msg/msg/body_plan.hpp"

#include "planner_class.h"
#include "planning_utils.h"
#include "rrt_star_connect.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace planning_utils;

//! A global body planning class for legged robots
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global
   body planning node. This algorithm requires an height map of the terrain as a
   GridMap message type, and will publish the global body plan as a BodyPlan
   message over a topic. It will also publish the discrete states used by the
   planner (from which the full path is interpolated).
*/
class GlobalBodyPlanner : public rclcpp::Node {
public:
  GlobalBodyPlanner() : Node("GlobalBodyPlanner") {
    first_run_planner_ = true;
    target_change_ = false;
    num_calls_ = 1;
    replan_time_limit_ = 1.0;
    map_frame_ = "map";
    algorithm_ = "rrt-connect";
    target_pos_.resize(3);
    start_pos_.resize(3);
    start_pos_[0] = 0.0;
    start_pos_[1] = 0.0;
    start_pos_[2] = 0.4;
    target_pos_[0] = start_pos_[0] + 8.0;
    target_pos_[1] = start_pos_[1];
    target_pos_[2] = start_pos_[2];

    callback_group_terrain_map_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto terrain_map_opt = rclcpp::SubscriptionOptions();
    terrain_map_opt.callback_group = callback_group_terrain_map_subscriber_;
    auto sub_target_pos_rviz_opt = rclcpp::SubscriptionOptions();
    sub_target_pos_rviz_opt.callback_group =
        callback_group_terrain_map_subscriber_;

    // Setup pubs and subs
    terrain_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain_map", rclcpp::QoS(10),
        std::bind(&GlobalBodyPlanner::terrainMapCallback, this,
                  std::placeholders::_1),
        terrain_map_opt);

    rviz_target_pos_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::QoS(10),
            std::bind(&GlobalBodyPlanner::rviz_target_pos_subscription_cb, this,
                      std::placeholders::_1),
            sub_target_pos_rviz_opt);

    body_plan_pub_ =
        this->create_publisher<planner_msg::msg::BodyPlan>("/body_plan", 10);

    discrete_body_plan_pub_ =
        this->create_publisher<planner_msg::msg::BodyPlan>(
            "/discrete_body_plan", 10);
  }

  ~GlobalBodyPlanner() {}

  /**
   * @brief Call the correct planning class and compute statistics
   */
  void callPlanner();

private:
  /**
   * @brief Callback function to handle new terrain map data
   * @param[in] msg the message contining map data
   */
  void
  terrainMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr callback_group_terrain_map_subscriber_;

  /**
   * @brief Set the start and goal states of the planner
   */
  void setStartAndGoalStates();

  /**
   * @brief Clear the plan member variables
   */
  void clearPlan();

  /**
   * @brief Update the body plan with the current plan
   * @param[in] t Time of state in trajectory
   * @param[in] body_state Body state
   * @param[in] body_plan_msg Body plan message
   */
  void addBodyStateToMsg(double t, State body_state,
                         planner_msg::msg::BodyPlan &body_plan_msg);

  void rviz_target_pos_subscription_cb(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  /**
   * @brief Publish the current body plan
   */
  void publishPlan();

  /// Subscriber for terrain map messages
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_sub_;

  // Subscriber for rviz target pos messages
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      rviz_target_pos_subscription_;

  /// Publisher for body plan messages
  rclcpp::Publisher<planner_msg::msg::BodyPlan>::SharedPtr body_plan_pub_;

  /// Publisher for discrete states in body plan messages
  rclcpp::Publisher<planner_msg::msg::BodyPlan>::SharedPtr
      discrete_body_plan_pub_;

  bool first_run_planner_;

  bool target_change_;

  /// Number of times to call the planner
  int num_calls_;

  /// Algorithm for planner to run (rrt-connect or rrt-star-connect)
  std::string algorithm_;

  /// Time after which replanning is halted;
  double replan_time_limit_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Struct for terrain map data
  FastTerrainMap terrain_;

  /// Std vector containing the interpolated robot body plan
  std::vector<State> body_plan_;

  /// Std vector containing the interpolated time data
  std::vector<double> t_plan_;

  std::vector<double> target_pos_;

  std::vector<double> start_pos_;

  /// Robot starting state
  State robot_start_;

  /// Robot goal state
  State robot_goal_;

  /// Sequence of discrete states in the plan
  std::vector<State> state_sequence_;

  /// Sequence of discrete actions in the plan
  std::vector<Action> action_sequence_;

  /// Vector of cost instances in each planning call (nested STL vectors)
  std::vector<std::vector<double>> cost_vectors_;

  /// Vector of time instances of cost data for each planning call (nested STL
  /// vectors)
  std::vector<std::vector<double>> cost_vectors_times_;

  /// Vector of solve times for each planning call
  std::vector<double> solve_time_info_;

  /// Vector of number of vertices for each planning call
  std::vector<int> vertices_generated_info_;
};

#endif // GLOBAL_BODY_PLANNER_H
