#include "global_body_planner.h"

using namespace planning_utils;

void GlobalBodyPlanner::terrainMapCallback(
    const grid_map_msgs::msg::GridMap::ConstSharedPtr msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);

  // Update the plan
  if (first_run_planner_ || target_change_) {
    callPlanner();
    iter_plan_ = 0;
    first_run_planner_ = false;
    target_change_ = false;
  }

  // publishPlan();

  pubForController();
}

void GlobalBodyPlanner::clearPlan() {
  // Clear old solutions
  body_plan_.clear();
  t_plan_.clear();
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vectors_.clear();
  cost_vectors_times_.clear();
}

void GlobalBodyPlanner::callPlanner() {
  std::cout << "call planner" << std::endl;
  // Get the most recent plan parameters and clear the old solutions
  setStartAndGoalStates();

  clearPlan();

  // Initialize statistics variables
  double plan_time;
  int success;
  int vertices_generated;
  double time_to_first_solve;
  double path_duration;
  double total_solve_time = 0;
  double total_vertices_generated = 0;
  double total_path_duration = 0;

  // Set up more objects
  cost_vectors_.reserve(num_calls_);
  cost_vectors_times_.reserve(num_calls_);
  RRTClass rrt_obj;
  RRTConnectClass rrt_connect_obj;
  RRTStarConnectClass rrt_star_connect_obj;

  // Loop through num_calls_ planner calls
  for (int i = 0; i < num_calls_; ++i) {
    // Clear out previous solutions and initialize new statistics variables
    state_sequence_.clear();
    action_sequence_.clear();
    std::vector<double> cost_vector;
    std::vector<double> cost_vector_times;

    // Call the appropriate planning method (either RRT-Connect or RRT*-Connect)
    if (algorithm_.compare("rrt-connect") == 0) {
      rrt_connect_obj.buildRRTConnect(terrain_, robot_start_, robot_goal_,
                                      state_sequence_, action_sequence_,
                                      replan_time_limit_);
      rrt_connect_obj.getStatistics(plan_time, success, vertices_generated,
                                    time_to_first_solve, cost_vector,
                                    cost_vector_times, path_duration);
    } else if (algorithm_.compare("rrt-star-connect") == 0) {
      rrt_star_connect_obj.buildRRTStarConnect(
          terrain_, robot_start_, robot_goal_, state_sequence_,
          action_sequence_, replan_time_limit_);
      rrt_star_connect_obj.getStatistics(plan_time, success, vertices_generated,
                                         time_to_first_solve, cost_vector,
                                         cost_vector_times, path_duration);
    } else {
      throw std::runtime_error("Invalid algorithm specified");
    }

    // Handle the statistical data
    cost_vectors_.push_back(cost_vector);
    cost_vectors_times_.push_back(cost_vector_times);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_duration += path_duration;

    std::cout << "Vertices generated: " << vertices_generated << std::endl;
    std::cout << "Solve time: " << plan_time << std::endl;
    std::cout << "Time to first solve: " << time_to_first_solve << std::endl;
    std::cout << "Path length: " << cost_vector.back() << std::endl;

    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);
  }

  // Report averaged statistics if num_calls_ > 1
  if (num_calls_ > 1) {
    std::cout << "Average vertices generated: "
              << total_vertices_generated / num_calls_ << std::endl;
    std::cout << "Average solve time: " << total_solve_time / num_calls_
              << std::endl;
    std::cout << "Average path duration: " << total_path_duration / num_calls_
              << std::endl;
  }

  // Interpolate to get full body plan
  double dt = 0.0002;
  std::vector<int> interp_phase;
  getInterpPath(state_sequence_, action_sequence_, dt, body_plan_, t_plan_,
                interp_phase);
}

void GlobalBodyPlanner::setStartAndGoalStates() {
  // Update any relevant planning parameters
  robot_start_ = {start_pos_[0], start_pos_[1], 0.4, 0, 0, 0, 0, 0};
  robot_goal_ = {target_pos_[0], target_pos_[1], 0.4, 0, 0, 0, 0, 0};

  robot_start_[2] += terrain_.getGroundHeight(robot_start_[0], robot_start_[1]);
  robot_goal_[2] += terrain_.getGroundHeight(robot_goal_[0], robot_goal_[1]);
}

void GlobalBodyPlanner::addBodyStateToMsg(double t, State body_state,
                                          planner_msg::msg::BodyPlan &msg) {

  // Make sure the timestamps match the trajectory timing
  // ros::Duration time_elapsed(t);
  // ros::Time current_time = msg.header.stamp + time_elapsed;
  rclcpp::Time current_time = this->now() + rclcpp::Duration::from_seconds(t);

  // Represent each state as an Odometry message
  nav_msgs::msg::Odometry state;
  state.header.frame_id = map_frame_;
  state.header.stamp = current_time;
  state.child_frame_id = "dummy";

  // Transform from RPY to quat msg
  tf2::Quaternion quat_tf;
  geometry_msgs::msg::Quaternion quat_msg;
  quat_tf.setRPY(0, body_state[6], atan2(body_state[4], body_state[3]));
  quat_msg = tf2::toMsg(quat_tf);

  // Load the data into the message
  state.pose.pose.position.x = body_state[0];
  state.pose.pose.position.y = body_state[1];
  state.pose.pose.position.z = body_state[2];
  state.pose.pose.orientation = quat_msg;

  state.twist.twist.linear.x = body_state[3];
  state.twist.twist.linear.y = body_state[4];
  state.twist.twist.linear.z = body_state[5];
  state.twist.twist.angular.x = 0;
  state.twist.twist.angular.y = body_state[6];
  state.twist.twist.angular.z = 0;

  msg.states.push_back(state);
}

void GlobalBodyPlanner::publishPlan() {
  // Construct BodyPlan messages
  planner_msg::msg::BodyPlan body_plan_msg;
  planner_msg::msg::BodyPlan discrete_body_plan_msg;

  // Initialize the headers and types
  rclcpp::Time timestamp = this->now();
  body_plan_msg.header.stamp = timestamp;
  body_plan_msg.header.frame_id = map_frame_;
  discrete_body_plan_msg.header = body_plan_msg.header;

  // Loop through the interpolated body plan and add to message
  for (int i = 0; i < body_plan_.size(); ++i)
    addBodyStateToMsg(t_plan_[i], body_plan_[i], body_plan_msg);

  // Loop through the discrete states and add to message
  for (int i = 0; i < state_sequence_.size(); i++)
    addBodyStateToMsg(t_plan_[i], state_sequence_[i], discrete_body_plan_msg);

  // Publish both interpolated body plan and discrete states
  body_plan_pub_->publish(body_plan_msg);
  discrete_body_plan_pub_->publish(discrete_body_plan_msg);
}

void GlobalBodyPlanner::pubForController() {
  nav_msgs::msg::Odometry state;
  State body_state;

  if (iter_plan_ < body_plan_.size() - 1) {
    body_state = body_plan_[iter_plan_];
    iter_plan_ = iter_plan_ + 1;
  } else {
    body_state = body_plan_[iter_plan_ - 1];
  }

  tf2::Quaternion quat_tf;
  geometry_msgs::msg::Quaternion quat_msg;
  quat_tf.setRPY(0, body_state[6], atan2(body_state[4], body_state[3]));
  quat_msg = tf2::toMsg(quat_tf);

  state.header.stamp = this->now();
  state.header.frame_id = map_frame_;

  state.pose.pose.position.x = body_state[0];
  state.pose.pose.position.y = body_state[1];
  state.pose.pose.position.z = body_state[2];
  state.pose.pose.orientation = quat_msg;

  state.twist.twist.linear.x = body_state[3];
  state.twist.twist.linear.y = body_state[4];
  state.twist.twist.linear.z = body_state[5];
  state.twist.twist.angular.x = 0;
  state.twist.twist.angular.y = body_state[6];
  state.twist.twist.angular.z = 0;

  // std::cout << "body plan size: " << body_plan_.size() << std::endl;

  robot_plan_pub_->publish(state);
}

void GlobalBodyPlanner::rviz_target_pos_subscription_cb(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  target_pos_[0] = msg->pose.position.x;
  target_pos_[1] = msg->pose.position.y;
  target_pos_[2] = msg->pose.position.z;

  target_change_ = true;
}

void GlobalBodyPlanner::mujoco_current_pos_subscription_cb(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  start_pos_[0] = msg->pose.pose.position.x;
  start_pos_[1] = msg->pose.pose.position.y;
  start_pos_[2] = msg->pose.pose.position.z;
}