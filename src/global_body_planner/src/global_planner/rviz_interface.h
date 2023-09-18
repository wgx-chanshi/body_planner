#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include "planner_msg/msg/body_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

//! A class for interfacing between RViz and planning topics.
/*!
   RVizInterface is a container for all of the logic utilized in the template
   node. The implementation must provide a clean and high level interface to the
   core algorithm
*/
class RVizInterface : public rclcpp::Node {
public:
  /**
   * @brief Constructor for RVizInterface Class
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type RVizInterface
   */
  RVizInterface() : Node("RVizInterface") {
    map_frame_ = "map";

    callback_group_body_plan_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto body_plan_opt = rclcpp::SubscriptionOptions();
    body_plan_opt.callback_group = callback_group_body_plan_subscriber_;
    auto discrete_body_plan_opt = rclcpp::SubscriptionOptions();
    discrete_body_plan_opt.callback_group =
        callback_group_body_plan_subscriber_;

    // Setup pubs and subs
    body_plan_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/target_pos", rclcpp::QoS(10),
        std::bind(&RVizInterface::bodyPlanCallback, this,
                  std::placeholders::_1),
        body_plan_opt);

    discrete_body_plan_sub_ =
        this->create_subscription<planner_msg::msg::BodyPlan>(
            "/discrete_body_plan", rclcpp::QoS(10),
            std::bind(&RVizInterface::discreteBodyPlanCallback, this,
                      std::placeholders::_1),
            discrete_body_plan_opt);

    body_plan_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/visualization/body_plan", 10);

    discrete_body_plan_viz_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization/discrete_body_plan", 10);
  }

  ~RVizInterface() {}

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_body_plan_subscriber_;

  /**
   * @brief Callback function to handle new body plan data
   * @param[in] Body plan message contining interpolated output of body planner
   */
  void bodyPlanCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * @brief Callback function to handle new body plan discrete state data
   * @param[in] Body plan message contining discrete output of body planner
   */
  void discreteBodyPlanCallback(
      const planner_msg::msg::BodyPlan::ConstSharedPtr msg);

  /// ROS subscriber for the body plan
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_plan_sub_;

  /// ROS subscriber for the body plan
  rclcpp::Subscription<planner_msg::msg::BodyPlan>::SharedPtr
      discrete_body_plan_sub_;

  /// ROS Publisher for the interpolated body plan vizualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_plan_viz_pub_;

  /// ROS Publisher for the discrete body plan vizualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      discrete_body_plan_viz_pub_;

  /// Update rate for sending and receiving data, unused since pubs are called
  /// in callbacks
  double update_rate_;

  /// Handle for the map frame
  std::string map_frame_;

  nav_msgs::msg::Path body_plan_viz;
};

#endif // RVIZ_INTERFACE_H
