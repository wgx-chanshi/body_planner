#include "rviz_interface.h"

void RVizInterface::bodyPlanCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  // Initialize Path message to visualize body plan
  // nav_msgs::msg::Path body_plan_viz;
  body_plan_viz.header = msg->header;

  // Loop through the BodyPlan message to get the state info and add to private
  // vector
  // int length = msg->states.size();
  // for (int i = 0; i < length; i++) {

  //   // Load in the pose data directly from the Odometry message
  //   geometry_msgs::msg::PoseStamped pose_stamped;
  //   pose_stamped.header = msg->states[i].header;
  //   pose_stamped.pose = msg->states[i].pose.pose;

  //   // Add to the path message
  //   body_plan_viz.poses.push_back(pose_stamped);
  // }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.pose = msg->pose.pose;

  // Add to the path message
  body_plan_viz.poses.push_back(pose_stamped);

  // Publish the full path
  body_plan_viz_pub_->publish(body_plan_viz);
}

// void RVizInterface::bodyPlanCallback(
//     const planner_msg::msg::BodyPlan::ConstSharedPtr msg) {

//   // Initialize Path message to visualize body plan
//   nav_msgs::msg::Path body_plan_viz;
//   body_plan_viz.header = msg->header;

//   // Loop through the BodyPlan message to get the state info and add to
//   private
//   // vector
//   int length = msg->states.size();
//   for (int i = 0; i < length; i++) {

//     // Load in the pose data directly from the Odometry message
//     geometry_msgs::msg::PoseStamped pose_stamped;
//     pose_stamped.header = msg->states[i].header;
//     pose_stamped.pose = msg->states[i].pose.pose;

//     // Add to the path message
//     body_plan_viz.poses.push_back(pose_stamped);
//   }

//   // Publish the full path
//   body_plan_viz_pub_->publish(body_plan_viz);
// }

void RVizInterface::discreteBodyPlanCallback(
    const planner_msg::msg::BodyPlan::ConstSharedPtr msg) {

  // Construct Marker message
  visualization_msgs::msg::Marker discrete_body_plan;

  // Initialize the headers and types
  discrete_body_plan.header = msg->header;
  discrete_body_plan.id = 0;
  discrete_body_plan.type = visualization_msgs::msg::Marker::POINTS;

  // Define the shape of the discrete states
  double scale = 0.2;
  discrete_body_plan.scale.x = scale;
  discrete_body_plan.scale.y = scale;
  discrete_body_plan.scale.z = scale;
  discrete_body_plan.color.r = 0.733f;
  discrete_body_plan.color.a = 1.0;

  // Loop through the discrete states
  int length = msg->states.size();
  for (int i = 0; i < length; i++) {
    geometry_msgs::msg::Point p;
    p.x = msg->states[i].pose.pose.position.x;
    p.y = msg->states[i].pose.pose.position.y;
    p.z = msg->states[i].pose.pose.position.z;
    discrete_body_plan.points.push_back(p);
  }

  // Publish both interpolated body plan and discrete states
  discrete_body_plan_viz_pub_->publish(discrete_body_plan);
}