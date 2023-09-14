#ifndef TERRAIN_MAP_PUBLISHER_H
#define TERRAIN_MAP_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <fstream>  // ifstream
#include <iostream> // cout
#include <sstream>  //istringstream
#include <string>
#include <vector>

//! A terrain map publishing class
/*!
   TerrainMapPublisher is a class for publishing terrain maps from a variety of
   sources, including from scratch.
*/
class TerrainMapPublisher : public rclcpp::Node {
public:
  TerrainMapPublisher()
      : Node("TerrainMapPublisher"),
        terrain_map_({"elevation", "dx", "dy", "dz"}) {
    map_data_source_ = "csv";
    terrain_type_ = "rough_terrain";
    map_frame_ = "map";

    // Setup pubs and subs
    terrain_map_pub_ =
        this->create_publisher<grid_map_msgs::msg::GridMap>("/terrain_map", 10);

    callback_group_map_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto map_opt = rclcpp::SubscriptionOptions();
    map_opt.callback_group = callback_group_map_subscriber_;

    // Add image subscriber if data source requests an image
    if (map_data_source_.compare("image") == 0) {
      resolution_ = 0.2;
      min_height_ = 0.0;
      max_height_ = 1.0;
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/image_publisher/image", rclcpp::QoS(10),
          std::bind(&TerrainMapPublisher::loadMapFromImage, this,
                    std::placeholders::_1),
          map_opt);
    }

    // Initialize the elevation layer on the terrain map
    terrain_map_.setBasicLayers({"elevation", "dx", "dy", "dz"});

    set_publish_option();
  }

  ~TerrainMapPublisher() {}

  /**
   * @brief Creates the map object from scratch
   */
  void createMap();

  /**
   * @brief Loads data from a specified CSV file into a nested std::vector
   * structure
   * @param[in] filename Path to the CSV file
   * @return Data from the CSV in vector structure
   */
  std::vector<std::vector<double>> loadCSV(std::string filename);

  /**
   * @brief Loads data into the map object from a CSV
   */
  void loadMapFromCSV();

  /**
   * @brief Publishes map data to the terrain_map topic
   */
  void publishMap();

private:
  /// ROS Subscriber for image data
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  /// ROS Publisher for the terrain map
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_pub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_map_subscriber_;

  rclcpp::TimerBase::SharedPtr publish_map_timer_;

  void set_publish_option();

  /**
   * @brief Loads data into the map object from an image topic
   * @param[in] msg ROS image message
   */
  void loadMapFromImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  /// Update rate for sending and receiving data, unused since pubs are called
  /// in callbacks
  double update_rate_;

  /// Handle for the map frame
  std::string map_frame_;

  /// GridMap object for terrain data
  grid_map::GridMap terrain_map_;

  /// String of the source of the terrain map data
  std::string map_data_source_;

  /// String of terrain type (if loading from csv)
  std::string terrain_type_;

  /// Bool to flag if the map has been initialized yet
  bool map_initialized_ = false;

  /// Double for map resolution
  double resolution_;

  /// Double for map resolution
  double min_height_;

  /// Double for map resolution
  double max_height_;
};

#endif // TERRAIN_MAP_PUBLISHER_H
