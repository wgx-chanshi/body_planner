#include "terrain_map_publisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

void TerrainMapPublisher::createMap() {

  // Set initial map parameters and geometry
  terrain_map_.setFrameId(map_frame_);
  terrain_map_.setGeometry(grid_map::Length(12.0, 5.0), 0.2,
                           grid_map::Position(4.0, 0.0));
  // ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  //          terrain_map_.getLength().x(), terrain_map_.getLength().y(),
  //          terrain_map_.getSize()(0), terrain_map_.getSize()(1));

  // Add an obstacle
  double obs_center[] = {2, 0};
  double obs_radius = 0.5;
  for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {

    grid_map::Position position;
    terrain_map_.getPosition(*it, position);
    double x_diff = position.x() - obs_center[0];
    double y_diff = position.y() - obs_center[1];

    if (x_diff * x_diff + y_diff * y_diff <= obs_radius * obs_radius) {
      terrain_map_.at("elevation", *it) = 0.1;
    } else {
      terrain_map_.at("elevation", *it) = 0.0;
    }

    terrain_map_.at("dx", *it) = 0.0;
    terrain_map_.at("dy", *it) = 0.0;
    terrain_map_.at("dz", *it) = 1.0;
  }
}

std::vector<std::vector<double>>
TerrainMapPublisher::loadCSV(std::string filename) {

  std::vector<std::vector<double>> data;
  std::ifstream inputFile(filename);
  int l = 0;

  while (inputFile) {
    l++;
    std::string s;
    if (!getline(inputFile, s))
      break;
    if (s[0] != '#') {
      std::istringstream ss(s);
      std::vector<double> record;

      while (ss) {
        std::string line;
        if (!getline(ss, line, ','))
          break;
        try {
          record.push_back(stod(line));
        } catch (const std::invalid_argument e) {
          std::cout << "NaN found in file " << filename << " line " << l
                    << std::endl;
          e.what();
        }
      }

      data.push_back(record);
    }
  }

  if (!inputFile.eof()) {
    std::cerr << "Could not read file " << filename << "\n";
    std::__throw_invalid_argument("File not found.");
  }

  return data;
}

void TerrainMapPublisher::loadMapFromCSV() {
  // Load in all terrain data
  std::string package_path =
      ament_index_cpp::get_package_share_directory("global_body_planner");
  std::vector<std::vector<double>> x_data =
      loadCSV(package_path + "/data/" + terrain_type_ + "/xdata.csv");
  std::vector<std::vector<double>> y_data =
      loadCSV(package_path + "/data/" + terrain_type_ + "/ydata.csv");
  std::vector<std::vector<double>> z_data =
      loadCSV(package_path + "/data/" + terrain_type_ + "/zdata.csv");
  std::vector<std::vector<double>> dx_data =
      loadCSV(package_path + "/data/" + terrain_type_ + "/dxdata.csv");
  std::vector<std::vector<double>> dy_data =
      loadCSV(package_path + "/data/" + terrain_type_ + "/dydata.csv");
  std::vector<std::vector<double>> dz_data =
      loadCSV(package_path + "/data/" + terrain_type_ + "/dzdata.csv");

  // Grab map length and resolution parameters, make sure resolution is square
  // (and align grid centers with data points)
  int x_size = z_data[0].size();
  int y_size = z_data.size();
  float x_res = x_data[0][1] - x_data[0][0];
  float y_res = y_data[1][0] - y_data[0][0];
  double x_length = x_data[0].back() - x_data[0].front() + x_res;
  double y_length = y_data.back()[0] - y_data.front()[0] + y_res;
  if (x_res != y_res) {
    throw std::runtime_error("Map did not have square elements, make sure x "
                             "and y resolution are equal.");
  }

  // Initialize the map
  terrain_map_.setFrameId(map_frame_);
  terrain_map_.setGeometry(
      grid_map::Length(x_length, y_length), x_res,
      grid_map::Position(x_data[0].front() - 0.5 * x_res + 0.5 * x_length,
                         y_data.front()[0] - 0.5 * y_res + 0.5 * y_length));
  // ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  //          terrain_map_.getLength().x(), terrain_map_.getLength().y(),
  //          terrain_map_.getSize()(0), terrain_map_.getSize()(1));
  RCLCPP_INFO(get_logger(),
              "Created map with size %f x %f m (%i x %i cells).\n"
              " The center of the map is located at (%f, %f) in the %s frame.",
              terrain_map_.getLength().x(), terrain_map_.getLength().y(),
              terrain_map_.getSize()(0), terrain_map_.getSize()(1),
              terrain_map_.getPosition().x(), terrain_map_.getPosition().y(),
              terrain_map_.getFrameId().c_str());

  // Load in the elevation and slope data
  for (grid_map::GridMapIterator iterator(terrain_map_); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position position;
    terrain_map_.getPosition(*iterator, position);

    terrain_map_.at("elevation", *iterator) =
        z_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
    terrain_map_.at("dx", *iterator) =
        dx_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
    terrain_map_.at("dy", *iterator) =
        dy_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
    terrain_map_.at("dz", *iterator) =
        dz_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
  }
}

void TerrainMapPublisher::loadMapFromImage(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {

  // Initialize the map from the image message if not already done so
  if (!map_initialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(*msg, resolution_,
                                                       terrain_map_);
    // ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).",
    //          terrain_map_.getLength().x(), terrain_map_.getLength().y(),
    //          terrain_map_.getSize()(0), terrain_map_.getSize()(1));
    map_initialized_ = true;
  }

  // Add the data layers
  grid_map::GridMapRosConverter::addLayerFromImage(
      *msg, "elevation", terrain_map_, min_height_, max_height_);
  grid_map::GridMapRosConverter::addColorLayerFromImage(*msg, "color",
                                                        terrain_map_);

  // Add in slope information
  for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    terrain_map_.at("dx", *it) = 0.0;
    terrain_map_.at("dy", *it) = 0.0;
    terrain_map_.at("dz", *it) = 1.0;
  }

  // Move the map to place starting location at (0,0)
  grid_map::Position offset = {4.5, 0.0};
  terrain_map_.setPosition(offset);
}

void TerrainMapPublisher::publishMap() {
  // Set the time at which the map was published
  rclcpp::Time time = this->now();
  terrain_map_.setTimestamp(time.nanoseconds());

  // Generate grid_map message, convert, and publish
  std::unique_ptr<grid_map_msgs::msg::GridMap> terrain_map_msg;
  terrain_map_msg = grid_map::GridMapRosConverter::toMessage(terrain_map_);
  terrain_map_pub_->publish(std::move(terrain_map_msg));
}

void TerrainMapPublisher::set_publish_option() {
  // Either wait for an image to show up on the topic or create a map from
  // scratch
  if (map_data_source_.compare("image") == 0) {
    // Spin until image message has been received and processed
    // boost::shared_ptr<sensor_msgs::msg::Image const> shared_image;
    // while ((shared_image == nullptr) && ros::ok()) {
    //   shared_image = ros::topic::waitForMessage<sensor_msgs::Image>(
    //       "/image_publisher/image", nh_);
    //   ros::spinOnce();
    // }
  } else if (map_data_source_.compare("csv") == 0) {
    loadMapFromCSV();
  } else {
    createMap();
  }

  std::chrono::microseconds ctrl_dt(static_cast<int>(0.001 * 1e6));
  publish_map_timer_ = this->create_wall_timer(
      ctrl_dt, std::bind(&TerrainMapPublisher::publishMap, this),
      callback_group_map_subscriber_);
}