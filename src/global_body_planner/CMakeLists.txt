cmake_minimum_required(VERSION 3.8)
project(global_body_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(planner_msg REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_subdirectory(src/global_planner)

set(BIN_NAME "main_planner")
add_executable(${BIN_NAME} src/main_planner.cpp)

set(LIBS body_planner)
ament_target_dependencies(main_planner rclcpp std_msgs grid_map_core grid_map_ros planner_msg)
target_link_libraries(${BIN_NAME} ${LIBS})

install(TARGETS
  ${BIN_NAME}
  ${LIBS}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY data DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
