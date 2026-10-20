cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "controller_manager_msgs"
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

# Add object_visualizer executable
add_executable(object_visualizer src/object_visualizer.cpp)
target_include_directories(object_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(object_visualizer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  object_visualizer
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometric_shapes"
)

# Add visual_servo executable
add_executable(visual_servo src/visual_servo.cpp)
target_include_directories(visual_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(visual_servo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  visual_servo
  "rclcpp"
  "controller_manager_msgs"
  "geometry_msgs"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

# Add action_dispatcher executable（统一调度 approach/grind/grasp）
add_executable(action_dispatcher src/action_dispatcher.cpp)
target_include_directories(action_dispatcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(action_dispatcher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  action_dispatcher
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "controller_manager_msgs"
  "geometry_msgs"
  "std_msgs"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

# Add bottle_visualizer executable（红瓶虚拟映射显示到 RViz）
add_executable(bottle_visualizer src/bottle_visualizer.cpp)
target_include_directories(bottle_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bottle_visualizer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  bottle_visualizer
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
  "geometric_shapes"
)

# Add bottle_insert executable（红瓶一次性插入：规划 -> 确认 -> 执行）
add_executable(bottle_insert src/bottle_insert.cpp)
target_include_directories(bottle_insert PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bottle_insert PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  bottle_insert
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "controller_manager_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

install(TARGETS hello_moveit object_visualizer visual_servo action_dispatcher
                bottle_visualizer bottle_insert
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

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
