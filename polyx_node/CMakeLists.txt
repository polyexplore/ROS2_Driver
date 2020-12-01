cmake_minimum_required(VERSION 3.5)
project(polyx_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# interface
find_package(rosidl_default_generators REQUIRED)

# include header directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  # ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# set msgs(interfaces)
set(msg_files
  "msg/CorrectedIMU.msg"
  "msg/Kalman.msg"
  "msg/StaticGeoPoseEvent.msg"
  "msg/Dmi.msg"
  "msg/LeapSeconds.msg"
  "msg/StaticHeadingEvent.msg"
  "msg/EulerAttitude.msg"
  "msg/NmeaGGA.msg"
  "msg/TimeSync.msg"
  "msg/Geoid.msg"
  "msg/RawIMU.msg"
  "msg/WheelSpeedEvent.msg"
  "msg/CompactNav.msg"
  "msg/SolutionStatus.msg"
)

set(dependencies
  "rclcpp" 
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "geographic_msgs"
)

# generate msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}_talker 
  src/${PROJECT_NAME}_talker_main.cpp
  src/polyx_nmea.cpp
  src/polyx_convert.cpp
  src/polyx_node_talker.cpp
)
add_executable(polyx_node_static_heading
  src/polyx_node_static_heading.cpp
)
add_executable(polyx_node_wheel_speed
  src/polyx_node_wheel_speed.cpp
)
add_executable(polyx_node_listener
  src/polyx_node_listener.cpp
)
add_executable(polyx_node_static_geopose
  src/polyx_node_static_geopose.cpp
)
ament_target_dependencies(${PROJECT_NAME}_talker 
  ${dependencies}
)
ament_target_dependencies(polyx_node_static_heading
  ${dependencies}
)
ament_target_dependencies(polyx_node_wheel_speed
  ${dependencies}
)
ament_target_dependencies(polyx_node_listener
  ${dependencies}
)
ament_target_dependencies(polyx_node_static_geopose
  ${dependencies}
)
#### installing
# Install executables
install(TARGETS
  ${PROJECT_NAME}_talker
  polyx_node_static_heading
  polyx_node_wheel_speed
  polyx_node_listener
  polyx_node_static_geopose
  DESTINATION lib/${PROJECT_NAME}
)
rosidl_target_interfaces(${PROJECT_NAME}_talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(polyx_node_static_heading
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(polyx_node_wheel_speed
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(polyx_node_listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(polyx_node_static_geopose
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install parameters
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
