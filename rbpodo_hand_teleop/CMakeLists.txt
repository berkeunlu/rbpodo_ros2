cmake_minimum_required(VERSION 3.8)
project(rbpodo_hand_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rbpodo_msgs REQUIRED)

# Original hand teleop node (uses move_l action - slower)
add_executable(hand_teleop_node src/hand_teleop_node.cpp)
ament_target_dependencies(hand_teleop_node
  rclcpp
  rclcpp_action
  sensor_msgs
  cv_bridge
  rbpodo_msgs
)
target_link_libraries(hand_teleop_node ${OpenCV_LIBS})

# NEW: Streaming hand teleop node (uses servo_l via eval - faster, real-time)
add_executable(hand_teleop_streaming_node src/hand_teleop_streaming_node.cpp)
ament_target_dependencies(hand_teleop_streaming_node
  rclcpp
  geometry_msgs
  std_msgs
  rbpodo_msgs
)

install(TARGETS 
  hand_teleop_node
  hand_teleop_streaming_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mac_camera_bridge.py
  scripts/hand_position_bridge.py
  scripts/mediapipe_hand_server.py
  scripts/hand_teleop_streaming.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
