cmake_minimum_required(VERSION 3.8)
project(ros2_bt_tut)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
)  

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(bt_tut_01 src/bt_tut_01.cpp)
add_executable(bt_tut_02 src/bt_tut_02.cpp src/bt_tut_02_component.cpp)
add_executable(bt_tut_03 src/bt_tut_03.cpp)
add_executable(bt_tut_05 src/bt_tut_05.cpp src/crossdoor_nodes.cpp)
add_executable(bt_tut_05_v2 src/bt_tut_05_v2.cpp src/crossdoor_nodes.cpp)
add_executable(bt_tut_06 src/bt_tut_06.cpp src/movebase_node.cpp src/bt_tut_02_component.cpp)
add_executable(bt_tut_07 src/bt_tut_07.cpp src/bt_tut_02_component.cpp)
add_executable(bt_tut_08 src/bt_tut_08.cpp)
add_executable(bt_tut_09 src/bt_tut_09.cpp)

ament_target_dependencies(bt_tut_01 ${dependencies})
ament_target_dependencies(bt_tut_02 ${dependencies})
ament_target_dependencies(bt_tut_03 ${dependencies})
ament_target_dependencies(bt_tut_05 ${dependencies})
ament_target_dependencies(bt_tut_05_v2 ${dependencies})
ament_target_dependencies(bt_tut_06 ${dependencies})
ament_target_dependencies(bt_tut_07 ${dependencies})
ament_target_dependencies(bt_tut_08 ${dependencies})
ament_target_dependencies(bt_tut_09 ${dependencies})

install(TARGETS
  bt_tut_01
  bt_tut_02
  bt_tut_03
  bt_tut_05
  bt_tut_05_v2
  bt_tut_06
  bt_tut_07
  bt_tut_08
  bt_tut_09
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
