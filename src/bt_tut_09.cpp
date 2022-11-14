#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/bt_tut_09.hpp"
#include <filesystem>
#include <iostream>

int main(int argc, char **argv)
{
  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/src/ros2_bt_tut/config/bt_tut_09_tree.xml";
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv); 
  BehaviorTreeFactory factory;
  factory.registerNodeType<MyAsyncAction>("MyAsyncAction");
  auto tree = factory.createTreeFromFile(xml_path);
  NodeStatus status = NodeStatus::RUNNING;
  while(rclcpp::ok() && status == NodeStatus::RUNNING){
    status = tree.tickRoot();
  }
  return 0;
}