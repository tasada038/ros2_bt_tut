#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros2_bt_tut/crossdoor_nodes.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <filesystem>
#include <iostream>


int main(int argc, char **argv)
{

  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/src/ros2_bt_tut/config/crossdoor_behavior.xml";
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  CrossDoor cross_door;
  cross_door.registerNodes(factory);

  // the XML is the one shown at the beginning of the tutorial
  auto tree = factory.createTreeFromFile(xml_path);
  BT::PublisherZMQ publisher_zmq(tree);

  // helper function to print the tree
  BT::printTreeRecursively(tree.rootNode());

  // tree.tickRootWhileRunning();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING){
    status = tree.tickRoot();
  }

  return 0;
}
