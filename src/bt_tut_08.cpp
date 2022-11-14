#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/bt_tut_08.hpp"
#include <filesystem>
#include <iostream>

int main(int argc, char **argv)
{
  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/src/ros2_bt_tut/config/bt_tut_08_tree.xml";
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv); 
  BehaviorTreeFactory factory;

  // A node builder is nothing more than a function pointer to create a
  // std::unique_ptr<TreeNode>.
  // Using lambdas or std::bind, we can easily "inject" additional arguments.
  NodeBuilder builder_A = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<Action_A>(name, config, 42, 3.14, "hello world");
  };

  // BehaviorTreeFactory::registerBuilder is the more general way to register a custom node.
  // Not the most user friendly, but definitely the most flexible one.
  factory.registerBuilder<Action_A>("Action_A", builder_A);

  // The regitration of  Action_B is done as usual, but we still need to call Action_B::init()
  factory.registerNodeType<Action_B>("Action_B");

  auto tree = factory.createTreeFromFile(xml_path);

  // Iterate through all the nodes and call init if it is an Action_B
  for (auto& node : tree.nodes)
  {
    if (auto action_B_node = dynamic_cast<Action_B*>(node.get()))
    {
      action_B_node->init(69, 9.99, "interesting_value");
    }
  }

  NodeStatus status = NodeStatus::RUNNING;
  while(rclcpp::ok() && status == NodeStatus::RUNNING){
    status = tree.tickRoot();
  }
  return 0;
}