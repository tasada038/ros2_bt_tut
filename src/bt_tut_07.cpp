#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/bt_tut_02.hpp"

#include <filesystem>
#include <iostream>

// #define MAIN_BT_XML "/home/ubuntu/dev_ws/src/ros2_bt_tut/config/bt_tut_07_tree.xml"

using namespace BT;

int main(int argc, char **argv)
{
  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/src/ros2_bt_tut/config/bt_tut_07_tree.xml";
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv); 
  BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");

  // Register the behavior tree definitions, but do not instantiate them yet.
  // Order is not important.
  //factory.registerBehaviorTreeFromText(xml_text_subA);
  //factory.registerBehaviorTreeFromText(xml_text_subB);
  //factory.registerBehaviorTreeFromText(MAIN_BT_XML);

  factory.registerBehaviorTreeFromFile(xml_path);

  auto tree = factory.createTreeFromFile(xml_path);
  NodeStatus status = NodeStatus::RUNNING;
  while(rclcpp::ok() && status == NodeStatus::RUNNING){
    status = tree.tickRoot();
  }

  //Check that the BTs have been registered correctly
  std::cout << "Registered BehaviorTrees:" << std::endl;
  for (const std::string& bt_name : factory.registeredBehaviorTrees())
  {
    std::cout << " - " << bt_name << std::endl;
  }

  // You can create the MainTree and the subtrees will be added automatically.
  //std::cout << "----- MainTree tick ----" << std::endl;
  //auto main_tree = factory.createTree("MainTree");
  //main_tree.tickRootWhileRunning();

  // ... or you can create only one of the subtree
  //std::cout << "----- SubA tick ----" << std::endl;
  //auto subA_tree = factory.createTree("SubA");
  //subA_tree.tickRootWhileRunning();

  return 0;
}