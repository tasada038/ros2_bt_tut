#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/bt_tut_02.hpp"

#define DEFAULT_BT_XML "/home/ubuntu/dev_ws/src/ros2_bt_tut/config/bt_tut_02_tree.xml"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  auto tree = factory.createTreeFromFile(DEFAULT_BT_XML);

  NodeStatus status = NodeStatus::RUNNING;
  while(rclcpp::ok() && status == NodeStatus::RUNNING){
    status = tree.tickRoot();
  }

  return 0;
}