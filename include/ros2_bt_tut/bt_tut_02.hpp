#ifndef ROS2_BT_TUT__BT_TUT_02_HPP_
#define ROS2_BT_TUT__BT_TUT_02_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode
{
public:
  // Constructor 
  SaySomething(const std::string& name, const NodeConfiguration& config);
  // STATIC method.
  static PortsList providedPorts();
  // Override the virtual function tick()
  virtual NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


class ThinkWhatToSay : public SyncActionNode
{
public:
  ThinkWhatToSay(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  // This Action writes a value into the port "text"
  virtual NodeStatus tick() override;
};

#endif //ROS2_BT_TUT__BT_TUT_02_HPP_