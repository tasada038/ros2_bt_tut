#ifndef ROS2_BT_TUT__BT_TUT_08_HPP_
#define ROS2_BT_TUT__BT_TUT_08_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;


// Action_A has a different constructor than the default one.
class Action_A : public SyncActionNode
{
public:
  // additional arguments passed to the constructor
  Action_A(const std::string& name, const NodeConfiguration& config,
           int arg1, double arg2, std::string arg3) :
    SyncActionNode(name, config), _arg1(arg1), _arg2(arg2), _arg3(arg3)
  {
    node_ = rclcpp::Node::make_shared("action_a");
    int_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/action/int",10);
    float_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/action/float",10);
    string_pub_ = node_->create_publisher<std_msgs::msg::String>("/action/string",10);
  }

  NodeStatus tick() override
  {
    std::cout << "Action_A: " << _arg1 << " / " << _arg2 << " / " << _arg3 << std::endl;
    std_msgs::msg::Int32 int_msg;
    std_msgs::msg::Float32 float_msg;
    std_msgs::msg::String string_msg;
    int_msg.data = _arg1;
    float_msg.data = _arg2;
    string_msg.data = _arg3;
    RCLCPP_INFO(node_->get_logger(),
                "arg1, arg2, arg3: [%d, %f, %s]",
                int_msg.data, float_msg.data, string_msg.data.c_str());
    int_pub_->publish(int_msg);
    float_pub_->publish(float_msg);
    string_pub_->publish(string_msg);
    
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts()
  {
    return {};
  }

private:
  int _arg1;
  double _arg2;
  std::string _arg3;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
};

// Action_B implements an init(...) method that must be called once at the beginning.
class Action_B : public SyncActionNode
{
public:
  Action_B(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config)
  {}

  // we want this method to be called ONCE and BEFORE the first tick()
  void init(int arg1, double arg2, std::string arg3)
  {
    _arg1 = (arg1);
    _arg2 = (arg2);
    _arg3 = (arg3);
  }

  NodeStatus tick() override
  {
    std::cout << "Action_B: " << _arg1 << " / " << _arg2 << " / " << _arg3 << std::endl;
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts()
  {
    return {};
  }

private:
  int _arg1;
  double _arg2;
  std::string _arg3;
};

#endif //ROS2_BT_TUT__BT_TUT_08_HPP_