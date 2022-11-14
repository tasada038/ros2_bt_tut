#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/bt_tut_02.hpp"

using namespace BT;

  // Constructor 
SaySomething::SaySomething(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("input");
  pub_ = node_->create_publisher<std_msgs::msg::String>("/input_string",10);
}

// It is mandatory to define this STATIC method.
PortsList SaySomething::providedPorts()
{
  // This action has a single input port called "message"
  return { InputPort<std::string>("message") };
}

// Override the virtual function tick()
NodeStatus SaySomething::tick()
{
  Optional<std::string> msg = getInput<std::string>("message");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
    throw RuntimeError("missing required input [message]: ", 
                            msg.error() );
  }
  auto msg_data = msg.value();
  // use the method value() to extract the valid message.
  std::cout << "Robot says: " << msg_data << std::endl;

  std_msgs::msg::String input_msg;
  input_msg.data = msg_data;
  RCLCPP_INFO(node_->get_logger(), "input data: %s", input_msg.data.c_str());
  pub_->publish(input_msg);

  return NodeStatus::SUCCESS;
}


ThinkWhatToSay::ThinkWhatToSay(const std::string& name, const NodeConfiguration& config)
  : SyncActionNode(name, config)
{ }

PortsList ThinkWhatToSay::providedPorts()
{
  return { OutputPort<std::string>("text") };
}

// This Action writes a value into the port "text"
NodeStatus ThinkWhatToSay::tick()
{
  // the output may change at each tick(). Here we keep it simple.
  setOutput("text", "The answer is 42" );
  return NodeStatus::SUCCESS;
}

