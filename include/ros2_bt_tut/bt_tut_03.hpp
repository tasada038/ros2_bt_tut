#ifndef ROS2_BT_TUT__BT_TUT_03_HPP_
#define ROS2_BT_TUT__BT_TUT_03_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

struct Position2D 
{ 
  double x, y;
  double yaw;
};

// Template specialization to converts a string to Position2D.
namespace BT
{
template <> inline
Position2D convertFromString(StringView str)
{
    // three real numbers separated by commas
    auto parts = BT::splitString(str, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input");
    }
    else
    {
        Position2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
		    output.yaw = convertFromString<double>(parts[2]);
		return output;
    }
}
} // end namespace BT

class CalculateGoal: public BT::SyncActionNode
{
  public:
    CalculateGoal(const std::string& name, const BT::NodeConfiguration& config):
      BT::SyncActionNode(name,config)
    {}

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<Position2D>("goal") };
    }

    virtual BT::NodeStatus tick() override
    {
      Position2D mygoal = {1.1, 2.3, 0.0};
      // printf("Calculate Goal: [ %.1f, %.1f, %.1f]\n", mygoal.x, mygoal.y, mygoal.yaw);
      setOutput<Position2D>("goal", mygoal);
      return BT::NodeStatus::SUCCESS;
    }
};

class PrintTarget: public BT::SyncActionNode
{
  public:
    PrintTarget(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name,config)
    {}

    static BT::PortsList providedPorts()
    {
      // Optionally, a port can have a human readable description
      const char*  description = "Simply print the goal on console...";
      return { BT::InputPort<Position2D>("target", description) };
    }
      
    virtual BT::NodeStatus tick() override
    {
      auto res = getInput<Position2D>("target");
      if( !res )
      {
        throw BT::RuntimeError("error reading port [target]:", res.error());
      }
      Position2D target = res.value();
      printf("Target positions: [ %.1f, %.1f, %.1f]\n", target.x, target.y, target.yaw);
      return BT::NodeStatus::SUCCESS;
    }
};

#endif //ROS2_BT_TUT__BT_TUT_03_HPP_