#include "ros2_bt_tut/bt_tut_01.hpp"

// ApproachObject Class of SyncActionNode
ApproachObject::ApproachObject(const std::string& name) :
    BT::SyncActionNode(name, {})
{}

NodeStatus ApproachObject::tick()
{
  std::cout << "ApproachObject: " << this->name() << std::endl;
  return NodeStatus::SUCCESS;
}

// CheckBattery
NodeStatus CheckBattery()
{
  std::cout << "Battery OK" << std::endl;
  return NodeStatus::SUCCESS;
}

// GripperInterface
GripperInterface::GripperInterface() : _open(true) {}

NodeStatus GripperInterface::open()
{
  _open = true;
  std::cout << "Gripper open" << std::endl;
  return NodeStatus::SUCCESS;
}

NodeStatus GripperInterface::close()
{
  _open = false;
  std::cout << "Gripper close" << std::endl;
  return NodeStatus::SUCCESS;
}