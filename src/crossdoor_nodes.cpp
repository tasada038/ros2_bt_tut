#include "ros2_bt_tut/crossdoor_nodes.hpp"

inline void SleepMS(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

using BT::NodeStatus;

NodeStatus CrossDoor::isDoorClosed()
{
  SleepMS(500);
  if (_door_open)
  {
    std::cout << "isDoorClosed: SUCCESS" << std::endl;
  }
  else{
    std::cout << "isDoorClosed: FAILURE" << std::endl;
  }
  return !_door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::passThroughDoor()
{
  SleepMS(500);
  if (_door_open)
  {
    std::cout << "passThroughDoor: SUCCESS" << std::endl;
  }
  else{
    std::cout << "passThroughDoor: FAILURE" << std::endl;
  }
  return _door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::openDoor()
{
  SleepMS(500);
  if (_door_locked)
  {
    std::cout << "openDoor: FAILURE" << std::endl;
    return NodeStatus::FAILURE;
  }
  else
  {
    _door_open = true;
    std::cout << "openDoor: SUCCESS" << std::endl;
    return NodeStatus::SUCCESS;
  }
}

NodeStatus CrossDoor::pickLock()
{
  SleepMS(500);
  _pick_attempts++;
  // succeed at 3rd attempt
  if (_door_locked && _pick_attempts < 3)
  {
    _door_locked = false;
    _door_open = true;
    std::cout << "pick a Lock: FAILURE" << std::endl;
    return NodeStatus::FAILURE;
  }
  std::cout << "pick a Lock: SUCCESS" << std::endl;
  return NodeStatus::SUCCESS;
}

NodeStatus CrossDoor::smashDoor()
{
  _door_locked = false;
  _door_open = true;
  // smash always works
  std::cout << "smashDoor: SUCCESS" << std::endl;
  return NodeStatus::SUCCESS;
}

void CrossDoor::registerNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerSimpleCondition(
      "IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this));
  
  factory.registerSimpleAction(
      "PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this));

  factory.registerSimpleAction(
      "OpenDoor", std::bind(&CrossDoor::openDoor, this));

  factory.registerSimpleAction(
      "PickLock", std::bind(&CrossDoor::pickLock, this));

  factory.registerSimpleCondition(
      "SmashDoor", std::bind(&CrossDoor::smashDoor, this));
}