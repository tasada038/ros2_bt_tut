#ifndef ROS2_BT_TUT__CROSSDOOR_NODES_HPP_
#define ROS2_BT_TUT__CROSSDOOR_NODES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

class CrossDoor
{
    public:
        void registerNodes(BT::BehaviorTreeFactory& factory);

        // SUCCESS if _door_open == true
        BT::NodeStatus isDoorClosed();

        // SUCCESS if _door_open == true
        BT::NodeStatus passThroughDoor();

        // After 3 attempts, will open a locked door
        BT::NodeStatus pickLock();

        // FAILURE if door locked
        BT::NodeStatus openDoor();

        // WILL always open a door
        BT::NodeStatus smashDoor();

    private:
        bool _door_open   = false;
        bool _door_locked = true;
        int _pick_attempts = 0;
};


#endif //ROS2_BT_TUT__CROSSDOOR_NODES_HPP_