#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros2_bt_tut/crossdoor_nodes.hpp"

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Fallback>
                <Inverter>
                    <IsDoorClosed/>
                </Inverter>
                <SubTree ID="DoorClosed"/>
            </Fallback>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="DoorClosed">
        <Fallback>
            <OpenDoor/>
            <RetryUntilSuccessful num_attempts="5">
                <PickLock/>
            </RetryUntilSuccessful>
            <SmashDoor/>
        </Fallback>
    </BehaviorTree>
</root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  CrossDoor cross_door;
  cross_door.registerNodes(factory);

  // the XML is the one shown at the beginning of the tutorial
  auto tree = factory.createTreeFromText(xml_text);

  // helper function to print the tree
  BT::printTreeRecursively(tree.rootNode());

  // tree.tickRootWhileRunning();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING){
    status = tree.tickRoot();
  }

  return 0;
}
