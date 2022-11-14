#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/movebase_node.hpp"
#include "ros2_bt_tut/bt_tut_02.hpp"

// clang-format off

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="main_sequence">
            <SetBlackboard output_key="move_goal" value="1;2;3" />
            <SubTree ID="MoveRobot" target="move_goal" output="move_result" />
            <SaySomething message="{move_result}"/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="MoveRobot">
        <Fallback name="move_robot_main">
            <SequenceStar>
                <MoveBase       goal="{target}"/>
                <SetBlackboard output_key="output" value="mission accomplished" />
            </SequenceStar>
            <ForceFailure>
                <SetBlackboard output_key="output" value="mission failed" />
            </ForceFailure>
        </Fallback>
    </BehaviorTree>
</root>
 )";

// clang-format on

using namespace BT;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);    
  BehaviorTreeFactory factory;

  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<MoveBaseAction>("MoveBase");

  auto tree = factory.createTreeFromText(xml_text);

  NodeStatus status = NodeStatus::RUNNING;//tree.tickRootWhileRunning();
  while(rclcpp::ok() && status == NodeStatus::RUNNING){
    status = tree.tickRoot();
  }

  // let's visualize some information about the current state of the blackboards.
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[0]->debugMessage();
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[1]->debugMessage();
  std::cout << "--------------" << std::endl;

  return 0;
}