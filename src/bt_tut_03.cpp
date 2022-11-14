#include "rclcpp/rclcpp.hpp"
#include "ros2_bt_tut/bt_tut_03.hpp"

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CalculateGoal goal="{GoalPosition}" />
            <PrintTarget   target="{GoalPosition}" />
            <SetBlackboard output_key="OtherGoal" value="-1;3;0" />
            <PrintTarget   target="{OtherGoal}" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  auto tree = factory.createTreeFromText(xml_text);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING){
    status = tree.tickRoot();
  }
  
  return 0;
}


// SetBlackboard is used instead of Script because bt_v3.8.x causes the following error.
// ```shell
// terminate called after throwing an instance of 'BT::RuntimeError' what(): Error at line 6: -> Node not recognized: SetBlackboard Aborted (core dumped)
// ```
// https://github.com/BehaviorTree/BehaviorTree.CPP/commit/0cef2e22408a96d80389238cc7bc8514e6da28a7