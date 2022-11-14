#include "ros2_bt_tut/movebase_node.hpp"

BT::NodeStatus MoveBaseAction::tick()
{
    Pose2D goal;
    if ( !getInput<Pose2D>("goal", goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }

    printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f yaw=%.2f\n", goal.x, goal.y, goal.yaw);

    _halt_requested.store(false);
    int count = 0;

    // Pretend that "computing" takes 250 milliseconds.
    // It is up to you to check periodically _halt_requested and interrupt
    // this tick() if it is true.
    while (!_halt_requested && count++ < 25)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    
    return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

void MoveBaseAction::halt()
{
    _halt_requested.store(true);
}