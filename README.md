# ROS2 Behavior Tree Tutorial 
This is the package that does the [Behavior Tree tutorial](https://www.behaviortree.dev/docs/3.8/category/tutorial---basics) on ROS2 and the ROS2 workspace

![BehaviorTree_Sample](/img/Behavior_Tree_Sample.png)

## Requirements
- Linux OS
    - Ubuntu 22.04 Laptop PC
- ROS
    - Humble

## Install & Build
The following commands download a package from a remote repository and install it in your colcon workspace.

```shell
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/tasada038/ros2_bt_tut.git
cd ~/dev_ws && colcon build
```

## Usage
Watch the tutorial and follow along!

1. Your first Behavior Tree
```shell
ros2 run ros2_bt_tut bt_tut_01

--------------------
Battery OK
Gripper open
ApproachObject: approach_object
Gripper close
```

2. Blackboard and ports
```shell
ros2 run ros2_bt_tut bt_tut_02

--------------------
Robot says: hello
[INFO] [1668345931.949463504] [input]: input data: hello
Robot says: The answer is 42
[INFO] [1668345931.949576916] [input]: input data: The answer is 42
```

3. Ports with generic types
```shell
ros2 run ros2_bt_tut bt_tut_03

--------------------
Target positions: [ 1.1, 2.3, 0.0]
Target positions: [ -1.0, 3.0, 0.0]
```

5. Using Subtrees
```shell
ros2 run ros2_bt_tut bt_tut_05

----------------
Sequence
   Fallback
      Inverter
         IsDoorClosed
      DoorClosed
         Fallback
            OpenDoor
            RetryUntilSuccessful
               PickLock
            SmashDoor
   PassThroughDoor
----------------
isDoorClosed: FAILURE
openDoor: FAILURE
pick a Lock: FAILURE
pick a Lock: SUCCESS
passThroughDoor: SUCCESS
```

6. Port Remapping
```shell
ros2 run ros2_bt_tut bt_tut_06

----------------
[ MoveBase: STARTED ]. goal: x=1 y=2.0 yaw=3.00
[ MoveBase: FINISHED ]
Robot says: mission accomplished
[INFO] [1668346370.479600523] [input]: input data: mission accomplished
--------------
move_result (std::string) -> full
move_goal (Pose2D) -> full
--------------
--------------
```

7. Use multiple XML files
```shell
ros2 run ros2_bt_tut bt_tut_07

Robot says: starting MainTree
[INFO] [1668346387.117569955] [input]: input data: starting MainTree
Robot says: Executing SubA
[INFO] [1668346387.117667646] [input]: input data: Executing SubA
Robot says: Executing SubB
[INFO] [1668346387.117699933] [input]: input data: Executing SubB
Registered BehaviorTrees:
 - MainTree
 - SubA
 - SubB
```

8. Pass additional arguments
```shell
ros2 run ros2_bt_tut bt_tut_08

Action_A: 42 / 3.14 / hello world
[INFO] [1668346442.312539579] [action_a]: arg1, arg2, arg3: [42, 3.140000, hello world]
Action_B: 69 / 9.99 / interesting_value
```

## License
This repository is licensed under the MIT license, see LICENSE.