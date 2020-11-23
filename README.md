# ARIAC Competition

Picking and delivering parts in an industrial environment with faulty parts/faulty gripper/sensor blackout/moving obstacles

## Results
[Scenario 5](https://youtu.be/OBoudhBBgK8)

To view previous scenarios and results, please click the link below

[Prev Results 1](https://github.com/Pradeep-Gopal/ARIAC---Agile-Software-Development-for-Robots)
[Prev Results 2](https://github.com/Pradeep-Gopal/ARIAC_Agile_SoftDev_for_Industrial_Robots)

## Team members
1. Pradeep Gopal
2. Rajesh 
3. Govind
4. Dakota Abernathy
5. Cheng Chen

## Steps to Run the package

Install Ariac package in your workspace using the steps mentioned in the following link

[Ariac Installation Instructions](https://github.com/usnistgov/ARIAC/blob/master/wiki/tutorials/installation.md)


Follow these instructions to run the package after installing ARIAC

1. Copy the package and paste it in /ariac_ws/src
2. Open a terminal and type the following commands
3. cd /ariac_ws
4. catkin build rwa5_group_1
5. source devel/setup.bash
6. roslaunch rwa5_group_1 rwa5.launch load_moveit:=true

Wait till the terminal says "you can start planning now"

7. Open a new terminal and enter the following command to run the node.
8. cd /ariac_ws
9. source devel/setup.bash
10. rosrun rwa5_group_1 rwa5_node


