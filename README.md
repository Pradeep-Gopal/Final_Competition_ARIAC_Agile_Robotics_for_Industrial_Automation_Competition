# ARIAC Competition

Picking and delivering parts in an industrial environment with faulty parts/faulty gripper/sensor blackout/moving obstacles

##Results
[Final Competition Video](https://youtu.be/3olX7gtbj1k)

## Previous Trial Results
[Scenario 5](https://youtu.be/OBoudhBBgK8)

[Scenario 6](https://www.youtube.com/watch?v=RjY7Hs_85mE)

To view previous scenarios and results, please click the link below

[Prev Results 1](https://github.com/Pradeep-Gopal/ARIAC---Agile-Software-Development-for-Robots)

[Prev Results 2](https://github.com/Pradeep-Gopal/ARIAC_Agile_SoftDev_for_Industrial_Robots)

## Team members
1. Pradeep Gopal
2. Rajesh NS
3. Govind Ajith Kumar
4. Dakota Abernathy
5. Cheng Chen

## Steps to Run the package

Install Ariac package in your workspace using the steps mentioned in the following link

[Ariac Installation Instructions](https://github.com/usnistgov/ARIAC/blob/master/wiki/tutorials/installation.md)

Follow these instructions to run the package after installing ARIAC

1. Copy the package and paste it in /ariac_ws/src
2. Open a terminal and type the following commands
```
cd /ariac_ws
catkin build final-group1 
source devel/setup.bash
roslaunch final-group1 final.launch load_moveit:=true
```

Wait till the terminal says "you can start planning now"

7. Open a new terminal and enter the following command to run the node.

```
cd /ariac_ws
source devel/setup.bash
rosrun final-group1 final_node
```

## Steps to change the order given to the robot
```
cd /ariac_ws/src/final-group1/launch
gedit final.launch
```

Change the final.yaml to any other yaml files present in the config folder of the package
 ```
-f $(find final-group1)/config/final.yaml
```


## Steps to view the Doxygen report

Open the index.html file present in "/ariac_ws/src/final-group1/Doxygen_report/html/index.html"



