# A Multi-Strategy Wheeled Robot System for Navigation

Prepared by
 1. Chala Bekabil ([cbg11](https://github.com/cbg11))
 2. Robbel Habtamu ([R-hab](https://github.com/R-hab))

## Launch the robot

After sourcing
```
roslaunch mybot gazebo.launch
```

## Commands used 

To run motor controller node, open new terminal
```
source devel/setup.bash
rosrun mybot motor_controller
```

To run controller node (To move the robot)
```
rosrun mybot controller.py
```

To see the ROS node tree
```
rqt_graph
```
