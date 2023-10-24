## Description
This ROS1 (Noetic) node provide a implementation of a reactive robot wall follower

## Dependencies

- [ros_noetic](http://wiki.ros.org/noetic/)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [std_msgs](http://wiki.ros.org/std_msgs)
- [sensor_msgs](http://wiki.ros.org/sensor_mgs)
- [PCL](http://wiki.ros.org/pcl_ros)
- [Eigen3](http://wiki.ros.org/eigen)


## Usage

### Compilation

Create workspace if you don´t have one:
````
mkdir catikin_ws
cd catkin_ws
mkdir src
````

Paste turtlebot3_control in src folder and compile:
````
cd catkin_ws
catkin_make
````

### Files structure
````
├── CMakeLists.txt
├── cfg
│   └── RobotMission.cfg
├── include
│   └── turtlebot3_control
├── launch
│   └── bringup_robot.launch
├── package.xml
├── rviz
│   └── turtlebot3.rviz
├── scripts
│   ├── respawn.py
│   └── robot_graph_results.py
├── src
│   ├── data.txt
│   ├── follow_wall.cpp
│   └── line_fitting.cpp
└── utils
    ├── convert.py
    ├── data.txt
    ├── data2.txt
    ├── data2_inside.txt
    ├── data_inside.txt
    ├── robot_graph_results.txt
    ├── robot_graph_results_2.txt
    ├── robot_graph_results_outside.txt
    └── robot_graph_results_outside_2.txt
````

### Launch

````
roslaunch turtlebot3_control bringup_robot.launch
````

### Dynamic Reconfigure Parameters

To adjust the robot parameters in real-time run:

````
rosrun rqt_reconfigure rqt_reconfigure

````

### Graphs visualization
To visualize the robot pose, heading, velocities and accelerations during the navigation run:
````
rosrun turtlebot3_control robot_graph_results.py

````
