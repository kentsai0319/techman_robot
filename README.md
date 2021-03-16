# Deprecated !

For driver using TMFlow, see https://github.com/TechmanRobotInc  
see also: [tmr_ros1_dev](https://github.com/kentsai0319/tmr_ros1_dev) and [tmrlib_dev](https://github.com/kentsai0319/tmrlib_dev)  

## techman_robot

A driver provides ROS support for techman robots. TM5_700 is available for ROS Indigo.  

## Overview

* Action interface on */joint\_trajectory\_action* for integration with __MoveIt__
* Publishes robot joint state on */joint\_states*
* Publishes TCP position on */tool\_position*
* Publishes TCP velocity on */tool\_velocity*
* Publishes TCP force on */wrench*
* Service call to set outputs on */tm\_driver/set\_io*


## Installation
First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like  /path/to/your/catkin_workspace/src/techman_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.  
Note that this package depends on hardware_interface, and controller_manager.  


## Usage with Moveit

### test in simulation:

To bring up moveit environment in simulation mode, run:  
```roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch```

### run with real robot:

set up networking:

1. Click on the network settings (double-arrow in the title bar) and select *Edit Connections*
2. Locate the new connection (mine was *Wired Connection 1*) and select *Edit*. Under the IPv4 tab, set:
    * address = 192.168.0.11 (or similar)
    * netmask = 255.255.255.0
3. Connect an ethernet cable and try to ping your connected robot:
    * ```ping 192.168.0.10```

To bring up moveit environment and connect to real robot, run:  
```roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch sim:=False robot_ip:=192.168.0.10```


## Usage with Gazebo
To bring up the simulated robot in Gazebo, run:  
```roslaunch tm_gazebo tm700.launch```
