# pedlab
A custom pedestrian simulator.

The node provides a simulation of a robot and a set of people using a Social Force Model.

**[DOCUMENTATION UNDER CONSTRUCTION]**

## Robot simulation
* The robot offers an interface to be controlled
* 360º 2D laser scans are simulated for the robot
* Detections of the persons presented are also simulated

## Person simulation

The persons are affected by the robot and other persons using the social force model

# Simulation configuration files



# ROS API

## Subscribed topics

The node subscribes to the following topics. The names can be configured (see ROS params section)

* */cmd\_vel*: <geometry_msgs::Twist>. Control command of the robot.
* */target/cmd_vel*: <geometry_msgs::Twist>. It is used to command another agent (pedestrian) which is not the the robot. To be used, the parameter "*teleoperated\_target*" must be True.

## Published topics

The following topics are published. The names can be configured (see below)

* */scan360*: <sensor_msgs::LaserScan> simulated 360º robot laser.
* */odom*: <nav_msgs::Oodometry> Robot odometry.
* */people\_detections*: <people_msgs::People> publication of the people detected.
* */plab/markers/detected_people*: <visualization_msgs::MarkerArray> RViz marker of the people detected by the agent.
* */plab/markers/people*: <animated_marker_msgs::AnimatedMarkerArray> RViz animated marker of the people in the scene.


## ROS params

### Scenario configuration
* *config_file*: route and name of the scenario configuration file.
* *freq*: Execution frequency of the simulation and the publication of topics (Default: 15 Hz).

### Laser scan configuration
* *scan360\_id*: topic name for simulated laser (default: "/scan360").
* *scan360_readings*: number of beams of the simulated laser scan (default: 1440).
* *scan\_range\_max*: maximum range distance of the scan (default: 10 m).

### Robot configuration
* *cmd\_vel_id*: topic name to control the robot
* *robot\_radius*:
* *robot\_max\_velocity*:
* *pose\_initial\_x*:
* *pose\_initial\_y*:
* *pose\_initial\_yaw*:
* *odom_id*: 

### Simulated people configuration
* *teleoperated\_target*:
* *cmd\_vel\_target\_id*:
* *person\_radius*:
* *people\_average\_vel*:
* *people\_sd\_vel*:
* *target\_cyclic\_goals*:
* *target\_force\_factor\_desired*:


### Uncertainty configuration
* *people\_detection\_range*:
* *noisy\_detections*:
* *sd\_noise*:
* *sd\_noise\_theta*:
* *false\_negative\_prob*:
* *perfect\_tracking*:





# Edition node

A secondary ROS node has been also implemented as a tool for generation of scenarios ...

## ROS API

### Subscribed topics
### Published topics
### ROS params


# ROS Dependencies
 
* Animated Markers for the markers for persons in RViz: https://github.com/srl-freiburg/animated_markers

* Detections are provided as People messages: http://wiki.ros.org/people_msgs

* Social Force Model library, lightsfm https://github.com/robotics-upo/lightsfm/tree/refactored (branch: refactored)


# Example launch file

```sh
roslaunch pedlab corridors.launch
```




