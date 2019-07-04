# pedlab
A custom pedestrian simulator.

The node provides a simulation of a robot and a set of people using a Social Force Model.

Robot simulation
* The robot offers an interface 
* 360º 2D laser scans are simulated for the robot
* Detections of the persons presented are also simulated


The persons are affected by the robot and other persons using the social force model

# Simulation configuration files



# ROS API

## Subscribed topics

## Published topics

The following topics are published. The names can be configured (see below)

* /scan360: <sensor_msgs::LaserScan> simulated 360º robot laser
* /odom: Robot odometry
* /people_detections <people_msgs::People>


## ROS params




# ROS Dependencies
 
Animated Markers for the markers for persons in RViz

https://github.com/srl-freiburg/animated_markers

Detections are provided as People messages:

http://wiki.ros.org/people_msgs

# External Dependencies


LightSFM library:

https://github.com/robotics-upo/lightsfm





