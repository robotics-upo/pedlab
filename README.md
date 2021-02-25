# pedlab
A custom pedestrian simulator.

The node provides a simulation of a robot and a set of people using a Social Force Model.

It is based on ROS and uses RViz for visualization.

**[DOCUMENTATION UNDER CONSTRUCTION]**

## Robot simulation
* The robot offers an interface to be controlled
* 360º 2D laser scans are simulated for the robot
* Detections of the persons presented are also simulated

## Person simulation

The persons are affected by the robot and other persons using the [Social Force Model](https://github.com/robotics-upo/lightsfm/tree/refactored)

# Simulation configuration files

The definition of the number of simulated pedestrians in the scenario and the goal positions that they must reach are provided through a simple XML file.

An example file is shown next:

```html
<?xml version="1.0" encoding="UTF-8"?>
<scenario>

  <!-- definition of goals and itermediate points-->
  <goal id ="A" x="7"  y="32" r="1"/> 	 
  <goal id ="B" x="27" y="7"  r="1"/>
  <waypoint id="D" x="27" y ="32" r="1"/>

  <!-- paths between goals -->
  <edge from="A" to="D"/>
  <edge from="D" to="B"/>

  <!-- pedestrian with initial position and goals to reach -->
  <agent x="6" y="31" n="2" dx="0" dy="0" type="1">
    <addwaypoint id="A"/>
    <addwaypoint id="B"/>
  </agent>

</scenario>
```

The configuration must be enclosed between the tags ```<scenario></scenario>```.

First, the different goals that the pedestrians must be reach can be indicated through the tag ```<goal/>``` with the arguments:
* *id*, identifier of the goal.
* position in the map, *x* coordinate (m) and *y* coordinate (m).
* Admisible distance from the goal point to consider that the goal has been reached *r* (m).

With the tag ```<waypoint/>``` is possible to add intermediate goals that can be used as points that the agents must visit to in the way to reach their goals.

Secondly, the tag ```<edge/>``` is used to connect the different goals and waypoints similarly to a graph. This way, the agents must follow the defined edges to reach the different goals.

Finally, each agent in the scenario is indicated by the tag ```<agent></agent>```, with the following arguments:
* Initial position in the map indicated by the arguments *x* and *y*.
* Arguments *dx* and *dy* employed to feed an uniform distribution to randomly vary the position of the agent.
* Type of agent, that can be *1*: regular pedestrian, or *2*: target pedestrian (a pedestrian that forms a walking group with the robot).
Moreover, the goals that the agent must try to reach are indicated by the *id* argument in the tag ```<addwaypoint/>```


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
* *config_file*: route and name of the scenario configuration file (see section "Simulation configuration files").
* *freq*: Execution frequency of the simulation and the publication of topics (Default: 15 Hz).

### Laser scan configuration
* *scan360\_id*: topic name for simulated laser (default: "/scan360").
* *scan360_readings*: number of beams of the simulated laser scan (default: 1440).
* *scan\_range\_max*: maximum range distance of the scan (default: 10 m).

### Robot configuration

The simulated robot is a differential robot with circular footprint. 

* *cmd\_vel_id*: topic name to control the robot (default:"/cmd_vel").
* *robot\_radius*: radius of the circumference that circumscribes the robot footprint (default: 0.3 m).
* *robot\_max\_velocity*: robot maximum velocity allowed (default: 0.6 m/s).
* *pose\_initial\_x*: initial robot position in the X-axis of the map (m).
* *pose\_initial\_y*: initial robot position in the Y-axis of the map(m).
* *pose\_initial\_yaw*: initial robot heading in the map (rad).
* *odom_id*: topic name of the robot odometry published by the simulator (default: "/odom").

### Simulated people configuration
* *teleoperated\_target*: boolean to give the option of teleoperating one of the human agents (default: false).
* *cmd\_vel\_target\_id*: topic name to control the human agent (default: "/target/cmd_vel").
* *person\_radius*: radius of the circumference that circumscribes the human footprint (default: 0.35 m).
* *people\_average\_vel*: average velocity of the simulated pedestrians (default: 0.9 m/s).
* *people\_sd\_vel*: standard deviation of the normal distribution that represents the pedestrian velocities (default: 0.001 m/s).
* *target\_cyclic\_goals*: it indicates whether the pedestrian used as target (for instance to be accompanied by the robot) performs a cyclic movements among its destination goals (default: false).
* *target\_force\_factor\_desired*: factor of the desired force to reach the goal for the simulated pedestrian marked as target (see the [Social Force Model](https://github.com/robotics-upo/lightsfm/tree/refactored)) (default: 4.0).


### Uncertainty configuration
* *people\_detection\_range*: maximum detection distance of a pedestrian (default: 8.0 m).
* *noisy\_detections*: if True, some noise is added to the position and orientation of the pedestrians detected (default: false).
* *sd\_noise*: standard deviation of the normal distribution that is employed to add noise to the position of the people detected (default: 0.01 m).
* *sd\_noise\_theta*: standard deviation of the normal distribution that is employed to add noise to the orientation of people detected (default: 1 rad).
* *false\_negative\_prob*: if *noisy\_detections* is True, this probability is employed to randomly remove people detections (default: 0.1).
* *perfect\_tracking*: if True, no noise and false negative detections are added to the people detected (default: false).


# ROS Dependencies
 
* Animated Markers for the markers for persons in RViz: https://github.com/srl-freiburg/animated_markers

* Detections are provided as People messages: http://wiki.ros.org/people_msgs

* Social Force Model library, lightsfm https://github.com/robotics-upo/lightsfm/tree/refactored (branch: refactored)


# Example launch file

```sh
roslaunch pedlab corridors.launch
```
This will launch a corridor scenario where a group of two pedestrians and a target pedestrian (accompanying the robot) are launched.
The robot is represented by the TF "base_link", and it is a regular differential robot that can be controlled through the topic "/cmd_vel".


# Edition node

A secondary ROS node has been also implemented as a simple tool for creating scenario configuration files.
It makes use of [RViz](http://wiki.ros.org/rviz) and [map\_server](http://wiki.ros.org/map_server).
With the map\_server an existing map can be loaded and published in ROS. Or even a blank map with the desired dimensions can be employed.
Then, the map can be visualized in RViz. The map is used as a reference to build a new simulation scenario.
To do it, the "publish point" tool of RViz is employed. By clicking on the desired map location in RViz, a new node (pedestrian goal) will be created. Nodes that are closer than an indicated distance threshold will be automatically linked and an edge between them will be created. 

Once the edition node is finished (CTRL+c shortkey), the configuration file is saved. 
Finally, the desired agents must be added manually to the configuration file created. 


## ROS API

### Subscribed topics

* */clicked\_point*: <geometry_msgs::PointStamped>. Point coordinates published by RViz "publish_point" tool.

### Published topics

* */plab/markers/edited\_nodes*: <visualization_msgs::MarkerArray> spheres representing the pedestrian goals in the scenario.
* */plab/markers/edited\_edges*: <visualization_msgs::Marker> list of lines (edges) that join the goal nodes and represent the paths to reach the goals.

### ROS params

* *file*: route and name of the configuration file that is going to be created (default: graph.xml)
* *freq*: frequency of topic publication (default: 15 Hz)
* *distance*: maximum distance between nodes to be connected by an edge (default: 3.0 m)

### Example launch file

```sh
roslaunch pedlab editor_example.launch
```







