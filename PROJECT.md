# Labyrinth Collaborative Discovery

## Research
this http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo page has a video utilizing a way to merge maps created by different robots.


## Launching Procedure for Solo Exploration run of robot

The robot is placed within the labyrinth solo at first.
The robot then has some time to discover the labyrinth on its own,
building a map and try to find all tags.

### Roscore

D$ roscore

### Turtlebot

D$ tbssh
(turtlebot)
R$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

### Monitoring

D$ rqt

### Movement Server

- move_base

D$ roslaunch /home/ros/catkin_ws/src/turtlebot3/turtlebot3_navigation/launch/move_base.launch

## SLAM

- gmapping

D$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

## Autonomous Exploration

- explore_lite

D$ roslaunch /opt/ros/kinetic/share/explore_lite/launch/explore.launch


/explore/costmap_topic: map
/explore/costmap_updates_topic: map_updates
/explore/gain_scale: 1.0
/explore/min_frontier_size: 0.75
/explore/orientation_scale: 0.0
/explore/planner_frequency: 0.33
/explore/potential_scale: 3.0
/explore/progress_timeout: 30.0
/explore/robot_base_frame: base_link
/explore/transform_tolerance: 0.3
/explore/visualize: True
/rosdistro: kinetic
/rosversion: 1.12.14

(does not quite work yet)

## Save currently built map

D$ rosrun map_server map_saver -f ~/map_filename

## TODO: other chapters








### Launch with namespaces for multi robot use (taken from tutorial)
- roslaunch turtlebot3_gazebo_ros multi_turtlebot3.launch

- roslaunch turtlebot3_gazebo_ros multi_turtlebot3_slam.launch ns:=tb3_0
- roslaunch turtlebot3_gazebo_ros multi_turtlebot3_slam.launch ns:=tb3_1
- roslaunch turtlebot3_gazebo_ros multi_turtlebot3_slam.launch ns:=tb3_2

- roslaunch turtlebot3_gazebo_ros multi_map_merge.launch

- rosrun rviz rviz -d `rospack find turtlebot3_gazebo_ros` /rviz/multi_turtlebot3_slam.rviz

- roslaunch turtlebot3_gazebo_ros _control_multi_turtlebot3.launch

### Interesting Packages
- Gazebo (Simulation without actual robot)
- ros-kinetic-multirobot-map-merge
  (can be used to merge maps of all participating robots into one) http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-slam-by-multiple-turtlebot3s
- gmapping (slam algorithm)
