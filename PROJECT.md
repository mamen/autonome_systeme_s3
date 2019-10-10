# Labyrinth Collaborative Discovery

## Research
this http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo page has a video utilizing a way to merge maps created by different robots.


### Launch 
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
