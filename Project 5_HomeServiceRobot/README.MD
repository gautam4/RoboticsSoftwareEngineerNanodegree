# Project Home Service Robot

## Introduction
* This project involves defining a Gazebo World, mapping the world, localizing a Turtlebot II robot in the world, and programming the robot to navigate to a pickup and delivery location to simulate the behavior of a home service delivery robot. 
* The object that the robot picks up is simulated by a red marker displayed in RViz that disappears when the robot reaches the pickup location and reappears once the robot reaches the delivery location
* For more information on the project and the specific packages utilized, see the included report file, Report_HomeServiceRobot.pdf.
## Layout
catkin_ws/src
* slam_gmapping: gmapping package
* turtlebot: contains turtlebot_teleop package
* turtlebot_interactions: contains turtlebot_rviz_launchers package and modified rviz config file, marker.rviz, to display markers when rviz launches
* turtlebot_simulator: contains modified turtlebot world launch file to launch turtlebot in the custom gazebo world
* map: contains the SLAM generated map.pgm and map.yaml
* scripts:
  * test_slam.sh : runs SLAM manually using teleop package
  * test_navigation.sh : tests robot’s AMCL navigation ability given a navigation goal
  * pick_objects.sh : tests pick_objects.cpp in simulated world
  * add_marker.sh : tests display_markers.cpp in world
  * home_service.sh : runs pick_objects and add_markers nodes in the simulated world
* rvizConfig
* pick_objects: contains pick_objects.cpp
* add_markers: contains display_markers.cpp and add_markers.cpp


