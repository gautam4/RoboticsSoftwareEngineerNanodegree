# Project Map My World

## Introduction 
* This project uses the ROS package RTAB-Map to construct a map of the simulated Gazebo world.

## Layout
* catkin_ws/src/my_robot
  * config
    * Configuration files containing the parameters for the move_base node
    * These files were provided by Udacity
  * launch
    * amcl.launch: 
    * localization.launch
    * mapping.launch
    * robot_description.launch
    * world.launch
  * maps
    * map.yaml
    * map.pgm
  * meshes
    * hokuyo.dae
  * model
    * Building/
    * BuildingThree/
    * Model/
  * urdf
    * my_robot.gazebo
    * my_robot.xacro
  * world
    * Project1GazeboWorld.world
    * WorldThree.world
    * WorldTwo.world
* 3D Map.PNG: Image of 3D map generated using RTAB-Map
* RTabMapViewer.PNG: Image of RTAB-Map file, indicating mapping with multiple loop closures.
* RTABMAP DB_GoogleDrive Link.txt: Due to file size, the RTAB Map file is stored on Google drive and can be accessed thru the link contained in this document.
    
  
