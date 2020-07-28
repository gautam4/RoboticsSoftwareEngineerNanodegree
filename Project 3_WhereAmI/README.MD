# Project Where Am I

## Introduction 
* This project uses the AMCL(Adaptive Monte Carlo Localization) algorithm to localize the robot in a simulated Gazebo world

## Layout
* my_robot: 
  * config: 
    * Configuration files containing the parameters for the move_base node
    * These files were provided by Udacity
  * launch
    * robot_description.launch
    * world.launch: launches world and robot in Gazebo and RViz
    * amcl.launch: Launches world, robot, and amcl nodes in Gazebo and RViz
  * maps
    * map.yaml: defines map.pgm metadata
    * map.pgm: 
  * meshes
    * hokuyo.dae: mesh for hokuyo lidar sensor
  * model
    * Building/: Defines Gazebo building 
    * Model/: Defines a stationary robot model
  * urdf
    * my_robot.gazebo
    * my_robot.xacro: Defines skid steer robot with a camera and lidar sensor
  * world
    * WorldTwo.world: Gazebo world containing building and robot
