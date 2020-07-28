# Project Go Chase It!

## Introduction
* This project involves defining a custom robot with additional sensors in a simulated Gazebo world and programming two nodes that enable the robot to chase a white ball based on camera input.

## Layout
* The files are organized around the two major catkin packages ball_chaser and my_robot
  * ball_chase/
    * src/drive_bot.cpp: Defines a server node that defines service to drive robot using linear x and angular z velocities.
    * src/process_image.cpp: Defines a client node that subscribes to the camera sensor and analyzes for the presence of a white ball. By requesting a drive_bot service, this node guides the robot to travel to the white ball's position.
  * my_robot/ 
    * world/ contains Gazebo world file, Project1GazeboWorld.world
    * urdf/ contains the robot urdf definition
    * launch/ contains the launch files for the simulation
    * meshes/ contains the hokuyo.dae mesh for the lidar sensor
    
 
  
 
