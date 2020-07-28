#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  // Initialize pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Define Pickup and Delivery locations
  move_base_msgs::MoveBaseGoal startGoal;
  move_base_msgs::MoveBaseGoal endGoal;

  // set up the frame parameters for pickup locations
  startGoal.target_pose.header.frame_id = "map";
  startGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  startGoal.target_pose.pose.position.x = -12.0;
  startGoal.target_pose.pose.position.y = 3.0;
  startGoal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup location");
  ac.sendGoal(startGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached pickup destination");
  else
    ROS_INFO("The base failed to move to pickup destination");
 
  ac.waitForServer(ros::Duration(5.0));

  //Send End Goal
  //Set up frame paramters for end location 
  endGoal.target_pose.header.frame_id = "map";
  endGoal.target_pose.header.stamp = ros::Time::now();
  
  // Define end goal
  endGoal.target_pose.pose.position.x = 3.0;
  endGoal.target_pose.pose.position.y = -6.0;
  endGoal.target_pose.pose.orientation.w = 1.0;

  // Send the end goal position and orientation
  ROS_INFO("Sending end location");
  ac.sendGoal(endGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached end destination");
  else
    ROS_INFO("The base failed to move to end destination");

  
  return 0;
}
