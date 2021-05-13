//includes
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
// Initialize the simple_navigation_goals node

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_objects");
 //tell the action client that we want to spin a thread by default
  
  MoveBaseClient ac("move_base", true);
  
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
//target position 
  goal.target_pose.pose.position.x = -4.0;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.w = 1.0;
  //sendeng robot to the target position
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reached the pickup zone!");
  else
    ROS_INFO("The base failed to move.");
    
  ros::Duration(5.0).sleep();
//dropoff pose
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = 5.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reached the dropoff zone!");
  else
    ROS_INFO("The base failed to move.");
    
  ros::Duration(5.0).sleep();
  
  return 0;
}