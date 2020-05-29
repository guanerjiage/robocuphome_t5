// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;


// Create a ROS action client to move TIAGo's arm
void createHeadClient(head_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new head_control_client("/head_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the head_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");


  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0;
  goal.trajectory.points[index].positions[1] = -0.5;

  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);


}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting run_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm controller action client to move the TIAGo's arm
  head_control_client_Ptr headClient;
  createHeadClient(headClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal head_goal;
  waypoints_head_goal(head_goal);

  // Sends the command to start the given trajectory 1s from now
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  headClient->sendGoal(head_goal);

  // Wait for trajectory execution
  while(!(headClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(10).sleep(); // sleep for four seconds
  }

  return EXIT_SUCCESS;
}
