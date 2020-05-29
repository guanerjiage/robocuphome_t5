#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>
#include <map>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <exception>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;

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
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = -0.65;

  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_the_map");
	ros::NodeHandle n;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	// initial localization
	std_srvs::Empty srv;
	ros::service::call("/global_localization", srv);
	ROS_INFO("call service global localization");
	// emulate key_tele
	ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 2;
	int c = 30;
	while(c)
	{
		pub.publish(msg);
		ROS_INFO_STREAM("send message to turn "<<c);
		c--;
		ros::Duration(0.8).sleep();
	}
	ros::service::call("/move_base/clear_costmaps", srv);
	ROS_INFO("call service clear costmap");


	double target[4] = {0.6535, 0.4, 0.71, 0.702};
	int count = 0;
	while (ros::ok())
	{
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = target[0];
		goal.target_pose.pose.position.y = target[1];
		goal.target_pose.pose.orientation.z = target[2];
		goal.target_pose.pose.orientation.w = target[3];
		ROS_INFO_STREAM("Sending goal to position.");
		ac.sendGoal(goal);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO_STREAM("Reach position.");
			if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
 		{
   			ROS_FATAL("Timed-out waiting for valid time.");
    			return EXIT_FAILURE;
  		}
			head_control_client_Ptr headClient;
  			createHeadClient(headClient);
			control_msgs::FollowJointTrajectoryGoal head_goal;
  			waypoints_head_goal(head_goal);

  			// Sends the command to start the given trajectory 1s from now
 			head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  			headClient->sendGoal(head_goal);

  			// Wait for trajectory execution
 			 while(!(headClient->getState().isDone()) && ros::ok())
  			{
    			ros::Duration(4).sleep(); // sleep for four seconds
  			}
                        return EXIT_SUCCESS;
			//move_head();
			}
		else
			ROS_INFO("Failed to move forward to the target");
  }
  return 0;
}
