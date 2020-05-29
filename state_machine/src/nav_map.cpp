#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <state_machine/command.h>
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

bool move(state_machine::command::Request &req, state_machine::command::Response &res)
{
	double target[4] = {0, 0, 0, 0};
    switch(req.type)
	{
		case 1:
			target[0] = 0.6535;
			target[1] = 0.4;
			target[2] = 0.71;
			target[3] = 0.702;
			break;
		case 2:
			target[0] = 0.6535;
			target[1] = 0.4;
			target[2] = 0.71;
			target[3] = 0.702;
			break;
		case 3:
			break;
		case 4:
			break;
	}
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = target[0];
	goal.target_pose.pose.position.y = target[1];
	goal.target_pose.pose.orientation.z = target[2];
	goal.target_pose.pose.orientation.w = target[3];
	ROS_INFO_STREAM("Sending goal to position.");
	MoveBaseClient ac("move_base", true);
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
		res.reply = 1;
	}
	else
	{
		ROS_INFO("Failed to move forward to the target");
		res.reply = 0;
	}
	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_the_map");
	ros::NodeHandle n;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	ros::ServiceServer service=n.advertiseService("myMove", move);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ros::spin();
  return 0;
}
