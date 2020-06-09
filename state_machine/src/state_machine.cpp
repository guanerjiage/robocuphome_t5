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
#include <std_srvs/Trigger.h>
#include <state_machine/lift_and_putClass.h>


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
  goal.trajectory.points[index].positions[1] = -0.55;

  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);


}

// localization and look down to the table
void init(ros::NodeHandle n)
{
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
	int c = 35;
	while(c)
	{
		pub.publish(msg);
		ROS_INFO_STREAM("send message to turn "<<c);
		c--;
		ros::Duration(0.8).sleep();
	}
	ros::service::call("/move_base/clear_costmaps", srv);
	ROS_INFO("call service clear costmap");

  // move the head down to look the table
  ROS_INFO_STREAM("Move your head down.");
  head_control_client_Ptr headClient;
  createHeadClient(headClient);
  control_msgs::FollowJointTrajectoryGoal head_goal;
  waypoints_head_goal(head_goal);
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  headClient->sendGoal(head_goal);
  // Wait for trajectory execution
  while(!(headClient->getState().isDone()) && ros::ok())
  {
      ros::Duration(4).sleep(); // sleep for four seconds
  }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_the_map");
	ros::NodeHandle n;
 
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
  lift_and_putClass lp;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = n.subscribe("/Point3D",10, &lift_and_putClass::object_cb, &lp);
  ros::ServiceClient move_client=n.serviceClient<state_machine::command>("myMove");

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
    init(n);
    // remember the default pose
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    geometry_msgs::PoseStamped default_pose = group_arm_torso.getCurrentPose();
    ROS_INFO("inilializition finished");
    // walk in front of the table
    state_machine::command msg;
    msg.request.type = 1;
    move_client.call(msg);

    ROS_INFO("start to call pick");
    lp.pick();
    ROS_INFO("return from pick");

    // move back to default pose
    group_arm_torso.setPoseTarget(default_pose);
    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(2.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group_arm_torso.setPlanningTime(5.0);
    group_arm_torso.plan(my_plan);
    ros::Time start = ros::Time::now();
    group_arm_torso.move();
    ROS_INFO_STREAM("Back to default pose");

    ros::WallDuration(3.0).sleep();
    // walk to the second table
    msg.request.type = 2;
    move_client.call(msg);
    ROS_INFO("start to call place");
    lp.place();
    ROS_INFO("return from place");

    ros::waitForShutdown();
}