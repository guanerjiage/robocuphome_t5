#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client>  gripper_control_client_Ptr;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//#include <moveit_visual_tools/moveit_visual_tools.h>
void creategripperClient(gripper_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new gripper_control_client("/gripper_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the gripper_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: gripper controller action server not available");
}


void waypoints_open_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints

  goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
  goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.4;
  goal.trajectory.points[index].positions[1] = 0.4;

  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

}
void waypoints_close_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints

  goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
  goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;

  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

}
void openGripper()
{
  gripper_control_client_Ptr gripperClient;
  creategripperClient(gripperClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_open_gripper_goal(gripper_goal);
  gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  gripperClient->sendGoal(gripper_goal);
}
void closeGripper()
{
  gripper_control_client_Ptr gripperClient;
  creategripperClient(gripperClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_close_gripper_goal(gripper_goal);
  gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  gripperClient->sendGoal(gripper_goal);
}
void addCollisionObjects()
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::CollisionObject> collision_objects;;
  collision_objects.resize(2);

  collision_objects[0].header.frame_id = "base_link";
  collision_objects[0].id = "box";
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.85;
  collision_objects[0].primitives[0].dimensions[1] = 0.91;
  collision_objects[0].primitives[0].dimensions[2] = 0.75;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.8259;
  collision_objects[0].primitive_poses[0].position.y = 0.01;
  collision_objects[0].primitive_poses[0].position.z = 0.33;


  collision_objects[0].operation = collision_objects[0].ADD;
  //std::vector<moveit_msgs::CollisionObject> collision_objects;
  //collision_objects.resize(2);
  //collision_objects[0].push_back(collision_objects[0]);

  ROS_INFO_NAMED("tutorial", "Add an obstacal into the world");
  //planning_scene_interface.applyCollisionObjects(collision_objects[0]);

  collision_objects[1].header.frame_id = "base_link";
  collision_objects[1].id = "object";
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.06;
  collision_objects[1].primitives[0].dimensions[1] = 0.06;
  collision_objects[1].primitives[0].dimensions[2] = 0.20;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.95;
  collision_objects[1].primitive_poses[0].position.y = 0.01;
  collision_objects[1].primitive_poses[0].position.z = 0.7715;


  collision_objects[1].operation = collision_objects[1].ADD;
 // std::vector<moveit_msgs::CollisionObject> object;
  //collision_objects[1].push_back(collision_objects[1]);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.applyCollisionObjects(collision_objects);

}

void pick()
{
    // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.

  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  group_arm_torso.setPlannerId("SBLkConfigDefault");//choose the planner
  group_arm_torso.setPoseReferenceFrame("base_link");
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(0.3);
  group_arm_torso.setPlanningTime(10.0);


  geometry_msgs::PoseStamped goal_pose;//the goal position
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = 0.60;
  goal_pose.pose.position.y = 0.01;
  goal_pose.pose.position.z = 0.7715;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);

  ROS_INFO_STREAM("the frame is:" << goal_pose.header.frame_id);
  ROS_INFO_STREAM("Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Point z : " << goal_pose.pose.position.z);
  //select group of joints
  group_arm_torso.setPoseTarget(goal_pose);//give the position
  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());
  //set the plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_front;
  bool success = bool(group_arm_torso.plan(my_plan_front));
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan_front.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  //openGripper();
  
  ros::WallDuration(2.0).sleep();
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = 0.8;
  goal_pose.pose.position.y = 0.01;
  goal_pose.pose.position.z = 0.7715;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);

  ROS_INFO_STREAM("the frame is:" << goal_pose.header.frame_id);
  ROS_INFO_STREAM("Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Point z : " << goal_pose.pose.position.z);
  //select group of joints
  group_arm_torso.setPoseTarget(goal_pose);//give the position
  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());
  //set the plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success_pick = bool(group_arm_torso.plan(my_plan));
  if ( !success_pick )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start_front = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  closeGripper();


  
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = 0.6;
  goal_pose.pose.position.y = 0.01;
  goal_pose.pose.position.z = 1;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
  ROS_INFO_STREAM("the frame is:" << goal_pose.header.frame_id);
  ROS_INFO_STREAM("Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Point z : " << goal_pose.pose.position.z);
  //select group of joints
  group_arm_torso.setPoseTarget(goal_pose);//give the position
  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());
  //set the plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_rise;
  bool success_pick_rise = bool(group_arm_torso.plan(my_plan_rise));
  if ( !success_pick_rise )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan_rise.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start_front_rise = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
 
}
void place()
{
	moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  group_arm_torso.setPlannerId("SBLkConfigDefault");//choose the planner
  group_arm_torso.setPoseReferenceFrame("base_link");
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(0.3);
  group_arm_torso.setPlanningTime(10.0);

  geometry_msgs::PoseStamped goal_pose;//the goal position
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = 0.60;
  goal_pose.pose.position.y = 0.2;
  goal_pose.pose.position.z = 0.7715;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);

  ROS_INFO_STREAM("the frame is:" << goal_pose.header.frame_id);
  ROS_INFO_STREAM("Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Point z : " << goal_pose.pose.position.z);
  //select group of joints
  group_arm_torso.setPoseTarget(goal_pose);//give the position
  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());
  //set the plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_place;
  bool success_place = bool(group_arm_torso.plan(my_plan_place));
  if ( !success_place )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan_place.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  openGripper();


}

void go_andere_table()
{
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x=0.0;
  goal.target_pose.pose.position.y=2;
  goal.target_pose.pose.orientation.x=0.0;
  goal.target_pose.pose.orientation.y=0.0;
  goal.target_pose.pose.orientation.z=1.0;
  goal.target_pose.pose.orientation.w=0.0;

  ROS_INFO("Sending first goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("First goal reached");   
  else
      ROS_INFO("The base failed to move to the goal for some reason");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 // moveit::planning_interface::MoveGroupInterface group("arm_torso");
 // group.setPlanningTime(45.0);

  addCollisionObjects();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick();
	place();

  ros::WallDuration(1.0).sleep();
  //go_andere_table();
  

  ros::waitForShutdown();
  return 0;
}



  












 

