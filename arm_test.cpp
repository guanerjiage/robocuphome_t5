#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client>  gripper_control_client_Ptr;

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
  moveit_msgs::CollisionObject collision_objects;

  collision_objects.header.frame_id = "base_link";
  collision_objects.id = "box";
  collision_objects.primitives.resize(1);
  collision_objects.primitives[0].type = collision_objects.primitives[0].BOX;
  collision_objects.primitives[0].dimensions.resize(3);
  collision_objects.primitives[0].dimensions[0] = 0.8;
  collision_objects.primitives[0].dimensions[1] = 2.3;
  collision_objects.primitives[0].dimensions[2] = 0.9;

  collision_objects.primitive_poses.resize(1);
  collision_objects.primitive_poses[0].position.x = 0.88;
  collision_objects.primitive_poses[0].position.y = 0.01;
  collision_objects.primitive_poses[0].position.z = 0.5;


  collision_objects.operation = collision_objects.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects_test;
  collision_objects_test.push_back(collision_objects);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.applyCollisionObjects(collision_objects_test);
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
  goal_pose.pose.position.x = 0.5;
  goal_pose.pose.position.y = -0.027;
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
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_front;
  bool success = bool(group_arm_torso.plan(my_plan_front));
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan_front.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  /*openGripper();
  ros::WallDuration(2.0).sleep();



  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = 0.84;
  goal_pose.pose.position.y = -0.027;
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
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success_pick = bool(group_arm_torso.plan(my_plan));
  if ( !success_pick )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start_front = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());*/
  closeGripper();
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

  ros::WallDuration(1.0).sleep();

  

  ros::waitForShutdown();
  return 0;
}



  












 

