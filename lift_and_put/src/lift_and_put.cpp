#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>

// global variables
geometry_msgs::Point object_position;
//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.4;
  posture.points[0].positions[1] = 0.4;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

/*
void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  moveit::planning_interface::MoveGroupInterface group("arm_torso");
  group.setPlanningTime(45.0);
  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  ///* For place location, we set the value to the exact location of the center of the object. 
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  ///* Defined with respect to frame_id 
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  ///* Direction is set as negative z axis 
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  ///* Defined with respect to frame_id 
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  ///moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  

  addCollisionObjects(planning_scene_interface);
ce_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  ///* Similar to the pick case 
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}
*/
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_footprint";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.6;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.58;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 1.05;
  collision_objects[0].primitive_poses[0].position.y = 0.2;
  collision_objects[0].primitive_poses[0].position.z = 0.29;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  /*
  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  ///* Define the primitive and its dimensions. 
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  // /* Define the pose of the table. 
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;
  */
  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "base_footprint";
  collision_objects[1].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.06/1;
  collision_objects[1].primitives[0].dimensions[1] = 0.06/1;
  collision_objects[1].primitives[0].dimensions[2] = 0.12/1;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = object_position.x;
  collision_objects[1].primitive_poses[0].position.y = object_position.y;
  collision_objects[1].primitive_poses[0].position.z = object_position.z;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void pre_pose()
{
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  group_arm_torso.setPlannerId("SBLkConfigDefault");//choose the planner
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  ros::AsyncSpinner spinner(1); 
	spinner.start();
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1);
  group_arm_torso.setPlanningTime(10.0);
  //set goal position
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = 0.6;
  goal_pose.pose.position.y = 0.2;
  goal_pose.pose.position.z = 0.8;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
  group_arm_torso.setPoseTarget(goal_pose);//give the position
  //set the plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.plan(my_plan);
  // Execute the plan
  group_arm_torso.move();
  spinner.stop();
}

bool pick(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  while(object_position.x==0 && object_position.y==0 && object_position.z==0)
    ros::WallDuration(1.0).sleep();
  ROS_INFO_STREAM("Command received, lift and put");
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionObjects(planning_scene_interface);
  pre_pose();
  ROS_INFO_STREAM("Prepose finished");
  
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  moveit::planning_interface::MoveGroupInterface group("arm_torso");
  group.setPlanningTime(45.0);
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "base_footprint";
  tf2::Quaternion orientation;
  orientation.setRPY(1.57, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = object_position.x-0.05;
  grasps[0].grasp_pose.pose.position.y = object_position.y+0.1;
  grasps[0].grasp_pose.pose.position.z = object_position.z;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.01;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.x = -1;
  grasps[0].post_grasp_retreat.min_distance = 0.05;
  grasps[0].post_grasp_retreat.desired_distance = 0.01;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  ROS_INFO_STREAM("Ready to pick");
  
  group.pick("object", grasps);
  // END_SUB_TUTORIAL
  ROS_INFO_STREAM("Finish to pick");
  res.success = true;
  return true;
}

void object_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  object_position.x = msg->x;
  object_position.y = msg->y;
  object_position.z = msg->z;
  //ROS_INFO_STREAM("3d coordinate of "<<" : "<<object_position.x<<" , "<<object_position.y<<" , "<<object_position.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = nh.subscribe("/Point3D",10, object_cb);
  ros::ServiceServer pick_service = nh.advertiseService("pick", pick);
  ros::WallDuration(1.0).sleep();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}