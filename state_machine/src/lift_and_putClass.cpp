#include <state_machine/lift_and_putClass.h>


std::vector<moveit_msgs::CollisionObject> collision_objects;
std::vector<moveit_msgs::CollisionObject> collision_objects_new;
lift_and_putClass::lift_and_putClass(/* args */)
{
    object_position.x = 0;
    object_position.y = 0;
    object_position.z = 0;
}

lift_and_putClass::~lift_and_putClass()
{}

void lift_and_putClass::creategripperClient(gripper_control_client_Ptr& actionClient)
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


void lift_and_putClass::waypoints_open_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
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
void lift_and_putClass::openGripper_hard()
{
  gripper_control_client_Ptr gripperClient;
  creategripperClient(gripperClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_open_gripper_goal(gripper_goal);
  gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  gripperClient->sendGoal(gripper_goal);
}


void lift_and_putClass::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void lift_and_putClass::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.01;
  posture.points[0].positions[1] = 0.01;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void lift_and_putClass::addCollisionObjects()
{
  // Creating Environment
  //std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.6;
  collision_objects[0].primitives[0].dimensions[1] = 1.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.7;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.85;
  collision_objects[0].primitive_poses[0].position.y = 0.2;
  collision_objects[0].primitive_poses[0].position.z = 0.25;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "base_link";
  collision_objects[1].id = "object";
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
  collision_objects[1].primitives[0].dimensions.resize(2);
  collision_objects[1].primitives[0].dimensions[0] = 0.13/1;
  //collision_objects[1].primitives[0].dimensions[1] = 0.065/1;
  collision_objects[1].primitives[0].dimensions[1] = 0.036/1;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = object_position.x;
  collision_objects[1].primitive_poses[0].position.y = object_position.y;
  collision_objects[1].primitive_poses[0].position.z = object_position.z;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  ROS_INFO_STREAM("x: "<<collision_objects[1].primitive_poses[0].position.x\
                <<" y: "<<collision_objects[1].primitive_poses[0].position.y\
                <<" z: "<<collision_objects[1].primitive_poses[0].position.z);
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}



void lift_and_putClass::addCollisionObjects_new()
{
  // Creating Environment
  //std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects_new.resize(1);

  // Add the first table
  collision_objects_new[0].id = "table2";
  collision_objects_new[0].header.frame_id = "base_link";

  collision_objects_new[0].primitives.resize(1);
  collision_objects_new[0].primitives[0].type = collision_objects_new[0].primitives[0].BOX;
  collision_objects_new[0].primitives[0].dimensions.resize(3);
  collision_objects_new[0].primitives[0].dimensions[0] = 0.7;
  collision_objects_new[0].primitives[0].dimensions[1] = 1.2;
  collision_objects_new[0].primitives[0].dimensions[2] = 0.7;

  collision_objects_new[0].primitive_poses.resize(1);
  collision_objects_new[0].primitive_poses[0].position.x = 0.85;
  collision_objects_new[0].primitive_poses[0].position.y = 0.2;
  collision_objects_new[0].primitive_poses[0].position.z = 0.25;
  collision_objects_new[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects_new[0].operation = collision_objects_new[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects_new);
}

bool lift_and_putClass::pick()
{
  ROS_INFO_STREAM("Prepose finished");

  ros::WallDuration(10.0).sleep();
  while(object_position.x==0 && object_position.y==0 && object_position.z==0)
  {
    ros::WallDuration(1.0).sleep();
    ROS_INFO_STREAM("waiting");
  
  }
  ROS_INFO_STREAM("Command received, add collision");
  addCollisionObjects();
 
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  moveit::planning_interface::MoveGroupInterface group("arm_torso");
  group.setPlanningTime(45.0);
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(1.57, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = object_position.x-0.15;
  grasps[0].grasp_pose.pose.position.y = object_position.y;
  grasps[0].grasp_pose.pose.position.z = object_position.z;

  // Setting pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;

  // Setting post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1;
  grasps[0].post_grasp_retreat.min_distance = 0.05;
  grasps[0].post_grasp_retreat.desired_distance = 0.2;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture); 
  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);
  // Set support surface as table1.
  group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  ROS_INFO_STREAM("Ready to pick");
  group.setNumPlanningAttempts(5);
  group.pick("object", grasps);
  ROS_INFO("Finish pick");

  return true;
}
void lift_and_putClass::tuckarm()
{
    std::map<std::string, double> target_position;

    target_position["torso_lift_joint"] = 0;
    target_position["arm_1_joint"] = 0.2;
    target_position["arm_2_joint"] = -1.34;
    target_position["arm_3_joint"] = -0.2;
    target_position["arm_4_joint"] = 1.94;
    target_position["arm_5_joint"] = -1.57;
    target_position["arm_6_joint"] = 1.37;
    target_position["arm_7_joint"] = 0.00;

   
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::vector<std::string> torso_arm_joint_names;
    //select group of joints
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    //choose your preferred planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");

    torso_arm_joint_names = group_arm_torso.getJoints();

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(0.6);

    for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
      if ( target_position.count(torso_arm_joint_names[i]) > 0 )
      {
        ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
        group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
      }

    moveit::planning_interface::MoveGroupInterface::Plan my_first_plan;
    group_arm_torso.setPlanningTime(5.0);
    bool success_first = bool(group_arm_torso.plan(my_first_plan));

    if ( !success_first )
      throw std::runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_first_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start_first = ros::Time::now();  
    moveit::planning_interface::MoveItErrorCode error_first = group_arm_torso.move();
    if (!bool(error_first))
      throw std::runtime_error("Error executing plan");

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start_first).toSec());

    spinner.stop();
    collision_objects[0].operation = collision_objects[0].REMOVE;
    collision_objects[1].operation = collision_objects[1].REMOVE;
    planning_scene_interface.applyCollisionObjects(collision_objects);
  //return true;
}



bool lift_and_putClass::place()
{
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  addCollisionObjects_new();
  ROS_INFO_STREAM("Command received, place");
  moveit::planning_interface::MoveGroupInterface group("arm_torso");
  group.setPlanningTime(45.0);
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  
  // Setting place location pose
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0, 0);
  place_location[0].place_pose.header.frame_id = "base_link";
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0.9;
  place_location[0].place_pose.pose.position.y = 0.2;
  place_location[0].place_pose.pose.position.z = 0.7;

  // Setting pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.05;
  place_location[0].pre_place_approach.desired_distance = 0.1;

  // Setting post-grasp retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.x = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.05;
  place_location[0].post_place_retreat.desired_distance = 0.1;

  // Setting posture of eef after placing object
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL

  ROS_INFO_STREAM("Finish place");
  return true;
}
void lift_and_putClass::place_hard()
{
  addCollisionObjects_new();
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
  goal_pose.pose.position.z = 0.7;
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
  //openGripper();
  openGripper_hard();
  //openGripper(place_location[0].post_place_posture);
  ROS_INFO_STREAM("Finish place");
  //return true;
}

void lift_and_putClass::object_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  object_position.x = msg->x;
  object_position.y = msg->y;
  object_position.z = msg->z;
  //ROS_INFO_STREAM("3d coordinate of "<<" : "<<object_position.x<<" , "<<object_position.y<<" , "<<object_position.z);
}
