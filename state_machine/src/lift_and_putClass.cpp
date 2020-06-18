#include <state_machine/lift_and_putClass.h>

lift_and_putClass::lift_and_putClass(/* args */)
{
    object_position.x = 0;
    object_position.y = 0;
    object_position.z = 0;
}

lift_and_putClass::~lift_and_putClass()
{}


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
  posture.points[0].positions[0] = 0.025;
  posture.points[0].positions[1] = 0.025;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void lift_and_putClass::addCollisionObjects(int id)
{
  // Creating Environment
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  if(id==1) // add object and table1
  {
    collision_objects.resize(2);
    // Add the first table
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.6;
    collision_objects[0].primitives[0].dimensions[1] = 1.2;
    collision_objects[0].primitives[0].dimensions[2] = 0.7;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1.05-0.05;
    collision_objects[0].primitive_poses[0].position.y = 0.2;
    collision_objects[0].primitive_poses[0].position.z = 0.25;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Define the object that we will be manipulating
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].id = "object";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.05;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;
    collision_objects[1].primitives[0].dimensions[2] = 0.12;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = object_position.x;
    collision_objects[1].primitive_poses[0].position.y = object_position.y;
    collision_objects[1].primitive_poses[0].position.z = object_position.z;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    ROS_INFO_STREAM("x: "<<collision_objects[1].primitive_poses[0].position.x\
                  <<" y: "<<collision_objects[1].primitive_poses[0].position.y\
                  <<" z: "<<collision_objects[1].primitive_poses[0].position.z);
    collision_objects[2].operation = collision_objects[1].ADD;
    planning_scene_interface.applyCollisionObjects(collision_objects);
  }
  else if(id==2) // add collision of table 2
  {
    // Add the second table where we will be placing the cube.
    collision_objects.resize(1);
    collision_objects[0].id = "table2";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.6;
    collision_objects[0].primitives[0].dimensions[1] = 1.2;
    collision_objects[0].primitives[0].dimensions[2] = 0.7;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1.05;
    collision_objects[0].primitive_poses[0].position.y = 0.2;
    collision_objects[0].primitive_poses[0].position.z = 0.25;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;
    planning_scene_interface.addCollisionObjects(collision_objects);
  }
  else if(id==3) // remove the collision of table1
  {
    std::vector<std::string> table;
    table.resize(1);
    table[0]="table1";
    planning_scene_interface.removeCollisionObjects(table);
  }
  
}


bool lift_and_putClass::pick()
{
  ROS_INFO_STREAM("open detection");
  detect_flag = true;
  ros::WallDuration(10.0).sleep();
  while(object_position.x==0 && object_position.y==0 && object_position.z==0)
  {
    ros::WallDuration(1.0).sleep();
    ROS_INFO_STREAM("waiting");
  
  }
  detect_flag = false;
  ROS_INFO_STREAM("Command received, add collision");
  addCollisionObjects(1);
  ROS_INFO("add collision of object and table1 finished");
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
  grasps[0].grasp_pose.pose.position.x = object_position.x-0.17;
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
  //group.pick("object", grasps);
  bool result=0;
  if(group.pick("object", grasps)==-1) 
    result=0;
  else
    result=1;
  //ROS_INFO_STREAM("return code is"<<group.pick("object", grasps));
  ROS_INFO("Finish pick");
  
  // clear cost map for better planning
  std_srvs::Empty srv;
  ros::service::call("/move_base/clear_costmaps", srv);
	ROS_INFO("call service clear costmap");
 return result;
}

bool lift_and_putClass::place()
{
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  ROS_INFO_STREAM("Command received, add table2");
  addCollisionObjects(2);
  ROS_INFO("add collision of table2 finished");
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

void lift_and_putClass::object_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  if(detect_flag)
  {
    object_position.x = msg->x;
    object_position.y = msg->y;
    object_position.z = msg->z;
    ROS_INFO_STREAM("3d coordinate of "<<" : "<<object_position.x<<" , "<<object_position.y<<" , "<<object_position.z);

  }
}
