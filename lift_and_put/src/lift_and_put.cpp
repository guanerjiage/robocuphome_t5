#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PlaceLocation.h>
//#include <moveit_msgs/Grasp.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>

#include <lift_and_put/lift_and_putClass.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_node");
  ros::NodeHandle nh;
  lift_and_putClass lp;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = nh.subscribe("/Point3D",10, &lift_and_putClass::object_cb, &lp);
  ros::ServiceServer pick_service = nh.advertiseService("pick", &lift_and_putClass::pick, &lp);
  ros::ServiceServer place_service = nh.advertiseService("place", &lift_and_putClass::place, &lp);
  ros::waitForShutdown();
  return 0;
}