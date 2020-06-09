#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PlaceLocation.h>
#include <moveit_msgs/Grasp.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>


class lift_and_putClass
{
private:
    geometry_msgs::Point object_position;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool detect_flag = false;
public:
    lift_and_putClass(/* args */);
    ~lift_and_putClass();
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void addCollisionObjects(int id);
    bool pick();
    bool place();
    //bool pick(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    //bool place(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void object_cb(const geometry_msgs::Point::ConstPtr& msg);
};
