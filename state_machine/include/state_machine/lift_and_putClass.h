#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PlaceLocation.h>
#include <moveit_msgs/Grasp.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client>  gripper_control_client_Ptr;

class lift_and_putClass
{
private:
    geometry_msgs::Point object_position;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
public:
    lift_and_putClass(/* args */);
    ~lift_and_putClass();
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void addCollisionObjects();
    void addCollisionObjects_new();
    void openGripper_hard();
    bool pick();
    bool place();
    void place_hard();
    void tuckarm();

    void creategripperClient(gripper_control_client_Ptr& actionClient);
    void waypoints_open_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal);
    //bool pick(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    //bool place(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void object_cb(const geometry_msgs::Point::ConstPtr& msg);
};
