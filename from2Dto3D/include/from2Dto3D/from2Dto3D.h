#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Char.h>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

//#include <image_geometry/pinhole_camera_model.h>

#include <perception_msgs/Rect.h>
#include <perception_msgs/RectArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
//#>>>>TODO: FIX DEPENDENCIES IN CMakeLists.txt and package.xml (Make sure that everithing compiles in one shot.)

//#>>>>TODO: Separate this template in a class library (.h and .cpp files) and an node (.cpp file). The header files must be in a "include" folder in the package.

//#>>>>TODO: Deliver the computed 3D point transformed into the base_link frame (or map/odom frame).

using namespace std;
using namespace cv;

class From2Dto3D
{

    private:
      // The node handle
      ros::NodeHandle nh_;
      // Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //#>>>>TODO: Define publishers and subscribers
      ros::Publisher pub_2Dto3D;
      ros::Subscriber sub_point_cloud;
      ros::Subscriber sub_rect_msg;
      //ros::Subscriber sub_rec_;


      //#>>>>TODO: Define the pointcloud structure and the bounding box local copy
      pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
      geometry_msgs::PointStamped msg;
      geometry_msgs::PointStamped base_point;
      
      // A tf transform listener if needed
      tf::TransformListener listener_;
      //detection_msg from object_detection
      perception_msgs::RectArray dt_msgs;

      //------------------ Callbacks -------------------

      // Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      // Process bounding boxes
      void processRect(const perception_msgs::RectArrayConstPtr & r);

      float cx,cy;
      int n,position;

     public:
      // Subscribes to and advertises topics

      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        sub_point_cloud=nh_.subscribe<sensor_msgs::PointCloud2>("/xtion/depth_registered/points",1,&From2Dto3D::processCloud,this);
        sub_rect_msg=nh_.subscribe<perception_msgs::RectArray>("/rect",1,&From2Dto3D::processRect,this);
        pub_2Dto3D = nh_.advertise<geometry_msgs::Point>("/Point3D",100);
	
        ROS_INFO("from2Dto3D initialized ...");

      }

      ~From2Dto3D() {}
};



















