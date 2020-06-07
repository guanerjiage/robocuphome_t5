#include <object_detection/object_detection.h>


void object_detection::show_class(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{

  for(int i=0; i<msg->bounding_boxes.size();i++)
    {	
      detection_msg.class_name = msg->bounding_boxes[i].Class;		
      detection_msg.x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax)/2;
      detection_msg.y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax)/2;
      detection_msg.height = (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);
      detection_msg.width = (msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin);
      //pub.publish(detection_msg);
      detection_msgs.rects.push_back(detection_msg);
      //ROS_INFO_STREAM( "Get the 2D position of: " <<detection_msg.class_name << " x: " <<detection_msg.x << " y: " <<detection_msg.y << " height: " <<detection_msg.height << " width: " << detection_msg.width);
            
    }
    
    pub.publish(detection_msgs);
    detection_msgs.rects.clear();


}
object_detection::object_detection(ros::NodeHandle node_detection): node_detection(node_detection)
{
  
      pub = node_detection.advertise<perception_msgs::RectArray>("rect",100);
      sub = node_detection.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&object_detection::show_class,this);
}
