#include <object_detection/object_detection.h>



int main(int argc,char **argv)
{
  
  ros::init(argc,argv,"object_detection");
  ros::NodeHandle node_detection;
  object_detection node(node_detection);

  ros::spin();
  

  
  return 0;
}

