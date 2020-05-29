#include <from2Dto3D/from2Dto3D.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    From2Dto3D node(nh);
    tf::TransformListener listener(ros::Duration(10));
    ros::spin();
    return 0;
}

