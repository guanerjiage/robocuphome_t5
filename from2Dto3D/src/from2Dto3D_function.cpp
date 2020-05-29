#include <from2Dto3D/from2Dto3D.h>

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{    
    //#>>>>TODO: store local data copy or shared, depending on the message
    pcl::fromROSMsg(*pc,pointcloud);   
    
}    

void From2Dto3D::processRect(const perception_msgs::RectArrayConstPtr& r)
{
    //#>>>>TODO: process bounding box and send 3D position to the topic   
    //get the names of objects from object_detection
    n = r->rects.size();
    string names[n];
    for(int i=0;i<n;i++)
    {
        dt_msgs.rects.push_back(r->rects[i]);
        names[i] = dt_msgs.rects[i].class_name;
    }

    //get 3d coordinate and publish it
    for(int i=0;i<n;i++)
    {
      cx = dt_msgs.rects[i].x;
      cy = dt_msgs.rects[i].y;
      position = pointcloud.width*cy+cx;
           
      for(int j=0;j<10;j++)
      {
          if (isnan(pointcloud.points[position].x)||isnan(pointcloud.points[position].y)||isnan(pointcloud.points[position].z))
          {
            position = position + 1;
          }
          else 
          {
	    msg.header.frame_id = "xtion_rgb_optical_frame";
	    msg.header.stamp = ros::Time();
	    msg.point.x=pointcloud.points[position].x;
 	    msg.point.y=pointcloud.points[position].y;
            msg.point.z=pointcloud.points[position].z;
	    //transform to base_link of robot
            try
            { 
              listener_.transformPoint("base_link", msg, base_point);
      	  
      	    }
      	    catch(tf::TransformException& ex)
            {
              ROS_ERROR("%s", ex.what());
      	    }

            ROS_INFO_STREAM("2d coordinate of "<<names[i]<<" : "<<cx<<" , "<<cy);
            ROS_INFO_STREAM("3d coordinate of "<<names[i]<<" : "<<base_point.point.x<<" , "<<base_point.point.y<<" , "<<base_point.point.z);
            pub_2Dto3D.publish(base_point);
            break;
          }
      }
    }

    dt_msgs.rects.clear();
    
}

