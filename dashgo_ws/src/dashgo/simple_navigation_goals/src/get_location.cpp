#include <ros/ros.h>
#include <tf/transform_listener.h>
 #include "tf/tf.h"  
int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");
 
  ros::NodeHandle node;
  double person_x;
  double person_y;
  double person_yaw;
 
  tf::TransformListener listener;
 
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/uwb_tag",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    person_x = transform.getOrigin().x();
    person_y = transform.getOrigin().y();
    ROS_INFO("it is person_x=%f",person_x);
    ROS_INFO("it is person_y=%f",person_y);
 
    rate.sleep();
  }
  return 0;
}
