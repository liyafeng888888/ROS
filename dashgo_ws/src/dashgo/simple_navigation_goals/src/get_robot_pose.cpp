#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard odom:[%s]",msg->data.c_str());
    std::cout<<" chatterCallback run"<<std::endl;
}

void Callback_pose(const geometry_msgs::PoseConstPtr& pose)
{
    float x, y, theta;
    x=pose->position.x;
    y=pose->position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose->orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    theta=rpy.z;
    ROS_INFO("I heard pose x:[%f] y:[%f] z:[%f]",x, y, theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listen_pose");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/robot_pose",1000,Callback_pose);
    //std::cout<<" for test1"<<std::endl;
    ros::spin();

  return 0;
}
