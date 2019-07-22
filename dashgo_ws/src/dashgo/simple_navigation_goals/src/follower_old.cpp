#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h> 
#include "std_msgs/String.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"    

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 
#define GOHOME "HOME"
double person_x=0;
double person_y=0;
double person_yaw=0;
double robot_x=0; 
double robot_y=0;
double robot_theta=0;
bool Callback_flag = false;
int goal_succeed = 1;
double dist=0.0;
string msg_str = "";


/*******************************默认amcl初始点******************************************/
typedef struct _POSE
{
  double X;
  double Y;
  double Z;
  double or_x;
  double or_y;
  double or_z;
  double or_w;
} POSE;
//POSE pose1 = {-2.136, 0.997, 0.0,  0.0, 0.0, -0.264, 0.964};
//POSE pose2 = {1.190, 2.517, 0.0,  0.0, 0.0, 0.872, 0.490};
//POSE pose3 = {0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0};
//POSE pose4 = {0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0};
POSE goal_pose; 
void setHome( ros::Publisher pub)
{
    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
    msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = 0.1;
    msg_poseinit.pose.pose.position.y = 0;
    msg_poseinit.pose.pose.position.z = 0;
    msg_poseinit.pose.pose.orientation.x = 0.0;
    msg_poseinit.pose.pose.orientation.y = 0.0;
    msg_poseinit.pose.pose.orientation.z = 0;
    msg_poseinit.pose.pose.orientation.w = 1;
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
}
 
void setGoal(POSE pose)
{
     //tell the action client that we want to spin a thread by default 
    MoveBaseClient ac("move_base", true); 
       
    //wait for the action server to come up 
    while(!ac.waitForServer(ros::Duration(5.0))){ 
        ROS_WARN("Waiting for the move_base action server to come up"); 
    } 
       
    move_base_msgs::MoveBaseGoal goal; 
       
    //we'll send a goal to the robot to move 1 meter forward 
    goal.target_pose.header.frame_id = "map"; 
    goal.target_pose.header.stamp = ros::Time::now(); 
    
    goal.target_pose.pose.position.x = pose.X;
    goal.target_pose.pose.position.y = pose.Y; 
    goal.target_pose.pose.position.z = pose.Z;  
    goal.target_pose.pose.orientation.x = pose.or_x;
    goal.target_pose.pose.orientation.y = pose.or_y;
    goal.target_pose.pose.orientation.z = pose.or_z;
    goal.target_pose.pose.orientation.w = pose.or_w;  
     
    ROS_INFO("Sending goal"); 
     
    ac.sendGoal(goal); 
       
    ac.waitForResult(); 
       
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
       ROS_INFO("it is successfu ");
    } 
    else 
       ROS_ERROR("The base failed  move to goal!!!");
    goal_succeed=1; 
    ROS_INFO("it is goal_succeed = %d ", goal_succeed);  
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard odom:[%s]",msg->data.c_str());
    std::cout<<" chatterCallback run"<<std::endl;
}

void Callback_pose(const geometry_msgs::PoseConstPtr& pose)
{   
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
    robot_x=pose->position.x;
    robot_y=pose->position.y;
    robot_theta=rpy.z;
    //ROS_INFO("I heard robot_pose x:[%f] y:[%f] z:[%f]",robot_x, robot_y, robot_theta);
    dist = sqrt((robot_x-person_x) * (robot_x-person_x) + (robot_y-person_y) * (robot_y-person_y));  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower");
    ros::NodeHandle n; 
    tf::TransformListener listener;
    ros::Publisher pub_initialpose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    ros::Subscriber sub = n.subscribe("/robot_pose",1000,Callback_pose);
    //ros::Rate rate(10.0);
    setHome(pub_initialpose);
    //std::cout<<" for test1"<<std::endl;
    //ros::spin();
  while (n.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/uwb_tag",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.5).sleep();// sleep for half a second
      continue;
    }
    person_x = transform.getOrigin().x();
    person_y = transform.getOrigin().y();
    //ROS_INFO("it is person_x=%f",person_x);
    //ROS_INFO("it is person_y=%f",person_y);
    if(dist > 1.2)     //get minimum distance   
       {  
         //ros::Publisher　 cancle_pub_ =  nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
         //actionlib_msgs::GoalID　first_goal;
         //cancle_pub_.publish(first_goal);
         goal_succeed=0;
         dist = 0.0;
         goal_pose.X=person_x;
         goal_pose.Y=person_y;
         goal_pose.Z=0.0;
         goal_pose.or_x=0.0;
         goal_pose.or_y=0.0;
         goal_pose.or_z=0.0;
         goal_pose.or_w=1.0;
         setGoal(goal_pose);
         ROS_INFO("send gasl,goal_succeed = %d ", goal_succeed);
    }
    ros::Duration(2).sleep(); 
    ros::spinOnce();    
    //rate.sleep();
  }

  return 0;
}
