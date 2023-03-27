#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>


class Framer
{
    public:
    Framer() 
    {
      n_.getParam("poses_supplied", poses_supplied);
      

      
    if(poses_supplied){
          transformsub_ =
          n_.subscribe("body", 3, &Framer::poseCallback, this);
        }
    else{
          transformsub_ =
          n_.subscribe("body", 3, &Framer::transformCallback, this);
        }
        
        
  
    }

    void transformCallback(const geometry_msgs::TransformStampedConstPtr &msg) 
    {
      //ros::Time ros_time = ros::Time::now();
      
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.transform = msg->transform;
      tf_msg.header.seq = 0;
      tf_msg.header.frame_id = "world";
      tf_msg.child_frame_id = "body";
      tf_msg.header.stamp = msg->header.stamp;
      T_bw_pub_.sendTransform(tf_msg);
    }

    void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg) 
    {
      //ros::Time ros_time = ros::Time::now();
      ROS_INFO("posecb");
      geometry_msgs::TransformStamped transform_stamped;
      
      transform_stamped.transform.translation.x = msg->pose.position.x;
      transform_stamped.transform.translation.y = msg->pose.position.y;
      transform_stamped.transform.translation.z = msg->pose.position.z;

      transform_stamped.transform.rotation.w = msg->pose.orientation.w;
      transform_stamped.transform.rotation.x = msg->pose.orientation.x;
      transform_stamped.transform.rotation.y = msg->pose.orientation.y;
      transform_stamped.transform.rotation.z = msg->pose.orientation.z;

      transform_stamped.header.stamp = msg->header.stamp;
      transform_stamped.header.frame_id = "body";
      transform_stamped.child_frame_id = "world";
      T_bw_pub_.sendTransform(transform_stamped);
    }

    private:
    ros::NodeHandle n_ = *(new ros::NodeHandle("~"));
    bool poses_supplied = false;
    tf2_ros::TransformBroadcaster T_bw_pub_;
    std::string cam_frame_topic = "dense_cam";
    ros::Subscriber transformsub_;
    
};

auto main(int argc, char **argv) -> int
{
 
  ros::init(argc, argv, "dense_verifier");



  Framer framerObject;
  
  
// %Tag(SPIN)%

  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%