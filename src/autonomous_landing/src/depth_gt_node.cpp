#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/mat.hpp>
#include "opencv2/imgproc.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include <cv_bridge/cv_bridge.h>

class DepthNode
{
    public:
    ros::NodeHandle n_ = *(new ros::NodeHandle("~"));
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    ros::Publisher depth_pub_;
    image_transport::ImageTransport it = *(new image_transport::ImageTransport(n_));
    image_transport::Subscriber imgsub_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;

    DepthNode()
    {
        imgsub_ = it.subscribe("/cam_left_depth", 60,
                             &DepthNode::imageCallback, this);
        depth_pub_ =
          n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("dense_pointcloud", 5);

        model_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        model_cloud->header.frame_id = "world";
        tf_buffer = std::make_unique<tf2_ros::Buffer>();
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        
        voxgrid.setLeafSize(0.1, 0.1, 0.1);
        voxgrid.setDownsampleAllData(true);
        voxgrid.setMinimumPointsNumberPerVoxel(1);
        voxgrid.setFilterLimits(-10.0, 200.0);
        
    
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        cv::Mat depth_image = cv_bridge::toCvShare(msg)->image.clone();
        cv::Mat color_image;
        unsigned long t_nsecs = msg->header.stamp.toNSec();
        ROS_INFO("stamp %lu", t_nsecs);
        
        unsigned long number = (t_nsecs - 1000000000000);
        
        int corr = std::rint((double)number/40000000.0);
                           
        ROS_INFO("nsec since 0 %lu", number);
        ROS_INFO("number %d", corr);
        std::string img_number = std::to_string(corr);
        unsigned long n_zero = 6;
        std::string img_name = std::string(n_zero - std::min(n_zero, img_number.length()), '0') + img_number + ".JPEG";
        color_image = cv::imread(std::string("/home/nvidia/AutonomousLanding/MidAir/PLE_training/fall/color_left/trajectory_4000/") + img_name);
        ROS_INFO("img name %s", img_name.c_str());
        ROS_INFO("color %d", color_image.size[0]);
        geometry_msgs::Transform  new_transform;
        try {
        new_transform =
            tf_buffer
                ->lookupTransform("world", "dense_cam",
                                  msg->header.stamp, ros::Duration(7.0))
                .transform;
        
      }
      catch (tf2::TransformException& e) {
        ROS_INFO("%s",e.what());
        
        return;
      }

      Eigen::Matrix4f transform_eig = tf2::transformToEigen(new_transform).matrix().cast<float>();
        
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        //pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
        cloud->header.frame_id = "dense_cam";
        cloud->height = 1024;
        cloud->width = 1024;
        cloud->is_dense = false;
        cloud->resize(1024*1024);

        depth_image.forEach<float>([this, cloud, color_image](float &pixel, const int position[]) -> void {
            if(pixel < 100.0)
            {
              pcl::PointXYZRGB point;
              float rxy = 
              pixel/(std::sqrt(
                (position[1]-512.0)*(position[1]-512.0) + 
                (position[0]-512.0)*(position[0]-512.0) +
                512.0*512.0
              ));
              point.x = (position[1] - 512.0)*rxy;
              point.y = (position[0] - 512.0)*rxy;
              point.z = 512.0*rxy;
              auto rgb_vec = color_image.at<cv::Vec3b>(position[0], position[1]);
              point.r = rgb_vec[2];
              point.g = rgb_vec[1];
              point.b = rgb_vec[0];
              cloud->at(position[1], position[0]) = point;
            }
            
        });
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform_eig);
        *model_cloud += *transformed_cloud;
        //if(corr % 10 == 0){
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        
          voxgrid.setInputCloud(this->model_cloud);
          voxgrid.filter(*out_cloud);
          this->model_cloud = out_cloud;
          this->model_cloud->header.frame_id = "world";
          depth_pub_.publish(this->model_cloud);
        //}
        
        
        //depth_pub_.publish(cloud);
        if(corr == 2204){
          //pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        
          //voxgrid.setInputCloud(this->model_cloud);
          //voxgrid.filter(*out_cloud);
          
          pcl::io::savePCDFile(std::string("/home/nvidia/AutonomousLanding/MidAir/midair.pcd"), *this->model_cloud);
        }

        
        

    }

};

auto main(int argc, char **argv) -> int
{
 
  ros::init(argc, argv, "depth_gt_node");



  DepthNode DepthNodeObject;
  
  
// %Tag(SPIN)%

  ros::spin();
// %EndTag(SPIN)%

  return 0;
}