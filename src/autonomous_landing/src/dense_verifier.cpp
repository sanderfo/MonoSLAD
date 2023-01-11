#include "ros/ros.h"
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include "psl_base/cameraMatrix.h"
#include <psl_stereo/cudaPlaneSweep.h>
#include "utilities.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class DenseVerifier
{
    public:
    DenseVerifier() 
    {
        n_.getParam("safetyZoneSize", safetyZoneSize);
        n_.getParam("sweep_n", sweep_n);

        pub_ = n_.advertise<PointCloud>("autonomous_landing/landing_pointcloud", 2);

        imgsub_ = it.subscribe("/cam0/image_raw", 1, &DenseVerifier::imageCallback, this);
        posesub_ = n_.subscribe("/svo/backend_pose_imu", 2, &DenseVerifier::poseCallback, this);
        landingsub_ = n_.subscribe("/autonomous_landing/landing_pointcloud", 2, &DenseVerifier::landingCallback, this);

        K(0,0) = 458.654;
        K(0,1) = 0;
        K(0,2) = 367.215;
        K(1,0) = 0;
        K(1,1) = 457.296;
        K(1,2) = 248.375;
        K(2,0) = 0;
        K(2,1) = 0;
        K(2,2) = 1;
  
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        
        ROS_INFO("image timestamp %d %d", msg->header.stamp.sec, msg->header.stamp.nsec);
        image_candidate_vector.push_back(msg);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        
        ROS_INFO("pose timestamp %d %d", msg->header.stamp.sec, msg->header.stamp.nsec);
        double distanceSquared = 
            std::pow(msg->pose.pose.position.x - current_pose.position.x, 2)
          + std::pow(msg->pose.pose.position.y - current_pose.position.y, 2)
          + std::pow(msg->pose.pose.position.z - current_pose.position.z, 2);
        if(distanceSquared > 1) {
            pose_array[pose_counter] = msg->pose.pose;
            current_pose = msg->pose.pose;

            for(int i = 0; i < image_candidate_vector.size(); i++) {
                if (msg->header.stamp.nsec == image_candidate_vector[i]->header.stamp.nsec 
                    && msg->header.stamp.sec == image_candidate_vector[i]->header.stamp.sec) {
                    
                    image_sweeping_array[pose_counter] = cv_bridge::toCvShare(image_candidate_vector[i]);
                    image_candidate_vector.erase(image_candidate_vector.begin(), image_candidate_vector.begin()+i);
                    break;
                }

            }

            pose_counter += 1;

            if(pose_counter == sweep_n) {
                planeSweep();
                pose_counter = 0;
                
            }
        };
    }

    void landingCallback(const PointCloud::ConstPtr& msg) {
        
    }

    private:
    ros::NodeHandle n_ = *(new ros::NodeHandle("~"));
    image_transport::ImageTransport it = *(new image_transport::ImageTransport(n_));
    ros::Publisher pub_;
    image_transport::Subscriber imgsub_;
    ros::Subscriber posesub_;
    ros::Subscriber landingsub_;

    float safetyZoneSize;
    int sweep_n;
    Eigen::Matrix3d K;

    std::vector<sensor_msgs::ImageConstPtr> image_candidate_vector;
    std::array<cv_bridge::CvImageConstPtr, 10> image_sweeping_array;

    std::array<geometry_msgs::Pose, 10> pose_array;
    geometry_msgs::Pose current_pose;
    int pose_counter = 0;

    void planeSweep() 
    {
        std::vector<PSL::CameraMatrix<double>> cameras;

        for (int c = 0; c < sweep_n; c++) 
        {
            cameras[c].setKRT(K, util::poseToRotation(pose_array[c]), util::poseToPosition(pose_array[c]));
            
        };



    };
};

auto main(int argc, char **argv) -> int
{
 
  ros::init(argc, argv, "dense_verifier");



  DenseVerifier denseVerifierObject;
  
  
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%