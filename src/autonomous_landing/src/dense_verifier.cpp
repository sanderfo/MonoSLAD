#include "ros/ros.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>
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
      n_.getParam("safety_zone_size", safety_zone_size);
      n_.getParam("sweep_n", sweep_n);
      n_.getParam("matching_threshold", matching_threshold);
      n_.getParam("distance_threshold", distance_threshold_squared);
      n_.getParam("uniqueness_threshold", uniqueness_threshold);
      n_.getParam("pointcloud_filter_radius", filter_radius);
      n_.getParam("pointcloud_filter_min_neighbours", filter_neighbours);

      std::vector<double> k_list;
      n_.getParam("camera/intrinsics", k_list);

      std::string dist_model;
      n_.getParam("camera/distortion_model", dist_model);
      if (dist_model == "none") image_is_distorted = false;

      n_.getParam("camera/distortion", dist_coeffs);
      
      
      std::vector<double> tbc_list;
      n_.getParam("T_B_C", tbc_list);


      

      distance_threshold_squared *= distance_threshold_squared;

      ROS_INFO("%f %d %f", safety_zone_size, sweep_n, matching_threshold);

      dense_pub_ =
          n_.advertise<PointCloud>("autonomous_landing/dense_pointcloud", 5);
      depth_pub_ = it.advertise("autonomous_landing/depth_image", 1);
      refimg_pub_ = it.advertise("autonomous_landing/ref_img", 1);
      vbox_tf_pub = n_.advertise<geometry_msgs::TransformStamped>("cam_transform", 1);

      imgsub_ = it.subscribe("cam", 60,
                             &DenseVerifier::imageCallback, this);
      posesub_ = n_.subscribe("pose", 1,
                              &DenseVerifier::poseCallback, this);
      landingsub_ = n_.subscribe("/autonomous_landing/landing_pointcloud", 2,
                                 &DenseVerifier::landingCallback, this);

      K(0, 0) = k_list[0];
      K(0, 1) = 0;
      K(0, 2) = k_list[2];
      K(1, 0) = 0;
      K(1, 1) = k_list[1];
      K(1, 2) = k_list[3];
      K(2, 0) = 0;
      K(2, 1) = 0;
      K(2, 2) = 1;

      cv::eigen2cv(K, cv_K);

      Eigen::Matrix3d Rbc_mat;
      Rbc_mat << tbc_list[0], tbc_list[1], tbc_list[2],
                tbc_list[4], tbc_list[5], tbc_list[6],
                tbc_list[8], tbc_list[9], tbc_list[10];

      Tbc.setIdentity();
      Tbc.topRightCorner(3, 1) = Eigen::Vector3d(tbc_list[3], tbc_list[7], tbc_list[11]);
      Tbc.topLeftCorner(3, 3) = Rbc_mat;

      image_sweeping_array = std::vector<cv::Mat>(sweep_n);
      pose_array = std::vector<geometry_msgs::Pose>(sweep_n);
      
        
  
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
      image_candidate_vector.push_back(msg);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        
        double distance_squared = 
            std::pow(msg->pose.pose.position.x - current_pose.position.x, 2)
          + std::pow(msg->pose.pose.position.y - current_pose.position.y, 2)
          + std::pow(msg->pose.pose.position.z - current_pose.position.z, 2);
        if(distance_squared > distance_threshold_squared) {
            pose_array[pose_counter] = msg->pose.pose;
            current_pose = msg->pose.pose;

            for(int i = 0; i < image_candidate_vector.size(); i++) {
                if (msg->header.stamp.nsec == image_candidate_vector[i]->header.stamp.nsec 
                    && msg->header.stamp.sec == image_candidate_vector[i]->header.stamp.sec) {

                    ROS_INFO("Found image match for pose, counter %d", pose_counter);
                    
                    cv::Mat undistorted;
                    if(image_is_distorted) {
                      cv::Mat distorted = cv_bridge::toCvShare(image_candidate_vector[i])->image;
                      cv::undistort(distorted, undistorted, cv_K, dist_coeffs);
                    }
                    else {
                      undistorted = cv_bridge::toCvShare(image_candidate_vector[i])->image;
                    }
                    
                    
                    image_sweeping_array[pose_counter] = undistorted;
                    image_candidate_vector.erase(image_candidate_vector.begin(), image_candidate_vector.begin()+i);
                    break;
                }

            }

            pose_counter += 1;

            if (pose_counter == sweep_n) {
              pose_counter = 0;
              ROS_INFO("Planesweep:");
              planeSweep();
              

            }
        };
    }

    void landingCallback(const PointCloud::ConstPtr& msg) {
        
    }

    private:
    ros::NodeHandle n_ = *(new ros::NodeHandle("~"));
    image_transport::ImageTransport it = *(new image_transport::ImageTransport(n_));
    ros::Publisher dense_pub_;
    ros::Publisher vbox_tf_pub;
    tf::TransformBroadcaster cam_pose_pub_;
    image_transport::Publisher depth_pub_;
    image_transport::Publisher refimg_pub_;
    image_transport::Subscriber imgsub_;
    ros::Subscriber posesub_;
    ros::Subscriber landingsub_;

    float safety_zone_size;
    int sweep_n;
    float matching_threshold;
    float distance_threshold_squared;
    float uniqueness_threshold;
    float filter_radius;
    int filter_neighbours;
    Eigen::Matrix3d K;
    cv::Mat cv_K;
    Eigen::Matrix4d Tbc;
    bool image_is_distorted = true;
    std::vector<double> dist_coeffs;

    std::vector<sensor_msgs::ImageConstPtr> image_candidate_vector;
    std::vector<cv::Mat> image_sweeping_array;

    std::vector<geometry_msgs::Pose> pose_array;
    geometry_msgs::Pose current_pose;
    int pose_counter = 0;

    void planeSweep() 
    {
      
      std::vector<PSL::CameraMatrix<double>> cameras(sweep_n);
      std::vector<geometry_msgs::Pose> poses = pose_array;
      
      for (int c = 0; c < sweep_n; c++) {
         
        Eigen::Matrix4d cam_pose = util::poseToRotationAndTranslation(poses[c], Tbc)*Tbc;

        cam_pose.topLeftCorner(3, 3).transposeInPlace();
        cam_pose.topRightCorner(3, 1) = -cam_pose.topLeftCorner(3, 3) * cam_pose.topRightCorner(3, 1);

        Eigen::Matrix3d cam_R = cam_pose.topLeftCorner(3, 3);
        Eigen::Vector3d cam_T = cam_pose.topRightCorner(3, 1);
        
        cameras[c].setKRT(K, cam_R, cam_T);
      };

        double avg_distance = 0; // in order to find a good z-range
        int num_distances = 0;

        for (int i = 0; i < sweep_n - 1; i++) {
          for (int j = i + 1; j < sweep_n; j++) {
            avg_distance += (cameras[i].getC() - cameras[j].getC()).norm();
            num_distances++;
          }
        };

        avg_distance /= num_distances;

        float min_z = 2.5f*avg_distance;
        float max_z = 100.0f*avg_distance;


        PSL::CudaPlaneSweep cps;
        cps.setZRange(min_z, max_z);
        cps.setMatchWindowSize(7, 7);
        cps.setNumPlanes(256);
        cps.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_BEST_K);
        cps.setOcclusionBestK(sweep_n / 2);
        cps.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cps.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
        cps.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cps.enableOutputBestDepth(true);
        cps.enableOutputBestCosts(true);
        cps.enableOuputUniquenessRatio(true);
        cps.enableSubPixel(true);
        

        int ref_id = -1;
        for (int i = 0; i < sweep_n; i++) {
          int id = cps.addImage(image_sweeping_array[i], cameras[i]);

          if (i == sweep_n / 2)
            ref_id = id;
        }

        cps.process(ref_id);
        
        PSL::DepthMap<float, double> depth_map = cps.getBestDepth();
        PSL::Grid<float> costs = cps.getBestCosts();
        PSL::Grid<float> uniqueness_map = cps.getUniquenessRatios();
        
        int width = depth_map.getWidth();
        int height = depth_map.getHeight();

        PointCloud::Ptr dense_pointcloud (new PointCloud());
        dense_pointcloud->is_dense = true;

        Eigen::Matrix3d R = depth_map.getCam().getR();
        Eigen::Vector3d T = depth_map.getCam().getT();

        R.transposeInPlace();
        T = -R*T;
        
        Eigen::Quaterniond eigen_q = Eigen::Quaterniond(R);
        
        tf::Quaternion quat(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
        

        

        tf::Vector3 vec(T[0], T[1], T[2]);
        

        tf::Transform transform;
        transform.setOrigin(vec);
        transform.setRotation(quat);
        
        ros::Time ros_time = ros::Time::now();
        cam_pose_pub_.sendTransform(tf::StampedTransform(transform, ros_time, "world", "cam"));
        
        geometry_msgs::TransformStamped vbox_transform_stamped;
        vbox_transform_stamped.header.frame_id = std::string("world");
        vbox_transform_stamped.header.stamp = ros_time;
        vbox_transform_stamped.transform.translation.x = T[0];
        vbox_transform_stamped.transform.translation.y = T[1];
        vbox_transform_stamped.transform.translation.z = T[2];

        vbox_transform_stamped.transform.rotation.x = eigen_q.x();
        vbox_transform_stamped.transform.rotation.y = eigen_q.y();
        vbox_transform_stamped.transform.rotation.z = eigen_q.z();
        vbox_transform_stamped.transform.rotation.w = eigen_q.w();

        vbox_tf_pub.publish(vbox_transform_stamped);

        
        
        
        


        for (int i = 0; i < width; i++) 
        {
            for (int j = 0; j < height; j++)
            {   Eigen::Vector4d eig_point = depth_map.unproject(i, j);
                
                
                if (eig_point[3] > 0.5 && costs(i, j) < matching_threshold && uniqueness_map(i,j) < uniqueness_threshold)
                {
                    

                    pcl::PointXYZI pcl_point;

                    auto depth = depth_map(i,j);
                
                    pcl_point.x = (i - K(0,2))*depth/K(0,0);
                    pcl_point.y = (j - K(1,2))*depth/K(1,1);
                    pcl_point.z = depth;
                    pcl_point.intensity = costs(i,j)*uniqueness_map(i,j);
                    
                    dense_pointcloud->push_back(pcl_point);

                }
                else {
                  depth_map(i,j) = 0;
                }
            }
        }
        PointCloud::Ptr filtered_pointcloud (new PointCloud());
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> remover;
        remover.setInputCloud(dense_pointcloud);
        remover.setMinNeighborsInRadius(filter_neighbours);
        remover.setRadiusSearch(filter_radius);
        
        remover.filter(*filtered_pointcloud);

        
        sensor_msgs::ImagePtr depth_msg = depth_map_to_msg(depth_map, min_z, max_z);
        depth_pub_.publish(depth_msg);

        cv::Mat refimg = image_sweeping_array[sweep_n / 2];
        std_msgs::Header header;
		    header.stamp = ros_time;
        

        refimg_pub_.publish(cv_bridge::CvImage(header, "mono8", refimg).toImageMsg());
        filtered_pointcloud->header.frame_id = std::string("cam");
        
        pcl_conversions::toPCL(ros_time, filtered_pointcloud->header.stamp);
        
        dense_pub_.publish(filtered_pointcloud);
    };

    void print4(Eigen::Matrix4d pose_matrix) 
    {

      ROS_INFO("%f %f %f %f", pose_matrix(0,0), pose_matrix(0,1), pose_matrix(0, 2), pose_matrix(0,3));
      ROS_INFO("%f %f %f %f", pose_matrix(1,0), pose_matrix(1,1), pose_matrix(1, 2), pose_matrix(1,3));
      ROS_INFO("%f %f %f %f", pose_matrix(2,0), pose_matrix(2,1), pose_matrix(2, 2), pose_matrix(2,3));
      ROS_INFO("%f %f %f %f", pose_matrix(3,0), pose_matrix(3,1), pose_matrix(3, 2), pose_matrix(3,3));
    }

    sensor_msgs::ImagePtr depth_map_to_msg(PSL::DepthMap<float, double> dM, float min_z,
                             float max_z) {
      cv::Mat_<float> depthsMat(dM.getHeight(), dM.getWidth(), dM.getDataPtr());
        cv::Mat_<uint8_t> invDepthsMat(dM.getHeight(), dM.getWidth());
        for (unsigned int y = 0; y < dM.getHeight(); y++)
        {
            for (unsigned int x = 0; x < dM.getWidth(); x++)
            {
                const float depth = depthsMat[y][x];
                if (depth > 0)
                {
                    invDepthsMat[y][x] = 256*(1/depthsMat[y][x]-1/max_z)/(1/min_z - 1/max_z);
                }
                else
                {
                    invDepthsMat[y][x] = 0;
                }
            }
        }

        
        
        std_msgs::Header header;
		    header.stamp = ros::Time::now();
        return cv_bridge::CvImage(header, "mono8", invDepthsMat).toImageMsg();
    }
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