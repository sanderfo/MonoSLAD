#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
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
      n_.getParam("safety_zone_size", safety_zone_size);
      n_.getParam("sweep_n", sweep_n);
      n_.getParam("matching_threshold", matching_threshold);
      n_.getParam("distance_threshold", distance_threshold_squared);
      n_.getParam("uniqueness_threshold", uniqueness_threshold);
      

      distance_threshold_squared *= distance_threshold_squared;

      ROS_INFO("%f %d %f", safety_zone_size, sweep_n, matching_threshold);

      dense_pub_ =
          n_.advertise<PointCloud>("autonomous_landing/dense_pointcloud", 1);
      depth_pub_ = it.advertise("autonomous_landing/depth_image", 1);
      refimg_pub_ = it.advertise("autonomous_landing/ref_img", 1);

      cam_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>(
          "autonomous_landing/cam_pose", 1);

      imgsub_ = it.subscribe("/cam0/image_raw", 1,
                             &DenseVerifier::imageCallback, this);
      posesub_ = n_.subscribe("/svo/backend_pose_imu", 2,
                              &DenseVerifier::poseCallback, this);
      landingsub_ = n_.subscribe("/autonomous_landing/landing_pointcloud", 2,
                                 &DenseVerifier::landingCallback, this);

      K(0, 0) = 458.654;
      K(0, 1) = 0;
      K(0, 2) = 367.215;
      K(1, 0) = 0;
      K(1, 1) = 457.296;
      K(1, 2) = 248.375;
      K(2, 0) = 0;
      K(2, 1) = 0;
      K(2, 2) = 1;

      Eigen::Matrix3d Rbc_mat;
      Rbc_mat << 0.0148655429818, -0.999880929698, 0.00414029679422,
                0.999557249008, 0.0149672133247, 0.025715529948,
                -0.0257744366974, 0.00375618835797, 0.999660727178;

      Tbc.topRightCorner(3, 1) = Eigen::Vector3d(-0.0216401454975, -0.064676986768, 0.00981073058949);
      Tbc.topLeftCorner(3, 3) = Rbc_mat;

      image_sweeping_array = std::vector<cv_bridge::CvImageConstPtr>(sweep_n);
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
                    
                    image_sweeping_array[pose_counter] = cv_bridge::toCvShare(image_candidate_vector[i]);
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
    ros::Publisher cam_pose_pub_;
    image_transport::Publisher depth_pub_;
    image_transport::Publisher refimg_pub_;
    tf::TransformBroadcaster br;
    image_transport::Subscriber imgsub_;
    ros::Subscriber posesub_;
    ros::Subscriber landingsub_;

    float safety_zone_size;
    int sweep_n;
    float matching_threshold;
    float distance_threshold_squared;
    float uniqueness_threshold;
    Eigen::Matrix3d K;
    Eigen::Matrix4d Tbc;

    std::vector<sensor_msgs::ImageConstPtr> image_candidate_vector;
    std::vector<cv_bridge::CvImageConstPtr> image_sweeping_array;

    std::vector<geometry_msgs::Pose> pose_array;
    geometry_msgs::Pose current_pose;
    int pose_counter = 0;

    void planeSweep() 
    {
      std::vector<PSL::CameraMatrix<double>> cameras(sweep_n);
      std::vector<geometry_msgs::Pose> poses = pose_array;
      std::vector<cv_bridge::CvImageConstPtr> images = image_sweeping_array;
      for (int c = 0; c < sweep_n; c++) {
        Eigen::Matrix3d cam_R;
        Eigen::Vector3d cam_T;
        Eigen::Matrix4d cam_pose;
        cam_pose = util::poseToRotationAndTranslation(poses[c], Tbc);
        cam_pose.topLeftCorner(3, 3).transposeInPlace();
        cam_pose.topRightCorner(3, 1) = -cam_pose.topLeftCorner(3, 3) * cam_pose.topRightCorner(3, 1);
        cam_pose = Tbc * cam_pose;
        cam_R = cam_pose.topLeftCorner(3, 3);
        cam_T = cam_pose.topRightCorner(3, 1);
        // cam_R.transposeInPlace();
        // auto cam_T = -cam_R * util::poseToPosition(pose_array[c]);
        //auto cam_T = util::poseToPosition(poses[c]);
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
          int id = cps.addImage(images[i]->image, cameras[i]);

          if (i == sweep_n / 2)
            ref_id = id;
        }

        cps.process(ref_id);
        
        PSL::DepthMap<float, double> depth_map = cps.getBestDepth();
        PSL::Grid<float> costs = cps.getBestCosts();
        PSL::Grid<float> uniqueness_map = cps.getUniquenessRatios();
        

        
        
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(0, 0), depth_map.getCam().getCam2Global()(0, 1), depth_map.getCam().getCam2Global()(0, 2), depth_map.getCam().getCam2Global()(0, 3));
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(1, 0), depth_map.getCam().getCam2Global()(1, 1), depth_map.getCam().getCam2Global()(1, 2), depth_map.getCam().getCam2Global()(1, 3));
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(2, 0), depth_map.getCam().getCam2Global()(2, 1), depth_map.getCam().getCam2Global()(2, 2), depth_map.getCam().getCam2Global()(2, 3));
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(3, 0), depth_map.getCam().getCam2Global()(3, 1), depth_map.getCam().getCam2Global()(3, 2), depth_map.getCam().getCam2Global()(3, 3));
        
        int width = depth_map.getWidth();
        int height = depth_map.getHeight();

        PointCloud::Ptr dense_pointcloud (new PointCloud);
        dense_pointcloud->is_dense = true;

        tf::Transform transform;

        auto R = depth_map.getCam().getR();
        auto T = depth_map.getCam().getT();

        tf::Vector3 t(T[0],T[1],T[2]);

        Eigen::Quaterniond eigen_q(R);
        tf::Quaternion q(eigen_q.x(),eigen_q.y(),eigen_q.z(),eigen_q.w());
        

        transform.setOrigin(t);
        transform.setRotation(q);
        
        Eigen::Affine3d pose_cam;
        pose_cam.setIdentity();
        pose_cam.pretranslate(T);
        pose_cam.rotate(R);
        
        geometry_msgs::PoseStamped pose_msg;
        
        tf::poseEigenToMsg(pose_cam, pose_msg.pose);
        pose_msg.header.frame_id = std::string("world");
        cam_pose_pub_.publish(pose_msg);


        for (int i = 0; i < width; i++) 
        {
            for (int j = 0; j < height; j++)
            {   Eigen::Vector4d eig_point = depth_map.unproject(i, j);
                
                
                if (eig_point[3] > 0.5 && costs(i, j) < matching_threshold && uniqueness_map(i,j) > uniqueness_threshold)
                {
                    

                    //eig_point[0] = (i - K(0,2))*depth_map(i,j)/K(0,0);
                    //eig_point[1] = (j - K(1,2))*depth_map(i,j)/K(1,1);
                    //eig_point[2] = depth_map(i,j);

                    //eig_point = pose_cam*eig_point;

                    //eig_point = depth_map.getCam().getCam2Global().inverse() * eig_point;
                    pcl::PointXYZI pcl_point;
                
                    pcl_point.x = eig_point[0];
                    pcl_point.y = eig_point[1];
                    pcl_point.z = eig_point[2];
                    pcl_point.intensity = images[ref_id]->image.at<double>(i, j);

                    dense_pointcloud->push_back(pcl_point);

                    

                }
                else {
                  depth_map(i,j) = 0;
                }
            }
        }
        //br.sendTransform(tf::StampedTransform(Tbc, ros::Time::now(), "dense_body", "dense_cam"));
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dense_body"));

        sensor_msgs::ImagePtr depth_msg = depth_map_to_msg(depth_map, min_z, max_z);
        depth_pub_.publish(depth_msg);

        auto refimg = images[sweep_n / 2]->image;
        std_msgs::Header header;
		    header.stamp = ros::Time::now();
        

        refimg_pub_.publish(cv_bridge::CvImage(header, "mono8", refimg).toImageMsg());
        dense_pointcloud->header.frame_id = std::string("world");
        pcl_conversions::toPCL(ros::Time::now(), dense_pointcloud->header.stamp);
        
        dense_pub_.publish(dense_pointcloud);
        
       

        
    };

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