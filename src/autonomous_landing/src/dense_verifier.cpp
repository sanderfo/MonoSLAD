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
        n_.getParam("sweepN", sweepN);
        n_.getParam("matching_threshold", matching_threshold);
        

        ROS_INFO("%f %d %f", safetyZoneSize, sweepN, matching_threshold);

        dense_pub_ =
            n_.advertise<PointCloud>("autonomous_landing/dense_pointcloud", 1);
        depth_pub_ =
            it.advertise("autonomous_landing/depth_image", 1);

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
        if(distanceSquared > 0.1) {
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

            if (pose_counter == sweepN) {
              ROS_INFO("Planesweep:");
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
    ros::Publisher dense_pub_;
    image_transport::Publisher depth_pub_;
    image_transport::Subscriber imgsub_;
    ros::Subscriber posesub_;
    ros::Subscriber landingsub_;

    float safetyZoneSize;
    int sweepN;
    float matching_threshold;
    Eigen::Matrix3d K;

    std::vector<sensor_msgs::ImageConstPtr> image_candidate_vector;
    std::array<cv_bridge::CvImageConstPtr, 10> image_sweeping_array;

    std::array<geometry_msgs::Pose, 10> pose_array;
    geometry_msgs::Pose current_pose;
    int pose_counter = 0;

    void planeSweep() 
    {
        std::vector<PSL::CameraMatrix<double>> cameras(sweepN);
        for (int c = 0; c < sweepN; c++) {
          auto cam_R = util::poseToRotation(pose_array[c]);
          cam_R.transposeInPlace();
          auto cam_T = -cam_R * util::poseToPosition(pose_array[c]);
          cameras[c].setKRT(K, cam_R,
                            cam_T);
        };
        

        double avg_distance = 0; // in order to find a good z-range
        int num_distances = 0;

        Eigen::Vector3d ref_coordinate = cameras[0].getC();
        for (int i = 0; i < sweepN - 1; i++) {
          for (int j = i + 1; j < sweepN; j++) {
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
        cps.setOcclusionBestK(sweepN/2);
        cps.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cps.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
        cps.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cps.enableOutputBestDepth(true);
        cps.enableOutputBestCosts(true);
        cps.enableSubPixel(true);
        

        int ref_id = -1;
        for (int i = 0; i < sweepN; i++) {
          int id = cps.addImage(image_sweeping_array[i]->image, cameras[i]);
         
          if (i == sweepN / 2)
            ref_id = id;
        }
        
        cps.process(ref_id);
        
        PSL::DepthMap<float, double> depth_map = cps.getBestDepth();
        PSL::Grid<float> costs = cps.getBestCosts();
        

        
        
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(0, 0), depth_map.getCam().getCam2Global()(0, 1), depth_map.getCam().getCam2Global()(0, 2), depth_map.getCam().getCam2Global()(0, 3));
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(1, 0), depth_map.getCam().getCam2Global()(1, 1), depth_map.getCam().getCam2Global()(1, 2), depth_map.getCam().getCam2Global()(1, 3));
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(2, 0), depth_map.getCam().getCam2Global()(2, 1), depth_map.getCam().getCam2Global()(2, 2), depth_map.getCam().getCam2Global()(2, 3));
        ROS_INFO("%f %f %f %f", depth_map.getCam().getCam2Global()(3, 0), depth_map.getCam().getCam2Global()(3, 1), depth_map.getCam().getCam2Global()(3, 2), depth_map.getCam().getCam2Global()(3, 3));
        
        int width = depth_map.getWidth();
        int height = depth_map.getHeight();

        PointCloud::Ptr dense_pointcloud (new PointCloud);
        dense_pointcloud->is_dense = true;


        for (int i = 0; i < width; i++) 
        {
            for (int j = 0; j < height; j++)
            {   Eigen::Vector4d eig_point = depth_map.unproject(i, j);
                
                
                if (eig_point[3] > 0 && costs(i, j) < matching_threshold)
                {
                    pcl::PointXYZI pcl_point;
                
                    pcl_point.x = eig_point[0];
                    pcl_point.y = eig_point[1];
                    pcl_point.z = eig_point[2];
                    pcl_point.intensity = image_sweeping_array[ref_id]->image.at<double>(i, j);

                    dense_pointcloud->push_back(pcl_point);

                    

                }
                else {
                  depth_map(i,j) = 0;
                }
            }
        }
        sensor_msgs::ImagePtr depth_msg = depth_map_to_msg(depth_map, min_z, max_z);
        depth_pub_.publish(depth_msg);
        dense_pointcloud->header.frame_id = std::string("world");
        
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