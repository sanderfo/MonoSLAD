#include "ros/ros.h"
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
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
      n_.getParam("cam_frame_topic", cam_frame_topic);
      n_.getParam("world_frame_topic", world_frame_topic);
      n_.getParam("image_scale", scale);
      n_.getParam("safety_zone_size", safety_zone_size);
      n_.getParam("sweep_n", sweep_n);
      n_.getParam("window_size", window_size);
      
      n_.getParam("matching_threshold", matching_threshold);
      n_.getParam("distance_threshold", distance_threshold_squared);
      n_.getParam("uniqueness_threshold", uniqueness_threshold);
      n_.getParam("pointcloud_filter_radius", filter_radius);
      n_.getParam("pointcloud_filter_min_neighbours", filter_neighbours);

      std::string occlusion_string;
      n_.getParam("occlusion_mode", occlusion_string);
      if (occlusion_string == "best_k")
        occlusion_mode = PSL::PLANE_SWEEP_OCCLUSION_BEST_K;
        
      
      else if(occlusion_string == "ref_split")
        occlusion_mode = PSL::PLANE_SWEEP_OCCLUSION_REF_SPLIT;
        
      
      std::vector<double> k_list;
      n_.getParam("camera/intrinsics", k_list);

      std::string dist_model;
      n_.getParam("camera/distortion_model", dist_model);
      if (dist_model == "none") image_is_distorted = false;

      n_.getParam("camera/distortion", dist_coeffs);
      
      
      std::vector<double> tbc_list;
      n_.getParam("T_B_C", tbc_list);


      

      distance_threshold_squared *= distance_threshold_squared;

     
      dense_pub_ =
          n_.advertise<PointCloud>("dense_pointcloud", 5);
      depth_pub_ = it.advertise("depth_image", 1);
      refimg_pub_ = it.advertise("ref_img", 1);
      refimg_pub2_ = it.advertise("ref_img2", 1);
      refimg_pub3_ = it.advertise("ref_img3", 1);
      vbox_tf_pub = n_.advertise<geometry_msgs::TransformStamped>("cam_transform", 1);

      imgsub_ = it.subscribe("cam", 60,
                             &DenseVerifier::imageCallback, this);
      
                      
      tf_buffer = std::make_unique<tf2_ros::Buffer>();
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

      if(scale < 0.9) {
        K(0, 0) = scale*k_list[0];
        K(0, 2) = scale*k_list[2];
        K(1, 1) = scale*k_list[1];
        K(1, 2) = scale*k_list[3];

      }
      else {
        K(0, 0) = k_list[0];
        K(0, 2) = k_list[2];
        K(1, 1) = k_list[1];
        K(1, 2) = k_list[3];

      }

      K(0, 1) = 0;
      K(1, 0) = 0;
      K(2, 0) = 0;
      K(2, 1) = 0;
      K(2, 2) = 1;

      cv::eigen2cv(K, cv_K);

      cam_info = n_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
      
      info.header.frame_id = cam_frame_topic;
      info.header.stamp = ros::Time::now();
      info.height = 240;
      info.width = 376;
      info.K = {K(0, 0), K(0, 1),K(0, 2),K(1, 0),K(1, 1),K(1, 2),K(2, 0),K(2, 1),K(2, 2)};
      info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
      info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      info.P = {K(0, 0), K(0, 1),K(0, 2), 0.0, K(1, 0),K(1, 1),K(1, 2), 0.0, K(2, 0),K(2, 1),K(2, 2), 0.0};
      info.distortion_model = "plumb_bob";
      cam_info.publish(info);

      

      Eigen::Matrix3d Rbc_mat;
      Rbc_mat << tbc_list[0], tbc_list[1], tbc_list[2],
                tbc_list[4], tbc_list[5], tbc_list[6],
                tbc_list[8], tbc_list[9], tbc_list[10];

      Tbc.setIdentity();
      Tbc.topRightCorner(3, 1) = Eigen::Vector3d(tbc_list[3], tbc_list[7], tbc_list[11]);
      Tbc.topLeftCorner(3, 3) = Rbc_mat;

      image_sweeping_array = std::vector<cv::Mat>(sweep_n);
      transform_array = std::vector<geometry_msgs::Transform>(sweep_n);
        
      image_headers = std::vector<std_msgs::Header>(sweep_n);
      
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
      cv::Mat distorted = cv_bridge::toCvShare(msg)->image.clone();
      geometry_msgs::Transform new_transform;
      geometry_msgs::Transform new_pose;
      try {
        new_transform =
            tf_buffer
                ->lookupTransform(cam_frame_topic, world_frame_topic,
                                  msg->header.stamp, ros::Duration(7.0))
                .transform;
        
      }
      catch (tf2::TransformException& e) {
        ROS_INFO("%s",e.what());
        
        return;
      }
      
      
      double distance_squared;
      if (image_counter == 0) 
      {
        current_transform = new_transform;
        distance_squared = 0.001;
      }
      else {
        auto new_pose = tf2::transformToEigen(new_transform).inverse();
        auto old_pose = tf2::transformToEigen(transform_array[0]).inverse();
        
        distance_squared = (new_pose.translation() - old_pose.translation()).squaredNorm();
        distance_squared = distance_squared - std::pow(new_pose.translation().z() - old_pose.translation().z(), 2);

        for(int i = 1; i < image_counter; i++)
        {
          old_pose = tf2::transformToEigen(transform_array[i]).inverse();
         
          double dist_i_squared = (new_pose.translation() - old_pose.translation()).squaredNorm();
          dist_i_squared = dist_i_squared - std::pow(new_pose.translation().z() - old_pose.translation().z(), 2);

          distance_squared = std::min(distance_squared, dist_i_squared);
        }
        
      }
      
      if(image_counter == 0 || distance_squared > distance_threshold_squared) {
        
        transform_array[image_counter] = new_transform;
        
        if(image_counter == sweep_n/2) ref_stamp = msg->header.stamp;

        cv::Mat distorted_scaled;
        if(scale < 0.9) 
        {
          cv::resize(distorted, distorted_scaled, cv::Size(distorted.cols*scale, distorted.rows*scale), cv::INTER_CUBIC);
        }
        else
        {
          distorted_scaled = distorted;
        }
        cv::Mat undistorted;
        if(image_is_distorted) 
        {
                       
          cv::undistort(distorted_scaled, undistorted, cv_K, dist_coeffs);
        }
        else 
        {
          undistorted = distorted_scaled;
        }
        image_sweeping_array[image_counter] = undistorted;
        image_headers[image_counter] = msg->header;
            

        image_counter += 1;

        if (image_counter == sweep_n) {
          
          image_counter = 0;

          

          ROS_INFO("Planesweep:");
          planeSweep();
              
        }
      }
        
    }

    

    private:
    ros::NodeHandle n_ = *(new ros::NodeHandle("~"));
    image_transport::ImageTransport it = *(new image_transport::ImageTransport(n_));
    ros::Publisher dense_pub_;
    ros::Publisher vbox_tf_pub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::string cam_frame_topic = "dense_cam";
    std::string world_frame_topic = "world";
    image_transport::Publisher depth_pub_;
    image_transport::Publisher refimg_pub_;
    image_transport::Publisher refimg_pub2_;
    image_transport::Publisher refimg_pub3_;
    image_transport::Subscriber imgsub_;
    ros::Publisher cam_info;
    sensor_msgs::CameraInfo info;

    float scale;
    float safety_zone_size;
    int sweep_n;
    int window_size = 3;
    enum PSL::PlaneSweepOcclusionMode occlusion_mode = PSL::PLANE_SWEEP_OCCLUSION_NONE;
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

    std::vector<cv::Mat> image_sweeping_array;
    std::vector<std_msgs::Header> image_headers;

    bool use_tf_transforms = false;
    bool poses_supplied = false;
    std::vector<geometry_msgs::Transform> transform_array;
    geometry_msgs::Transform current_transform;
    ros::Time ref_stamp;
    int image_counter = 0;
    

    void planeSweep() 
    {
      
      std::vector<PSL::CameraMatrix<double>> cameras(sweep_n);
      //std::vector<geometry_msgs::Pose> poses = pose_array;
      
      for (int c = 0; c < sweep_n; c++) {
         
        auto cam_transform = tf2::transformToEigen(transform_array[c]);
        
        cameras[c].setKRT(K, cam_transform.rotation(), cam_transform.translation());
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
        cps.setMatchWindowSize(window_size, window_size);
        cps.setNumPlanes(256);
        cps.setOcclusionMode(occlusion_mode);
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
        try {
          cps.process(ref_id);
        }
        catch (...)
        {
          ROS_INFO("process failed, img size %d %d", image_sweeping_array[ref_id].size[0], image_sweeping_array[ref_id].size[1]);
          
          return;
        }
        
        
        PSL::DepthMap<float, double> depth_map = cps.getBestDepth();
        PSL::Grid<float> costs = cps.getBestCosts();
        PSL::Grid<float> uniqueness_map = cps.getUniquenessRatios();
        
        int width = depth_map.getWidth();
        int height = depth_map.getHeight();

        PointCloud::Ptr ordered_pointcloud (new PointCloud());
        
        //ordered_pointcloud->height = height;
        //ordered_pointcloud->width = width;
        ordered_pointcloud->is_dense = true;
        //ordered_pointcloud->resize(height * width);
        
        

        Eigen::Matrix3d R = depth_map.getCam().getR();
        Eigen::Vector3d T = depth_map.getCam().getT();
        //R.transposeInPlace();
        //T = -R*T;
        
        
        Eigen::Quaterniond eigen_q = Eigen::Quaterniond(R);

        
        geometry_msgs::Vector3 vec;
        vec.x = T[0];
        vec.y = T[1];
        vec.z = T[2];
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.transform.translation = vec;

       
        transform_stamped.transform.rotation = tf2::toMsg(eigen_q);
        
        //ros::Time ros_time = ros::Time::now();
        transform_stamped.header.stamp = ref_stamp;
        transform_stamped.header.frame_id = world_frame_topic;
        transform_stamped.child_frame_id = "dense_cam";
        //T_bw_pub_.sendTransform(transform_stamped);

        geometry_msgs::TransformStamped vbox_transform_stamped;
        vbox_transform_stamped.header.frame_id = std::string(world_frame_topic);
        vbox_transform_stamped.header.stamp = ref_stamp;
        vbox_transform_stamped.transform.translation.x = T[0];
        vbox_transform_stamped.transform.translation.y = T[1];
        vbox_transform_stamped.transform.translation.z = T[2];

        vbox_transform_stamped.transform.rotation.x = eigen_q.x();
        vbox_transform_stamped.transform.rotation.y = eigen_q.y();
        vbox_transform_stamped.transform.rotation.z = eigen_q.z();
        vbox_transform_stamped.transform.rotation.w = eigen_q.w();

        vbox_tf_pub.publish(transform_stamped);

        
        
        cv::Mat refimg = image_sweeping_array[sweep_n / 2];
        


        for (int i = 0; i < width; i++) 
        {
            for (int j = 0; j < height; j++)
            {   //Eigen::Vector4d eig_point = depth_map.unproject(i, j);
                
                
                if (depth_map(i,j) > 0.0 && costs(i, j) < matching_threshold && uniqueness_map(i,j) < uniqueness_threshold)
                {
                    

                    pcl::PointXYZI pcl_point;

                    auto depth = depth_map(i,j);
                
                    pcl_point.x = (i - K(0,2))*depth/K(0,0);
                    pcl_point.y = (j - K(1,2))*depth/K(1,1);
                    pcl_point.z = depth;
                    
                    pcl_point.intensity = refimg.at<uint8_t>(j,i);
                    
                    //ordered_pointcloud->at(i, j) = pcl_point;
                    ordered_pointcloud->push_back(pcl_point);
                    
                }
                else {
                  depth_map(i,j) = 0;
                }
            }
        }

        
        PointCloud::Ptr filtered_pointcloud (new PointCloud());
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> remover;
        remover.setInputCloud(ordered_pointcloud);
        remover.setMinNeighborsInRadius(filter_neighbours);
        remover.setRadiusSearch(filter_radius);
        
        
        remover.filter(*filtered_pointcloud);

        
        sensor_msgs::ImagePtr depth_msg = depth_map_to_msg(depth_map, min_z, max_z);
        depth_pub_.publish(depth_msg);

        
        std_msgs::Header header;
		    header.stamp = ref_stamp;
        header.frame_id = cam_frame_topic;

        image_headers[0].frame_id = cam_frame_topic;
        image_headers[1].frame_id = cam_frame_topic;
        image_headers[2].frame_id = cam_frame_topic;

        info.header.stamp = image_headers[0].stamp;
        cam_info.publish(info);
        

        refimg_pub_.publish(cv_bridge::CvImage(image_headers[1], "mono8", refimg).toImageMsg());
        refimg_pub2_.publish(cv_bridge::CvImage(image_headers[0], "mono8", image_sweeping_array[0]).toImageMsg());
        refimg_pub3_.publish(cv_bridge::CvImage(image_headers[2], "mono8", image_sweeping_array[2]).toImageMsg());

        filtered_pointcloud->header.frame_id = cam_frame_topic;

        pcl_conversions::toPCL(ref_stamp, filtered_pointcloud->header.stamp);
        
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