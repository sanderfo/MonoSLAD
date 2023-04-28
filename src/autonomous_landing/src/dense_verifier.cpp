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
#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include <cv_bridge/cv_bridge.h>
#include "psl_base/cameraMatrix.h"
#include <psl_stereo/cudaPlaneSweep.h>
#include "utilities.h"

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudNormal = pcl::PointCloud<pcl::Normal>;

class DenseVerifier
{
    public:
    DenseVerifier() 
    {
      n_.getParam("eval", eval);
      n_.getParam("eval_filename", eval_filename);
      n_.getParam("laplacian_ksize", laplacian_ksize);
      n_.getParam("gaussian_ksize", gaussian_ksize);
      n_.getParam("gaussian_sigma", gaussian_sigma);
      n_.getParam("grad_treshold", grad_treshold);
      n_.getParam("write_vis_images", write_vis_images);
      n_.getParam("write_to_file_debug", write_to_file_debug);
      n_.getParam("cam_frame_topic", cam_frame_topic);
      n_.getParam("world_frame_topic", world_frame_topic);
      n_.getParam("image_scale", scale);
      n_.getParam("safety_zone_size", safety_zone_size);
      n_.getParam("sweep_n", sweep_n);
      n_.getParam("window_size", window_size);
      
      n_.getParam("matching_threshold", matching_threshold);
      n_.getParam("distance_threshold", distance_threshold_squared);
      n_.getParam("uniqueness_threshold", uniqueness_threshold);
      n_.getParam("normal_smoothing_size", normal_smoothing_size);
      n_.getParam("max_depth_change_factor", max_depth_change_factor);

      std::string occlusion_string;
      n_.getParam("occlusion_mode", occlusion_string);
      if (occlusion_string == "best_k"){
        occlusion_mode = PSL::PLANE_SWEEP_OCCLUSION_BEST_K;
        n_.getParam("occlusion_best_k", occlusion_best_k);
        
      }
        
      
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
      n_.getParam("median_filter_kernel_size", kernel_size);
      n_.getParam("voxel_size", voxel_size);

      

      distance_threshold_squared *= distance_threshold_squared;

     
      dense_pub_ =
          n_.advertise<PointCloudRGB>("dense_pointcloud", 5);
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

      //fs.open(eval_filename, cv::FileStorage::WRITE);
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
      
      if((msg->header.stamp - ref_stamp).toSec() > 5.0) {
          image_counter = 0; // discarding old images
        }
      
      
      double distance_squared;
      if (image_counter == 0) 
      {
        current_transform = new_transform;
        distance_squared = 0.001;
      }
      else {
        
        
        auto new_pose = tf2::transformToEigen(new_transform).inverse(Eigen::Isometry);
        auto old_pose = tf2::transformToEigen(transform_array[0]).inverse(Eigen::Isometry);
        
        distance_squared = (new_pose.translation() - old_pose.translation()).squaredNorm();
        //distance_squared = distance_squared - std::pow(new_pose.translation().z() - old_pose.translation().z(), 2);

        for(int i = 1; i < image_counter; i++)
        {
          old_pose = tf2::transformToEigen(transform_array[i]).inverse(Eigen::Isometry);
         
          double dist_i_squared = (new_pose.translation() - old_pose.translation()).squaredNorm();
          //dist_i_squared = dist_i_squared - std::pow(new_pose.translation().z() - old_pose.translation().z(), 2);

          distance_squared = std::min(distance_squared, dist_i_squared);
        }
        
      }
      
      if(image_counter == 0 || distance_squared > distance_threshold_squared) {
        
        transform_array[image_counter] = new_transform;
        
        if(image_counter <= sweep_n/2) ref_stamp = msg->header.stamp;

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

        if (image_counter >= sweep_n) {
          
          image_counter = 0;

          

          //ROS_INFO("Planesweep:");
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
    int kernel_size = 5;
    int sweep_n;
    int window_size = 7;
    enum PSL::PlaneSweepOcclusionMode occlusion_mode = PSL::PLANE_SWEEP_OCCLUSION_NONE;
    int occlusion_best_k = 2;
    float matching_threshold;
    float distance_threshold_squared;
    float uniqueness_threshold;
    int normal_smoothing_size = 3;
    float max_depth_change_factor = 0.2f;
    int filter_neighbours;
    Eigen::Matrix3d K;
    float voxel_size;
    float grad_treshold = 5.0f;
    int laplacian_ksize = 1;
    int gaussian_ksize = 5;
    double gaussian_sigma = 2.0;

    int ref_id = -1;
    
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
    ros::Time ref_stamp = ros::Time::now();
    int image_counter = 0;

    bool eval = false;
    bool write_vis_images = false;
    bool write_to_file_debug = false;
    std::ofstream out_file = std::ofstream("/home/nvidia/AutonomousLanding/test/transforms.txt");
    
    std::string eval_filename;
    cv::FileStorage fs;
    void planeSweep() 
    {
      auto t0 = ros::Time::now();
      std::vector<PSL::CameraMatrix<double>> cameras(sweep_n);
      //std::vector<geometry_msgs::Pose> poses = pose_array;
      Eigen::Vector3d optical_direction = tf2::transformToEigen(transform_array[0]).rotation().row(2);
      int ref_index = -1;
      double max_distance_in_optical_direction = std::numeric_limits<double>::lowest();
      for (int c = 0; c < sweep_n; c++) {
         
        auto cam_transform = tf2::transformToEigen(transform_array[c]);
        
        cameras[c].setKRT(K, cam_transform.rotation(), cam_transform.translation());
        Eigen::Vector3d cam_pose_translation = cam_transform.inverse(Eigen::Isometry).translation();
        double distance_in_optical_direction = cam_pose_translation.dot(optical_direction);
        if(distance_in_optical_direction > max_distance_in_optical_direction)
        {
          ref_index = c;
          max_distance_in_optical_direction = distance_in_optical_direction;
        }
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

        Eigen::MatrixXd A((sweep_n-1)*5, 4);
        
        int normals_counter = 0;
        auto ref_transform = tf2::transformToEigen(transform_array[ref_index]);
        for(int i = 0; i < sweep_n; i++){
          if(i != ref_index){
            //Eigen::MatrixXd N_a(5, 3);
            double norm_factor_x = 1/std::sqrt(K(0,0)*K(0,0) + K(0,2)*K(0,2));
            double norm_factor_y = 1/std::sqrt(K(1,1)*K(1,1) + K(1,2)*K(1,2));
            
            auto relative_transform = (tf2::transformToEigen(transform_array[i]) * ref_transform.inverse(Eigen::Isometry)).matrix();
            relative_transform.transposeInPlace();
            //auto relative_orientation = relative_transform.rotation();
            //auto relative_translation = relative_transform.translation();
            /*N_a << 0, 0, 1,
                   K(0,0), 0, K(0,2),
                   -K(0,0), 0, K(0,2),
                   0, K(1,1), K(1,2),
                   0, -K(1,1), K(1,2);
            */
            Eigen::MatrixXd N_a(5, 4);
            N_a << 0, 0, 1, -1,
                   K(0,0)*norm_factor_x, 0, K(0,2)*norm_factor_x, 0,
                   -K(0,0)*norm_factor_x, 0, K(0,2)*norm_factor_x, 0,
                   0, K(1,1)*norm_factor_y, K(1,2)*norm_factor_y, 0,
                   0, -K(1,1)*norm_factor_y, K(1,2)*norm_factor_y, 0;
            N_a.transposeInPlace();
            N_a = relative_transform * N_a;
            
            N_a.transposeInPlace();
      
           
            A.block(normals_counter*5, 0, 5, 4) = N_a;
            //N_test.topLeftCorner(1, 3).normalize();
            
            
            normals_counter++;
            
          }
        }
        
        PSL::CudaPlaneSweep cps;
        cps.setZRange(min_z, max_z);
        cps.setMatchWindowSize(window_size, window_size);
        cps.setNumPlanes(256);
        cps.setOcclusionMode(occlusion_mode);
        cps.setOcclusionBestK(occlusion_best_k);
        cps.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cps.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
        cps.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cps.enableOutputBestDepth(true);
        cps.enableOutputBestCosts(true);
        cps.enableOuputUniquenessRatio(true);
        cps.enableSubPixel(true);
        
        

        
        std::vector<int> image_ids(sweep_n);
        
        
        for (int i = 0; i < sweep_n; i++) {
          int id = cps.addImage(image_sweeping_array[i], cameras[i]);
          image_ids[i] = id;
          
          if (i == ref_index)
            ref_id = id;
        }
        try {
          auto pslt0 = ros::Time::now();
          cps.process(ref_id);
          //ROS_INFO("PSL Time %f", (ros::Time::now() - pslt0).toSec());
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

        
        
        cv::Mat_<float> depth_cv(height, width, depth_map.getDataPtr());
        ROS_INFO("using mask:");

        // culling:
        cv::Mat depth_cv2(depth_cv);

        depth_cv2.forEach<float>([this, A](float &pixel, const int position[]) -> void {
          double x = (position[1] - this->K(0,2))*pixel/this->K(0,0);
          double y = (position[0] - this->K(1,2))*pixel/this->K(1,1);
          double z = pixel;
          
          Eigen::Vector4d point_eigen(x, y, z, 1);

          Eigen::VectorXd d = A*point_eigen;
          
          for(int i = 0; i < d.size(); i++){
            if(d(i) < 0) {
              pixel = 0.0f;
              break;
            }
          }
        });
        //depth_cv.setTo(0.0, bb);
        //depth_cv(cv::Range(0, height), cv::Range(0, bb[0])) = 0.0;
        //depth_cv(cv::Range(0, height), cv::Range(bb[1], width)) = 0.0;
        
        
        //depth_cv(cv::Range(0, bb[2]), cv::Range(0, width)) = 0.0;
        //depth_cv(cv::Range(bb[3], height), cv::Range(0, width)) = 0.0;
        cv::Mat depth_cv_filtered;
        
        cv::Mat_<float> costs_cv(height, width, costs.getDataPtr());
        cv::Mat cost_mask = costs_cv > matching_threshold;
        
        cv::Mat_<float> unique_cv(height, width, uniqueness_map.getDataPtr());
        cv::Mat unique_mask = unique_cv > uniqueness_threshold;

        cv::Mat mask = cost_mask | unique_mask;
        if(write_vis_images){
          std::string filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[ref_index].stamp.toNSec());
          cv::imwrite(filename + "ref" + std::to_string(ref_index)+ "depth.png",  depth_cv_to_inv(depth_cv, min_z, max_z));
          for(int i = 0; i < sweep_n; i++){
              filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[ref_index].stamp.toNSec());
              
              cv::imwrite(filename + "ref" + std::to_string(i) + ".png",  image_sweeping_array[i]);
    
              
            }
            
        }
        cv::Mat depth_median_blurred;
        
        
        cv::Mat grads;
        cv::Mat blurred_grads;
        

        //cv::GaussianBlur(depth_cv, depth_blurred, cv::Size(gaussian_ksize, gaussian_ksize), gaussian_sigma);
        //cv::medianBlur(depth_cv, depth_median_blurred, 5);
        //cv::GaussianBlur(depth_median_blurred, depth_blurred, cv::Size(gaussian_ksize, gaussian_ksize), gaussian_sigma);
        //cv::Laplacian(depth_blurred, edges, -1, laplacian_ksize); 
        
        cv::Laplacian(depth_cv, grads, -1, laplacian_ksize);
        cv::GaussianBlur(cv::abs(grads), blurred_grads, cv::Size(gaussian_ksize, gaussian_ksize), gaussian_sigma);
        cv::Mat blurred_grads_corrected = cv::max(blurred_grads, cv::abs(grads));
        cv::Mat grad_mask = blurred_grads_corrected > grad_treshold;
        //cv::Mat edges_mask = cv::abs(edges) < grad_treshold / 2;
        //cv::Mat total_mask = grad_mask & edges_mask;
        
        
        /*if(write_vis_images){
          std::string filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[ref_index].stamp.toNSec());
          cv::imwrite(filename + "ref" + std::to_string(ref_index)+ "depth_blurred.png",  depth_cv_to_inv(depth_median_blurred, min_z, max_z));
        }*/
        //depth_cv = depth_median_blurred;
        int border_size = gaussian_ksize/2;
        depth_cv.setTo(0.0, grad_mask);
        depth_cv(cv::Range(0, border_size), cv::Range(0, width)) = 0.0;
        depth_cv(cv::Range(height-border_size, height), cv::Range(0, width)) = 0.0;
        depth_cv(cv::Range(0, height), cv::Range(0, border_size)) = 0.0;
        depth_cv(cv::Range(0, height), cv::Range(width-border_size, width)) = 0.0;
        
        
        //width -= gaussian_ksize-1;
        //height -= gaussian_ksize-1;
        if(write_vis_images){
          std::string filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[ref_index].stamp.toNSec());
          
          cv::imwrite(filename + "ref" + std::to_string(ref_index)+ "depth_laplace.png",  depth_cv_to_inv(depth_cv, min_z, max_z));
          
        }
        


        
       
        cv::medianBlur(depth_cv, depth_cv_filtered, kernel_size);

        if(write_vis_images){
          std::string filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[ref_index].stamp.toNSec());
          cv::imwrite(filename + "ref" + std::to_string(ref_index)+"depth_median_filtered.png",  depth_cv_to_inv(depth_cv_filtered, min_z, max_z));
          
        }
        if(eval){
          depth_cv_to_tiff(depth_cv_filtered, ref_index, "final");
        }
        
        //std::vector<cv::Mat> arr {depth_cv_filtered, costs_cv, unique_cv};

        //cv::Mat merged;
        //cv::merge(arr, merged);
        PointCloudXYZ::Ptr ordered_pointcloud (new PointCloudXYZ());
        
        ordered_pointcloud->height = height;
        ordered_pointcloud->width = width;
        ordered_pointcloud->is_dense = false;
        ordered_pointcloud->resize(height * width);

        
        typedef float Pixel;

        
        depth_cv_filtered.forEach<Pixel>([this, ordered_pointcloud](Pixel &pixel, const int position[]) -> void {
          pcl::PointXYZ pcl_point;
          if (pixel > 0.0 && pixel < 30.0 /*&& pixel.y < matching_threshold && pixel.z < uniqueness_threshold*/)
                {

                      pcl_point.x = (position[1] - this->K(0,2))*pixel/this->K(0,0);
                      pcl_point.y = (position[0] - this->K(1,2))*pixel/this->K(1,1);
                      pcl_point.z = pixel;

                      
                      //pcl_point.intensity  = this->image_sweeping_array[this->ref_id].at<uint8_t>(position[0],position[1]);
                      
                      ordered_pointcloud->at(position[1], position[0]) = pcl_point;
                      
                      //ordered_pointcloud->erase(iterator position)
                      //ordered_pointcloud->push_back(pcl_point);
                    
                    //}
                    
                }
              else {
                pcl_point.z = NAN;
                ordered_pointcloud->at(position[1], position[0]) = pcl_point;
                pixel = 0.0;
              }
        });
        
        
        //integral images
        Eigen::Vector3d z_dir = tf2::transformToEigen(transform_array[ref_index]).rotation().col(2);
        
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        PointCloudRGB::Ptr colored_pointcloud (new PointCloudRGB);

        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor(max_depth_change_factor);
        ne.setNormalSmoothingSize((float)normal_smoothing_size);
        ne.setDepthDependentSmoothing(false);
        ne.setInputCloud(ordered_pointcloud);

        
        int step_size_x = width/5; //initially assuming far away points
        int step_size_y = height/5;
        
        int border = normal_smoothing_size;

        
        
        for(int i = 0; i < width-step_size_x; i += step_size_x){
          
          for(int j = 0; j < height-step_size_y; j += step_size_y) {
            //ROS_INFO("i j stepx stepy %d %d %d %d", i, j, step_size_x, step_size_y);
            float depth_max = 0.0f;
            
            
            int rect_x;
            int rect_y;
            
            
            
            
            for(int k = 0; k < step_size_x; k++) 
            {
              for(int l = 0; l < step_size_y; l++)
              {
                
                if(!std::isnan(ordered_pointcloud->at(i+k, j+l).z))
                {
                  pcl::PointXYZ pointxyz = ordered_pointcloud->at(i+k, j+l);
                  depth_max = std::max<double>(pointxyz.z, depth_max);
                    //centroid.x += pointxyz.x;
                    //centroid.y += pointxyz.y;
                    //centroid.z += pointxyz.z;
                    //depth_counts++;
                  }
                
                }
            }
            if(depth_max < 1.0) continue;
            float dist_x = depth_max/K(0,0);
            float dist_y = depth_max/K(1,1);
            rect_x = std::max((int)std::ceil(voxel_size/dist_x), 3);
            rect_y = std::max((int)std::ceil(voxel_size/dist_y), 3);

            
            
            for(int k = 0; k < step_size_x-rect_x; k += rect_x) 
            {
              for(int l = 0; l < step_size_y-rect_y; l += rect_y)
              {
                if(i+k < normal_smoothing_size*rect_x/2 || i+k >= width - normal_smoothing_size*rect_x/2) continue;
                if(j+l < normal_smoothing_size*rect_y/2 || j+l >= height - normal_smoothing_size*rect_y/2) continue;

                pcl::PointXYZRGB centroid;
                
                int depth_counts = 0;
                for(int i2 = i+k; i2 < i+k + rect_x; i2++){
                  for(int j2 = j+l; j2 < j+l + rect_y; j2++){
                    
                    pcl::PointXYZ pointxyz = ordered_pointcloud->at(i2, j2);
                    if(!std::isnan(pointxyz.z))
                    {
                      centroid.x += pointxyz.x;
                      centroid.y += pointxyz.y;
                      centroid.z += pointxyz.z;
                      depth_counts++;
                    }
                    
                  }
                }
                if(depth_counts < 1) continue;
                centroid.x /= depth_counts;
                centroid.y /= depth_counts;
                centroid.z /= depth_counts;
                ne.setRectSize(normal_smoothing_size*rect_x, normal_smoothing_size*rect_y);
                pcl::Normal normal;
                unsigned index_x = i+k + rect_x/2;
                unsigned index_y = j+l + rect_y/2;
                unsigned point_index = index_y * width + index_x;
                ne.computePointNormal(index_x, index_y, point_index, normal);
                
                pcl::PointXYZRGB pt;

                pt.x = centroid.x;
                pt.y = centroid.y;
                pt.z = centroid.z;
                // undo this for normals
                pt.r = 255*lerp(0.0, 0.05, normal.curvature);
                Eigen::Vector3d eig_normal(normal.normal[0], normal.normal[1], normal.normal[2]);
                pt.g = 255*lerp(0.85, 1.0, z_dir.dot(eig_normal));
                pt.b = 0;
                /*
                pt.r = image_sweeping_array[ref_index].at<uint8_t>(index_y,index_x);
                pt.g = image_sweeping_array[ref_index].at<uint8_t>(index_y,index_x);
                pt.b = image_sweeping_array[ref_index].at<uint8_t>(index_y,index_x);
                */
                colored_pointcloud->push_back(pt);
              }
            }

            

                
                

              
            
            
          }

        }
        
        cv::Mat depth_mat = depth_map_to_msg(depth_map, min_z, max_z);
        sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(image_headers[sweep_n/2], "mono8", depth_mat).toImageMsg();
        
        depth_pub_.publish(depth_msg);

        
        std_msgs::Header header;
		    header.stamp = ref_stamp;
        header.frame_id = cam_frame_topic;

        image_headers[0].frame_id = cam_frame_topic;
        image_headers[1].frame_id = cam_frame_topic;
        //image_headers[2].frame_id = cam_frame_topic;

        info.header.stamp = image_headers[0].stamp;
        cam_info.publish(info);
        

        refimg_pub_.publish(cv_bridge::CvImage(image_headers[1], "mono8", refimg).toImageMsg());
        refimg_pub2_.publish(cv_bridge::CvImage(image_headers[0], "mono8", image_sweeping_array[0]).toImageMsg());
        //refimg_pub3_.publish(cv_bridge::CvImage(image_headers[2], "mono8", image_sweeping_array[2]).toImageMsg());
        
        if(write_to_file_debug)
        {
        std::string filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[sweep_n/2].stamp.toNSec());
        cv::imwrite(filename + "depth.png",  depth_mat);
        if(!out_file.is_open()) out_file.open("/home/nvidia/AutonomousLanding/test/transforms.txt", std::ios_base::app);
        for(int i = 0; i < sweep_n; i++){
          filename = "/home/nvidia/AutonomousLanding/test/" + std::to_string(image_headers[i].stamp.toNSec());
          cv::imwrite(filename + ".png",  image_sweeping_array[i]);

          auto pose_eigen = tf2::transformToEigen(transform_array[i]).inverse(Eigen::Isometry);
          auto eigen_rotation_q = Eigen::Quaterniond(pose_eigen.rotation());
          std::string poses_string = "";
          poses_string += filename + "\n";
          poses_string += std::to_string(eigen_rotation_q.w()) + " " 
                          + std::to_string(eigen_rotation_q.x()) + " "
                          + std::to_string(eigen_rotation_q.y()) + " "
                          + std::to_string(eigen_rotation_q.z()) + " "
                          + std::to_string(pose_eigen.translation().x()) + " "
                          + std::to_string(pose_eigen.translation().y()) + " "
                          + std::to_string(pose_eigen.translation().z()) + "\n";
                          

          out_file << poses_string;
        }
        out_file.close();
        }
        

        colored_pointcloud->header.frame_id = cam_frame_topic;

        pcl_conversions::toPCL(ref_stamp, colored_pointcloud->header.stamp);
        
        
        dense_pub_.publish(colored_pointcloud);
        //ROS_INFO("total time %f", (ros::Time::now()- t0).toSec());
    }
    auto lerp(double edge0, double edge1, double x) -> double {
      if(x < edge0) return 0.0;
      if(x > edge1) return 1.0;

      x = (x - edge0) / (edge1 - edge0);
      return x;

    }

    auto smoothstep(double edge0, double edge1, double x) -> double {
      if(x < edge0) return 0.0;
      if(x > edge1) return 1.0;

      x = lerp(edge0, edge1, x);
      return x*x*(3-2*x);

    }

    auto loglerp(double edge0, double edge1, double x) -> double {
      if(x < edge0) return 0.0;
      if(x > edge1) return 1.0;

      x = std::pow(edge1, x)*std::pow(edge0, 1-x);
      return x;

    }

    void print4(Eigen::Matrix4d pose_matrix) 
    {

      ROS_INFO("%f %f %f %f", pose_matrix(0,0), pose_matrix(0,1), pose_matrix(0, 2), pose_matrix(0,3));
      ROS_INFO("%f %f %f %f", pose_matrix(1,0), pose_matrix(1,1), pose_matrix(1, 2), pose_matrix(1,3));
      ROS_INFO("%f %f %f %f", pose_matrix(2,0), pose_matrix(2,1), pose_matrix(2, 2), pose_matrix(2,3));
      ROS_INFO("%f %f %f %f", pose_matrix(3,0), pose_matrix(3,1), pose_matrix(3, 2), pose_matrix(3,3));
    }

    cv::Mat depth_map_to_msg(PSL::DepthMap<float, double> dM, float min_z,
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
        return invDepthsMat;
    }
    cv::Mat depth_cv_to_inv(cv::Mat_<float> depthsMat, float min_z,
                             float max_z) {
      //cv::Mat_<float> depthsMat(dM.getHeight(), dM.getWidth(), dM.getDataPtr());
        cv::Mat_<uint8_t> invDepthsMat(depthsMat.size[0], depthsMat.size[1]);
        for (unsigned int y = 0; y < depthsMat.size[0]; y++)
        {
            for (unsigned int x = 0; x < depthsMat.size[1]; x++)
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
        return invDepthsMat;
    }

    void depth_cv_to_tiff(cv::Mat_<float> depthsMat, int ref_index, std::string tag) {
      //cv::Mat_<float> depthsMat(dM.getHeight(), dM.getWidth(), dM.getDataPtr());
      
      std::string element_name = tag + std::to_string(image_headers[ref_index].stamp.toNSec());
      //cv::Mat_<double> depths_double;
      //depths_double = depthsMat;
      //ROS_INFO("d %f", depthsMat[100][100]);
      //ROS_INFO("dd %lf", depths_double[100][100]);

      //bool ok = cv::imwrite(filename + tag + ".tiff", depths_double);

      //ROS_INFO("write ok %u", ok);
      fs.open(eval_filename, cv::FileStorage::APPEND);
      fs.write(element_name, depthsMat);
      fs.release();
      return;
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