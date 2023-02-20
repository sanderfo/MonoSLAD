#include <unordered_map>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/features/linear_least_squares_normal.h>
#include <string>
#include <opencv2/core/mat.hpp>
#include "opencv2/imgproc.hpp"

using PointCloudIntensity = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudNormal = pcl::PointCloud<pcl::PointXYZRGB>;

class NormalEstimationOMP : public pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> {
  public:
  auto get_centroid() -> Eigen::Vector4f {
    return xyz_centroid_;
  }
};
class DenseGrid {
  private:
  std::vector<std::vector<PointCloudIntensity>> clouds_;

};
class SLAD
{
  public:
  SLAD() {
    n_.getParam("boxSize", boxSize);
    n_.getParam("minPoints", minPoints);
    n_.getParam("flatness_requirement", flatness_requirement);
    n_.getParam("voxel_size", voxel_size);

    ROS_INFO("box, min, flat: %f, %d, %f", boxSize, minPoints, flatness_requirement);
    
    pub_ = n_.advertise<PointCloudNormal>("landing_pointcloud", 2);

    sub_ = n_.subscribe("input_pointcloud", 2, &SLAD::pointCloudCallback, this);
    tf_buffer = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    global_landing_map->header.frame_id = "world";
  }

  // %Tag(CALLBACK)%
  void pointCloudCallback(const PointCloudIntensity::ConstPtr& msg)
  {
    
    PointCloudIntensity::Ptr temp_pc (new PointCloudIntensity);
    
    //pcl::io::savePCDFileASCII("test_pcd.pcd", *temp_pc);
    current_transform = tf_buffer->lookupTransform("world", "cam", ros::Time(0), ros::Duration(3.0)).transform;
    
    pcl_ros::transformPointCloud(*msg, *temp_pc, current_transform);
    *local_dense_map += *temp_pc;
    
    PointCloudIntensity::Ptr dense_filtered (new PointCloudIntensity);
    pcl::VoxelGrid<pcl::PointXYZI> vgrid;
    vgrid.setInputCloud(local_dense_map);
    vgrid.setLeafSize(voxel_size, voxel_size, voxel_size);
    vgrid.filter(*dense_filtered);
    
    *local_dense_map = *dense_filtered;


  
    PointCloudNormal::Ptr landing_pc = splitPointCloud(local_dense_map);
    landing_pc->header.stamp = msg->header.stamp;
    landing_pc->header.frame_id = "world";

    
    
    
    //*global_landing_map = *landing_pc;
 
    pub_.publish(landing_pc);
  

  
  }
  // %EndTag(CALLBACK)%
  auto calculateFlatness(PointCloudIntensity::Ptr cloud) -> std::tuple<Eigen::Vector4f, Eigen::Vector3d, float>
  {
    
  
    float nx; float ny; float nz; float curvature;
    std::vector<int> indices(cloud->size());
    std::iota(indices.begin(), indices.end(), 0);
  
    NormalEstimationOMP ne;
    ne.setInputCloud(cloud);

    Eigen::Isometry3d transform = tf2::transformToEigen(current_transform);
    Eigen::Matrix3d rot = transform.rotation();
    Eigen::Vector3d t = transform.translation();
    rot.transposeInPlace();

    Eigen::Vector3d inverse_translation = -rot * t;
    // husk invers for pose
    ne.setViewPoint(inverse_translation[0], inverse_translation[1], inverse_translation[2]);
    ne.computePointNormal(*cloud, indices, nx, ny, nz, curvature);
    auto centroid = ne.get_centroid();
    Eigen::Vector3d normal {nx, ny, nz};

  
    //float fitness = std::abs(nz)*(1-3*curvature); // higher is better, max is 1, min is 0
    return {centroid, normal, curvature};
  }

  auto splitPointCloud(PointCloudIntensity::Ptr pointCloud) -> PointCloudNormal::Ptr
  {
    
    auto cloud_map = gridify(*pointCloud);
    
    
    PointCloudNormal::Ptr landing_pc (new PointCloudNormal());

    //fitness_img.forEach<float>([this, pointCloud, min_point, max_point, &coordinate_image](float& fitness, const int pos[]) -> void {
    for(auto kv : cloud_map)
    {
      if(kv.second->size() > minPoints) {
      
    
      
        
        std::tuple<Eigen::Vector4f, Eigen::Vector3d, float> flatness_and_curvature = calculateFlatness(kv.second);
       
        Eigen::Vector4f centroid = std::get<0>(flatness_and_curvature);
        
        Eigen::Vector3d normal = std::get<1>(flatness_and_curvature);
        
        
        //fitness_img.at<float>(i, j) = std::get<0>(flatness_and_curvature);
        
        //pcl::compute3DCentroid(*kv.second, centroid);
        
        
        //coordinate_image.at<Eigen::Vector3f>(i, j) = {centroid[0], centroid[1], centroid[2]};
        pcl::PointXYZRGB pt;
        pt.x = centroid[0];
        pt.y = centroid[1];
        pt.z = centroid[2];
        pt.r = 765*std::get<2>(flatness_and_curvature);
        pt.g = 255*smoothstep(0.6, 1.0, normal[2]);
        pt.b = 0;
      
        landing_pc->push_back(pt);
      }
      

    }  
     
    //});

    //cv::GaussianBlur(fitness_img, filtered_fitness, cv::Size(5, 5), 1.5);

    //cv::Mat filtered_min(grid_width, grid_height, CV_32F);

    //cv::min(fitness_img, filtered_fitness, filtered_min); // so that fitness can only get worse from neighbours

    //std::cout << fitness_img << "\n";

    //fitness_img.forEach<float>([this, &coordinate_image, landing_pc](float& fitness, const int pos[]) -> void {
    //for(int i = 0; i< grid_width; i++) {
    //  float * flat_ptr = fitness_img.ptr<float>(i);
    //  Eigen::Vector3f * normal_ptr = coordinate_image.ptr<Eigen::Vector3f>(i);
    //  for(int j = 0; j < grid_height; j++) {
    //    
    //    Eigen::Vector3f coords = normal_ptr[j];
    //    pcl::PointXYZRGB pt;
    //    pt.x = coords[0];
    //    pt.y = coords[1];
    //    pt.z = coords[2];
    //    pt.r = 255*flat_ptr[j];
    //    landing_pc->push_back(pt);
    //  
    //  }
    //}
    

    //}
    //});

    return landing_pc;
  }

  auto gridify(PointCloudIntensity cloud) -> std::unordered_map<std::string, PointCloudIntensity::Ptr> {
    std::unordered_map<std::string, PointCloudIntensity::Ptr> cloud_map;
    float origin_x = cloud[0].x;
    float origin_y = cloud[0].y;
    int width; int height;
    float boxSizeInv = 1/boxSize;

    for(pcl::PointXYZI point : cloud) {
      int index_x = (int)std::floor((point.x-origin_x)*boxSizeInv);
      int index_y = (int)std::floor((point.y-origin_y)*boxSizeInv);
      std::string key = std::to_string(index_x) + std::to_string(index_y);
      if(cloud_map.count(key) == 0) {
        PointCloudIntensity::Ptr inner_pointcloud (new PointCloudIntensity());
        cloud_map[key] = inner_pointcloud;
      }
      
      cloud_map[key]->push_back(point);
      
    }
    return cloud_map;
  }

  auto smoothstep(double edge0, double edge1, double x) -> double {
    if(x < edge0) return 0.0;
    if(x > edge1) return 1.0;

    x = (x - edge0) / (edge1 - edge0);
    return x*x*(3-2*x);

  }
  
  private:
  ros::NodeHandle n_ = *(new ros::NodeHandle("~")); 
  ros::Publisher pub_;
  ros::Subscriber sub_;

  float boxSize;
  int minPoints;
  float flatness_requirement;

  geometry_msgs::Transform current_transform;
  geometry_msgs::Transform ref_transform;
  bool set_new_reference_frame = true;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  PointCloudNormal::Ptr global_landing_map = PointCloudNormal::Ptr(new PointCloudNormal());
  PointCloudIntensity::Ptr local_dense_map = PointCloudIntensity::Ptr(new PointCloudIntensity());
  float voxel_size = 0.1f;

};



auto main(int argc, char **argv) -> int
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  
  ros::init(argc, argv, "listener");



  SLAD SLADObject;
  
  
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%