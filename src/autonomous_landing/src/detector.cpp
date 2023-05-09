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
#include <pcl/io/pcd_io.h>
#include <string>
#include <opencv2/core/mat.hpp>
#include "opencv2/imgproc.hpp"
#include <voxblox_msgs/Mesh.h>

using PointCloudIntensity = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudNormal = pcl::PointCloud<pcl::PointXYZRGB>;

class NormalEstimationOMP : public pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> {
  public:
  auto get_centroid() -> Eigen::Vector4f {
    return xyz_centroid_;
  }
};
class NormalEstimationRGB : public pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> {
  public:
  auto get_centroid() -> Eigen::Vector4f {
    return xyz_centroid_;
  }
};

class Mesh2D {
  private: 
  std::unordered_map<std::string, PointCloudNormal::Ptr> cloud_map;
  std::unordered_map<std::string, pcl::PointXYZRGB> landing_map;
  std::unordered_map<std::string, pcl::PointXYZRGB> landing_map_out;
  float box_size;
  public:
    Mesh2D(voxblox_msgs::Mesh mesh, float box_size_in) {
      // int block_scale = (int)std::ceil(box_size / mesh.block_edge_length);
      // auto grid_size = mesh.block_edge_length * block_scale;
      box_size = box_size_in;
      constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      float origin_x;
      float origin_y;
      bool origin_set = false;
      for (auto mesh_block : mesh.mesh_blocks) {

        // saving computation time by only considering one vertex per face
        for (int i = 0; i < mesh_block.x.size(); i += 3) {

          pcl::PointXYZRGB point;

          point.x = (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
                     static_cast<float>(mesh_block.index[0])) *
                    mesh.block_edge_length;
          point.y = (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
                     static_cast<float>(mesh_block.index[1])) *
                    mesh.block_edge_length;
          point.z = (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
                     static_cast<float>(mesh_block.index[2])) *
                    mesh.block_edge_length;

          if (!origin_set) {
            origin_set = true;
            origin_x = point.x;
            origin_y = point.y;
          }

          point.r = mesh_block.r[i];
          point.g = mesh_block.g[i];
          point.b = mesh_block.b[i];
          int index_x = (int)std::floor((point.x - origin_x) / box_size_in);
          int index_y = (int)std::floor((point.y - origin_y) / box_size_in);
          std::string key =
              std::to_string(index_x) + " " + std::to_string(index_y);
          if (cloud_map.count(key) == 0) {
            PointCloudNormal::Ptr inner_pointcloud(new PointCloudNormal());
            cloud_map[key] = inner_pointcloud;
          }
          cloud_map[key]->push_back(point);
        }
      }
    }

  auto gaussian(float x, float sigma){
    float exp_factor = std::exp(-0.5*(x/sigma)*(x/sigma));
    return exp_factor;
  }

  auto getLandingCloud(int min_points, uint8_t min_z_comp, uint8_t max_curve) -> PointCloudNormal::Ptr {
    PointCloudNormal::Ptr landing_pc (new PointCloudNormal());
    landing_pc->header.frame_id = "world";
    for(auto kv : cloud_map) {
      if(kv.second->size() > min_points) {
        
        pcl::PointXYZRGB point;
        uint8_t max_r = 0;
        uint8_t min_g = 255;
        for(auto pt : kv.second->points){
          max_r = std::max(max_r, pt.r);
          min_g = std::min(min_g, pt.g);
          point.x += pt.x;
          point.y += pt.y;
          point.z += pt.z;
        }
        int r = 0;
        int g = 255;
        float sum_weight = 0;
        point.x /= kv.second->size();
        point.y /= kv.second->size();
        point.z /= kv.second->size();
        bool bad_pts = false;

        for(auto pt : kv.second->points){
          if(pt.r > max_curve || pt.g < min_z_comp){
            bad_pts = true;
            auto dist_2d = std::sqrt((point.x - pt.x) * (point.x - pt.x) +
                                   (point.y - pt.y) * (point.y - pt.y));
            auto normalized_dist = 2 * dist_2d / (box_size);
            float weight = gaussian(dist_2d, 1);
            if(pt.r > r){
              float r_new = weight*pt.r;
              if(r_new > r) {
                r = r_new;
              }
            }
              
              
            if(pt.g < g){
              

              float g_new = weight*pt.g;
              if(g_new < g) {
                g = g_new;
              }
            }
              
            sum_weight += weight;
          }
          
        }

        uint8_t ur;
        uint8_t ug;

        if(bad_pts){
          ur = r;
          ug = g;
        }
        else {
          ur = max_r;
          ug = min_g;
        }
        //point.x = centroid[0];
        //point.y = centroid[1];
        //point.z = centroid[2];
        if(ur < max_curve && ug > min_z_comp){
          
          point.r = ur;
          point.g = ug;
          point.b = 0;
  
        //std::array<float, 5> arr = {};
        //landing_map[kv.first] = point;
          landing_pc->push_back(point);
        }
        
    
        
      };
      
    }

    return landing_pc;
    
  }
  private:
  auto smoothstep(double edge0, double edge1, double x) -> double {
    if(x < edge0) return 0.0;
    if(x > edge1) return 1.0;

    x = (x - edge0) / (edge1 - edge0);
    return x*x*(3-2*x);

  }

};

class SLAD
{
  public:
  SLAD() {
    
    n_.getParam("boxSize", boxSize);
    n_.getParam("minPoints", minPoints);
    n_.getParam("voxel_size", voxel_size);
    n_.getParam("max_angle_offset_deg", max_angle_offset);
    n_.getParam("max_curvature", max_curvature);
    n_.getParam("save_to_file", save_to_file);
    n_.getParam("filename", filename);

    minimum_z_component = 255*lerp(0.85, 1.0, std::cos(max_angle_offset*3.14/180));
    max_curvature_8bit = 255*lerp(0.0, 0.05, max_curvature);
    //pub_ = n_.advertise<PointCloudNormal>("landing_pointcloud", 2);
    norm_pub_ = n_.advertise<PointCloudNormal>("normal_pointcloud", 2);

    mesh_sub_ = n_.subscribe("input_mesh", 10, &SLAD::meshCallback, this);
    tf_buffer = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    global_landing_map->header.frame_id = "world";
  }
  //void mapCallback(const )

  //largely derived from voxblox
  void meshCallback(const voxblox_msgs::Mesh::ConstPtr& msg)
  {

    ROS_INFO("meshcallback");
    /*Eigen::Affine3f transform;
    try {
      transform = tf2::transformToEigen(tf_buffer->lookupTransform("dense_cam", "world",
                                msg->header.stamp, ros::Duration(3.0)).transform).matrix().cast<float>();
    }
    catch (tf2::TransformException& e) {
        ROS_INFO("%s",e.what());
        
        return;
      }*/
    
    ROS_INFO("msg size %lu", msg->mesh_blocks.size());
    
    Mesh2D mesh_grid(*msg, boxSize);
    PointCloudNormal::Ptr cloud = mesh_grid.getLandingCloud(minPoints, minimum_z_component, max_curvature_8bit);
    ROS_INFO("cloud size %lu", cloud->size());
    /*auto t_cloud = sortCloudByLandingness(cloud, transform);
    if(!t_cloud->empty())
      ROS_INFO("best spot %f %f %f", cloud->at(0).x, cloud->at(0).y, cloud->at(0).z);
    */
    
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    
    norm_pub_.publish(*cloud);
    if(save_to_file && !cloud->empty())
    pcl::io::savePCDFileASCII(std::string("/home/nvidia/AutonomousLanding/") + filename, *cloud);
    return;
    
  }

  PointCloudNormal::Ptr sortCloudByLandingness(PointCloudNormal::Ptr landing_cloud,
                              Eigen::Affine3f transform) {
    if(landing_cloud->empty()) return PointCloudNormal::Ptr(new PointCloudNormal());
    ROS_INFO("best spot:");
    float best_cost = 1e10;
    geometry_msgs::Vector3 best_spot;
    struct {
      bool operator()(pcl::PointXYZRGB a, pcl::PointXYZRGB b) const {
        ROS_INFO("comparing");
        return a.getVector3fMap().squaredNorm() * (255 - a.b) < b.getVector3fMap().squaredNorm() * (255 - b.b);
      }
    }
    better_than;

    PointCloudNormal::Ptr transformed_cloud(new PointCloudNormal());
    pcl::transformPointCloud(*landing_cloud, *transformed_cloud, transform);
    ROS_INFO("transform ok");
    for(auto& point : transformed_cloud->points) {
      /*auto dist = point.x*point.x
        + point.y*point.y
        + point.z*point.z;*/
      auto cost = (1.0 + (float)point.r/255)/(1 + (float)point.g/255);
      
      point.b = 255 - ((cost-0.5)/2)*255;
    }
    ROS_INFO("sorting");
    std::sort(transformed_cloud->points.begin(), transformed_cloud->points.end(), better_than);
    *landing_cloud = *transformed_cloud;
    return transformed_cloud;
    
  }

  auto lerp(double edge0, double edge1, double x) -> double {
    if(x < edge0) return 0.0;
    if(x > edge1) return 1.0;

    x = (x - edge0) / (edge1 - edge0);
    return x;
  }

  auto smoothstep(double edge0, double edge1, double x) -> double {
    
    double y = lerp(edge0, edge1, x);
    return y*y*(3-2*y);
  }
  
  private:
  ros::NodeHandle n_ = *(new ros::NodeHandle("~")); 
  ros::Publisher pub_;
  ros::Publisher norm_pub_;
  ros::Subscriber sub_;
  ros::Subscriber mesh_sub_;

  float boxSize;
  int minPoints;
  float max_angle_offset;
  float max_curvature;
  uint8_t max_curvature_8bit;
  uint8_t minimum_z_component;
  bool save_to_file = false;
  std::string filename;

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