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
  public: 
  Mesh2D(voxblox_msgs::Mesh mesh, float box_size){
    //int block_scale = (int)std::ceil(box_size / mesh.block_edge_length);
    //auto grid_size = mesh.block_edge_length * block_scale;
    constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
    float origin_x;
    float origin_y;
    bool origin_set = false;
    for(auto mesh_block : mesh.mesh_blocks) {
      
      
      
      //int index_x = (int)std::floor((mesh_block.index[0])*block_scale);
      //int index_y = (int)std::floor((point.y-origin_y)*boxSizeInv);
      /*std::string key = std::to_string(mesh_block.index[0]) + " " + std::to_string(mesh_block.index[1]);
      if(cloud_map.count(key) == 0) {
        PointCloudNormal::Ptr inner_pointcloud (new PointCloudNormal());
        cloud_map[key] = inner_pointcloud;
      }*/
      for(int i = 0; i < mesh_block.x.size(); i++){
        
        pcl::PointXYZRGB point;
        
        
      point.x =
          (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
           static_cast<float>(mesh_block.index[0])) *
          mesh.block_edge_length;
      point.y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(mesh_block.index[1])) *
          mesh.block_edge_length;
      point.z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(mesh_block.index[2])) *
          mesh.block_edge_length;

        if(!origin_set){
          origin_set = true;
          origin_x = point.x;
          origin_y = point.y;
        }
        
        point.r = mesh_block.r[i];
        point.g = mesh_block.g[i];
        point.b = mesh_block.b[i];
        int index_x = (int)std::floor((point.x - origin_x)/box_size);
        int index_y = (int)std::floor((point.y - origin_y)/box_size);
        std::string key = std::to_string(index_x) + " " + std::to_string(index_y);
        if(cloud_map.count(key) == 0) {
          PointCloudNormal::Ptr inner_pointcloud (new PointCloudNormal());
          cloud_map[key] = inner_pointcloud;
        }
        cloud_map[key]->push_back(point);
      }
    }
  }
  auto getLandingCloud(int min_points) -> PointCloudNormal::Ptr {
    PointCloudNormal::Ptr landing_pc (new PointCloudNormal());
    landing_pc->header.frame_id = "world";
    for(auto kv : cloud_map) {
      if(kv.second->size() > min_points) {
        
        /*float nx; float ny; float nz; float curvature;
        std::vector<int> indices(kv.second->size());
        std::iota(indices.begin(), indices.end(), 0);
    
        NormalEstimationRGB ne;
        ne.setInputCloud(kv.second);
  
        ne.computePointNormal(*kv.second, indices, nx, ny, nz, curvature);
        Eigen::Vector4f centroid = ne.get_centroid();
        */
        //std::max_element(kv.second->points.begin(), kv.second->points.end());
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

        
        
        //point.x = centroid[0];
        //point.y = centroid[1];
        //point.z = centroid[2];
        if(max_r < 127 && min_g > 127){
          point.x /= kv.second->size();
          point.y /= kv.second->size();
          point.z /= kv.second->size();
          point.r = max_r;
          point.g = min_g;
          point.b = 0;
  
        //std::array<float, 5> arr = {};
        //landing_map[kv.first] = point;
          landing_pc->push_back(point);
        }
        
        

      
        
      };
    
    // inheriting flatness and curvature from neighbouring cells
    /*for(auto kv : landing_map) {
      int x = std::stoi(kv.first.substr(0, kv.first.find(" ")));
      int y = std::stoi(kv.first.substr(kv.first.find(" "), kv.first.length()));
      uint8_t worst_nz = kv.second.g;
      uint8_t worst_curvature = kv.second.r;
      bool has_neighbours = true;
      for(int i = -1; i < 2; i++)
      {
        for(int j = -1; j < 2; j++) {
          if(has_neighbours){
            std::string key = std::to_string(x+i) + " " + std::to_string(y+j);
            if(landing_map.count(key) == 0) 
            {
              has_neighbours = false;
              worst_nz = 0;
              worst_curvature = 255;
              
            }
            else{
              worst_nz = std::min(worst_nz, landing_map[key].g);
              worst_curvature = std::max(worst_curvature, landing_map[key].r);
            }
          }
        }
      }
      if(has_neighbours){
        pcl::PointXYZRGB new_point;
        new_point.x = kv.second.x;
        new_point.y = kv.second.y;
        new_point.z = kv.second.z;
        new_point.g = worst_nz;
        new_point.r = worst_curvature;
        new_point.b = kv.second.b;
        landing_pc->push_back(new_point);
      } 
    }*/

      
      
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
    n_.getParam("flatness_requirement", flatness_requirement);
    n_.getParam("voxel_size", voxel_size);

    ROS_INFO("box, min, flat: %f, %d, %f", boxSize, minPoints, flatness_requirement);
    
    //pub_ = n_.advertise<PointCloudNormal>("landing_pointcloud", 2);
    norm_pub_ = n_.advertise<PointCloudNormal>("normal_pointcloud", 2);

    //sub_ = n_.subscribe("input_pointcloud", 2, &SLAD::pointCloudCallback, this);
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
    Eigen::Affine3f transform;
    try {
      transform = tf2::transformToEigen(tf_buffer->lookupTransform("dense_cam", "world",
                                msg->header.stamp, ros::Duration(3.0)).transform).matrix().cast<float>();
    }
    catch (tf2::TransformException& e) {
        ROS_INFO("%s",e.what());
        
        return;
      }
    
    ROS_INFO("msg size %lu", msg->mesh_blocks.size());
    
    Mesh2D mesh_grid(*msg, boxSize);
    PointCloudNormal::Ptr cloud = mesh_grid.getLandingCloud(minPoints);
    auto t_cloud = sortCloudByLandingness(cloud, transform);
    ROS_INFO("best spot %f %f %f", cloud->at(0).x, cloud->at(0).y, cloud->at(0).z);
    cloud->header.frame_id = "dense_cam";
    pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
    
    norm_pub_.publish(*cloud);
    
    
    return;
    
    /*for(auto meshblock : msg->mesh_blocks)
    {
      if(meshblock.x.size() == 0) continue;
      PointCloudIntensity::Ptr test_cloud(new PointCloudIntensity());
      pcl::PointXYZRGB point;
      
      //Eigen::Vector3d origin {meshblock.index[0], meshblock.index[1], meshblock.index[2]};
      std::vector<Eigen::Vector3d> vertices;
      for(int i = 0; i < meshblock.x.size(); ++i)
      {
        Eigen::Vector3d vertex;
        pcl::PointXYZI test_point;

        constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      vertex[0] =
          (static_cast<float>(meshblock.x[i]) * point_conv_factor +
           static_cast<float>(meshblock.index[0])) *
          msg->block_edge_length;
      vertex[1] =
          (static_cast<float>(meshblock.y[i]) * point_conv_factor +
           static_cast<float>(meshblock.index[1])) *
          msg->block_edge_length;
      vertex[2] =
          (static_cast<float>(meshblock.z[i]) * point_conv_factor +
           static_cast<float>(meshblock.index[2])) *
          msg->block_edge_length;
      
      test_point.x = vertex[0];
      test_point.y = vertex[1];
      test_point.z = vertex[2];
      test_point.intensity = 100.0;

      vertices.emplace_back(vertex);
      test_cloud->push_back(test_point);

      }
      std::vector<Eigen::Vector3d> centroids;
      

      std::vector<Eigen::Vector3d> normals;
      //double curvature = 0;
      
      for (size_t i = 0; i < vertices.size(); i += 3) {

        centroids.emplace_back(std::accumulate(vertices.begin()+i, vertices.begin()+i+3, Eigen::Vector3d{0.0, 0.0, 0.0})/3);
      

        const Eigen::Vector3d dir0 = vertices[i] - vertices[i + 1];
        const Eigen::Vector3d dir1 = vertices[i] - vertices[i + 2];
        const Eigen::Vector3d normal = dir0.cross(dir1).normalized();
        normals.push_back(normal);

      }
      //estimating curvature:
      //Eigen::Vector3d normal_finite_difference {0.0, 0.0, 0.0};
      // remember to comment this out
      double normal_finite_difference = 0;
      for(size_t i = 1; i < normals.size(); i++){
        //for(size_t j = i+1; j < normals.size(); j++){
          double distance = (centroids[i] - centroids[0]).norm();
          normal_finite_difference += (normals[i] - normals[0]).norm()/distance;


        //}
      }
      ROS_INFO("before normalizing %f", normal_finite_difference);
      normal_finite_difference /= normals.size();
      ROS_INFO("after %f", normal_finite_difference);
      
      Eigen::Vector3d normal_average = std::accumulate(normals.begin(), normals.end(), Eigen::Vector3d{0.0, 0.0, 0.0}).normalized();
      Eigen::Vector3d centroid = std::accumulate(centroids.begin(), centroids.end(), Eigen::Vector3d{0.0, 0.0, 0.0})/centroids.size();
      
      float nx; float ny; float nz; float curvature;
      std::vector<int> indices(test_cloud->size());
      std::iota(indices.begin(), indices.end(), 0);
  
      NormalEstimationOMP ne;
      ne.setInputCloud(test_cloud);

      ne.computePointNormal(*test_cloud, indices, nx, ny, nz, curvature);

      ROS_INFO("pcl normal: %f %f %f", nx, ny, nz);
      ROS_INFO("avg normal: %f %f %f", normal_average[0], normal_average[1], normal_average[2]);

      Eigen::Vector4f pcl_centroid = ne.get_centroid();
      ROS_INFO("pcl centroid: %f %f %f", pcl_centroid[0], pcl_centroid[1], pcl_centroid[2]);
      ROS_INFO("avg centrod: %f %f %f", centroid[0], centroid[1], centroid[2]);

      ROS_INFO("pcl curvature: %f", curvature);



      point.x = centroid[0];
      point.y = centroid[1];
      point.z = centroid[2];

      point.g = 255*smoothstep(0.6, 1.0, normal_average[2]);
      point.b = 0;
      point.r = 3*255*curvature;

      cloud.push_back(point);

    }
    cloud.header.frame_id = "world";
    norm_pub_.publish(cloud);*/
    

  }

  PointCloudNormal::Ptr sortCloudByLandingness(PointCloudNormal::Ptr landing_cloud,
                              Eigen::Affine3f transform) {
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

  // %Tag(CALLBACK)%
  void pointCloudCallback(const PointCloudIntensity::ConstPtr& msg)
  {
    
    PointCloudIntensity::Ptr temp_pc (new PointCloudIntensity);
    
    //pcl::io::savePCDFileASCII("test_pcd.pcd", *temp_pc);
    current_transform = tf_buffer->lookupTransform("world", "cam_pos", ros::Time(0), ros::Duration(3.0)).transform;
    
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
  ros::Publisher norm_pub_;
  ros::Subscriber sub_;
  ros::Subscriber mesh_sub_;

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