/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/centroid.h>
#include <pcl/features/linear_least_squares_normal.h>
#include <boost/foreach.hpp>
#include <string>
#include <opencv2/core/mat.hpp>
#include "opencv2/imgproc.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class SLAD
{
  public:
  SLAD() {
    n_.getParam("boxSize", boxSize);
    n_.getParam("minPoints", minPoints);
    n_.getParam("flatness_requirement", flatness_requirement);

    ROS_INFO("box, min, flat: %f, %d, %f", boxSize, minPoints, flatness_requirement);
    
    pub_ = n_.advertise<PointCloud>("autonomous_landing/landing_pointcloud", 2);

    sub_ = n_.subscribe("/svo/global_map_pts", 1, &SLAD::pointCloudCallback, this);
  }

  // %Tag(CALLBACK)%
void pointCloudCallback(const PointCloud::ConstPtr& msg)
{
  PointCloud::Ptr temp_pc (new PointCloud);
  *temp_pc = *msg;

  PointCloud::Ptr landing_pc = splitPointCloud(temp_pc);
  landing_pc->header.frame_id = std::string("world");
 
  pub_.publish(landing_pc);
  

  
}
// %EndTag(CALLBACK)%
auto calculateFlatness(PointCloud::Ptr cloud) -> float
{
  if(cloud->size() < minPoints) {
    return 0;
  };
  
  float nx; float ny; float nz; float curvature;
  std::vector<int> indices(cloud->size());
  std::iota(indices.begin(), indices.end(), 0);
  
  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::Normal out;
  ne.computePointNormal(*cloud, indices, nx, ny, nz, curvature);

  
  float fitness = std::abs(nz)/(1+10*std::abs(curvature)); // higher is better, max is 1, min is 0
  return fitness;
}

auto splitPointCloud(PointCloud::Ptr pointCloud) -> PointCloud::Ptr
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;

  feature_extractor.setInputCloud(pointCloud);
  feature_extractor.compute();

  pcl::PointXYZI min_point;
  pcl::PointXYZI max_point;
  feature_extractor.getAABB(min_point, max_point);

  int grid_width = std::ceil((max_point.x - min_point.x)/boxSize);
  int grid_height = std::ceil((max_point.y - min_point.y)/boxSize);

  cv::Mat fitness_img(grid_width, grid_height, CV_32F);
  cv::Mat coordinate_image(grid_width, grid_height, CV_32FC3);
  cv::Mat filtered_fitness(grid_width, grid_height, CV_32F);

  PointCloud::Ptr landing_pc (new PointCloud());

  //fitness_img.forEach<float>([this, pointCloud, min_point, max_point, &coordinate_image](float& fitness, const int pos[]) -> void {
  for(int i = 0; i < grid_width; i++)
  {
    for (int j = 0; j < grid_height; j++)
    {
      //int i = pos[0]; int j = pos[1];
    
    float x_min = min_point.x + i*boxSize;
    float x_max = min_point.x + i*boxSize + boxSize;
    float y_min = min_point.y + j*boxSize;
    float y_max = min_point.y + j*boxSize + boxSize;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (pointCloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    
    PointCloud::Ptr inner_pc (new PointCloud);
    
    pass.filter(*inner_pc);
    pass.setInputCloud (inner_pc);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter(*inner_pc);
    float flatness = calculateFlatness(inner_pc);
    fitness_img.at<float>(i, j) = flatness;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*inner_pc, centroid);
    coordinate_image.at<Eigen::Vector3f>(i, j) = {centroid[0], centroid[1], centroid[2]};
     

    }

  }  
     
  //});

  cv::GaussianBlur(fitness_img, filtered_fitness, cv::Size(5, 5), 1.5);

  cv::Mat filtered_min(grid_width, grid_height, CV_32F);

  cv::min(fitness_img, filtered_fitness, filtered_min); // so that fitness can only get worse from neighbours

  //std::cout << fitness_img << "\n";

  //fitness_img.forEach<float>([this, &coordinate_image, landing_pc](float& fitness, const int pos[]) -> void {
  for(int i = 0; i< grid_width; i++) {
    float * ptr = filtered_min.ptr<float>(i);
    for(int j = 0; j < grid_height; j++) {
      if(ptr[j] > flatness_requirement) {
      Eigen::Vector3f coords = coordinate_image.at<Eigen::Vector3f>(i, j);
      pcl::PointXYZI pt;
      pt.x = coords[0];
      pt.y = coords[1];
      pt.z = coords[2];
      pt.intensity = ptr[j];
      landing_pc->push_back(pt);
    }
    }
  }
    

    //}
  //});

  return landing_pc;

}
  private:
  ros::NodeHandle n_ = *(new ros::NodeHandle("~")); 
  ros::Publisher pub_;
  ros::Subscriber sub_;

  float boxSize;
  int minPoints;
  float flatness_requirement;

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