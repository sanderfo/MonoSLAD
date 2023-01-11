#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace util {
    Eigen::Matrix3d poseToRotation(geometry_msgs::Pose pose);
    Eigen::Vector3d poseToPosition(geometry_msgs::Pose pose);
}