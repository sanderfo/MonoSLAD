#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace util {
    Eigen::Matrix4d poseToRotationAndTranslation(geometry_msgs::Pose pose, Eigen::Matrix4d Tbc);
}