#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace util {
    Eigen::Matrix3d poseToRotation(geometry_msgs::Pose pose) 
    {
        Eigen::Matrix3d mat;
        Eigen::Quaterniond q;

        q.w() = pose.orientation.w;
        q.x() = pose.orientation.x;
        q.y() = pose.orientation.y;
        q.z() = pose.orientation.z;
        
        return q.toRotationMatrix();
    }

    Eigen::Vector3d poseToPosition(geometry_msgs::Pose pose) 
    {
        Eigen::Vector3d vec {pose.position.x, pose.position.y, pose.position.z};
        
        return vec;
    }
}
