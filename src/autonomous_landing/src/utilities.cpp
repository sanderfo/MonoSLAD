#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace util {
    Eigen::Matrix4d poseToRotationAndTranslation(geometry_msgs::Pose pose, Eigen::Matrix4d Tbc) 
    {
      Eigen::Matrix3d R_mat;
      Eigen::Quaterniond q;

        q.w() = pose.orientation.w;
        q.x() = pose.orientation.x;
        q.y() = pose.orientation.y;
        q.z() = pose.orientation.z;

        R_mat = q.toRotationMatrix();

        Eigen::Vector3d T_vec;
        T_vec << pose.position.x, pose.position.y, pose.position.z;
        
        Eigen::Matrix4d eig_pose;
        eig_pose.setIdentity();
        eig_pose.topLeftCorner(3, 3) = R_mat;
        eig_pose.topRightCorner(3, 1) = T_vec;


        return eig_pose;
    }

    Eigen::Vector3d poseToPosition(geometry_msgs::Pose pose) 
    {
        Eigen::Vector3d vec {pose.position.x, pose.position.y, pose.position.z};
        
        return vec;
    }
}
