#include "graspnet_ros/transform_utils.hpp"

namespace graspnet_ros {

Eigen::Matrix4f tfToEigenMatrix(const geometry_msgs::msg::TransformStamped& transform) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    // Extract quaternion and translation
    Eigen::Quaternionf q(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z
    );
    
    Eigen::Vector3f t(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );
    
    // Build transformation matrix
    matrix.block<3,3>(0,0) = q.toRotationMatrix();
    matrix.block<3,1>(0,3) = t;
    
    return matrix;
}

Eigen::Quaternionf rotationToQuaternion(const Eigen::Matrix3f& rotation) {
    return Eigen::Quaternionf(rotation);
}

Grasp transformGrasp(const Grasp& grasp, const Eigen::Matrix4f& transform_matrix) {
    Grasp transformed = grasp;
    
    // Extract rotation and translation from transform matrix
    Eigen::Matrix3f R = transform_matrix.block<3,3>(0,0);
    Eigen::Vector3f t = transform_matrix.block<3,1>(0,3);
    
    // Apply transformation: p_new = R*p + t, R_new = R*R_old
    transformed.translation = R * grasp.translation + t;
    transformed.rotation = R * grasp.rotation;
    
    return transformed;
}

std::vector<Grasp> transformGrasps(const std::vector<Grasp>& grasps, 
                                    const Eigen::Matrix4f& transform_matrix) {
    std::vector<Grasp> transformed_grasps;
    transformed_grasps.reserve(grasps.size());
    
    for (const auto& grasp : grasps) {
        transformed_grasps.push_back(transformGrasp(grasp, transform_matrix));
    }
    
    return transformed_grasps;
}

void transformPoints(std::vector<Eigen::Vector3f>& points, 
                     const Eigen::Matrix4f& transform_matrix) {
    for (auto& p : points) {
        Eigen::Vector4f p4(p.x(), p.y(), p.z(), 1.0f);
        Eigen::Vector4f pt = transform_matrix * p4;
        p = pt.head<3>();
    }
}

} // namespace graspnet_ros
