#include <glim/util/transform.hpp>

namespace glim {
  Eigen::Isometry3d remove_roll_pitch(const Eigen::Isometry3d & pose) {
    Eigen::Isometry3d result = pose;

    Eigen::Vector3d euler = pose.rotation().eulerAngles(2, 1, 0);

    Eigen::Matrix3d new_rotation;
    new_rotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

    result.linear() = new_rotation;
    return result;
  }
}