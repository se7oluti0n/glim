#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {
  Eigen::Isometry3d remove_roll_pitch(const Eigen::Isometry3d & pose);
}  // namespace glim
