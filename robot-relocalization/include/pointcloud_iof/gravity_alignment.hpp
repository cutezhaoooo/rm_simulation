#pragma once
#include <Eigen/Core>

namespace pciof {
inline Eigen::Matrix4f calc_gravity_alignment_matrix(const Eigen::Vector3f& acc) {
  Eigen::Vector3f th;
  // 计算imu的俯仰角和横滚角
  th.x() = std::atan2(acc.y(), acc.z());                                                // roll
  th.y() = std::atan2(-acc.x(), std::sqrt(acc.y() * acc.y() + acc.z() * acc.z()));      // pitch
  th.z() = 0.0f;                                                                        // yaw

  // 构造一个旋转矩阵
  Eigen::Matrix3f rot;
  rot = Eigen::AngleAxisf(th.x(), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(th.y(), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(th.z(), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = rot;

  return mat;
}
}  // namespace pciof
