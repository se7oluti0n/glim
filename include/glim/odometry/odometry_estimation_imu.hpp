#pragma once

#include <map>
#include <memory>
#include <random>

#include <boost/shared_ptr.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>

namespace gtsam {
class Pose3;
class Values;
class ImuFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class OdomIntegration;
class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;
class InitialStateEstimation;

/**
 * @brief Parameters for OdometryEstimationIMU
 */
struct OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMUParams();
  virtual ~OdometryEstimationIMUParams();

public:
  // Sensor params;
  bool fix_imu_bias;
  double imu_bias_noise;
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  // Init state
  std::string initialization_mode;
  bool estimate_init_state;
  Eigen::Isometry3d init_T_world_imu;
  Eigen::Vector3d init_v_world_imu;
  double init_pose_damping_scale;

  // Optimization params
  double smoother_lag;
  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  // Logging params
  bool save_imu_rate_trajectory;

  int num_threads;                  // Number of threads for preprocessing and per-factor parallelism
  int num_smoother_update_threads;  // Number of threads for TBB parallelism in smoother update (should be kept 1)

  // wheel
  bool use_wheel;
  Eigen::Isometry3d T_robot_imu;
  double wheel_radius;  // [m]  Please use the actual wheel radius of your robot.
  double wheel_base;    // [m]  Please use the actual wheelbase of your robot.

  double differential_drive_model_noise;
  double wheel_odometry_noise;
};

/**
 * @brief Base class for LiDAR-IMU odometry estimation
 */
class OdometryEstimationIMU : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params);
  virtual ~OdometryEstimationIMU() override;

  virtual void insert_raw_odom(const double stamp, const double left_angular_vel, const double right_angular_vel) override;
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

protected:
  virtual void create_frame(EstimationFrame::Ptr& frame) {}
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) = 0;

  virtual void fallback_smoother() {}
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors);

  virtual void
  update_smoother(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values, const std::map<std::uint64_t, double>& new_stamp, int update_count = 0);
  virtual void update_smoother(int update_count = 1);

protected:
  std::unique_ptr<OdometryEstimationIMUParams> params;

  // Sensor extrinsic params
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // Frames & keyframes
  int marginalized_cursor;
  std::vector<EstimationFrame::Ptr> frames;

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<OdomIntegration> odom_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
