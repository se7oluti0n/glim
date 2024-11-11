#include <glim/common/odom_integration.hpp>


namespace glim
{
/**
 * @brief Insert an Raw Differential Wheel angular velocity
 * @param stamp         Timestamp
 * @param left_angular_vel    Left wheel angular velocity rad/s
 * @param right_angular_vel   Right wheel angular velocity rad/s
 */
void OdomIntegration::insert_raw_odom(const double stamp, const double left_angular_vel, const double right_angular_vel) {
    Eigen::Matrix<double, 3, 1> odom_data;
    odom_data << stamp, left_angular_vel, right_angular_vel;
    odom_queue.push_back(odom_data);
}

int OdomIntegration::integrate_odom(double start_time, double end_time, 
    double& left_angle_displacement, double& right_angle_displacement) 
{
    int cursor = 0;
    auto odom_itr = odom_queue.begin();
    double last_stamp = start_time;
    left_angle_displacement = 0.0;
    right_angle_displacement = 0.0;

  for (; odom_itr != odom_queue.end(); odom_itr++, cursor++) {
    const auto& imu_frame = *odom_itr;
    const double odom_stamp = imu_frame[0];

    if (odom_stamp > end_time) {
      break;
    }

    const double dt = odom_stamp - last_stamp;
    if (dt <= 0.0) {
      continue;
    }

    const auto& left_angular_vel = imu_frame[1];
    const auto& right_angular_vel = imu_frame[2];

    left_angle_displacement += (left_angular_vel * dt);
    right_angle_displacement += (right_angular_vel * dt);

    // const auto& a = imu_frame.block<3, 1>(1, 0);
    // const auto& w = imu_frame.block<3, 1>(4, 0);
    // imu_measurements->integrateMeasurement(a, w, dt);

    last_stamp = odom_stamp;
    // (*num_integrated)++;
  }

  const double dt = end_time - last_stamp;
  if (dt > 0.0) {
    Eigen::Matrix<double, 3, 1> last_imu_frame = odom_itr == odom_queue.end() ? *(odom_itr - 1) : *odom_itr;
    const auto& left_angular_vel = last_imu_frame[1];
    const auto& right_angular_vel = last_imu_frame[2];

    left_angle_displacement += (left_angular_vel * dt);
    right_angle_displacement += (right_angular_vel * dt);
  }

    return cursor;
}

void OdomIntegration::erase_odom_data(int last) {
  odom_queue.erase(odom_queue.begin(), odom_queue.begin() + last);
}


} // namespace glim