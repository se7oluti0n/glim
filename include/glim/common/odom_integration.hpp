#pragma once

#include <deque>
#include <vector>
#include <Eigen/Core>

namespace glim {

class OdomIntegration {
public:
    OdomIntegration(){}

    /**
     * @brief Insert an Raw Differential Wheel angular velocity
     * @param stamp         Timestamp
     * @param left_angular_vel    Left wheel angular velocity rad/s
     * @param right_angular_vel   Right wheel angular velocity rad/s
     */
    void insert_raw_odom(const double stamp, const double left_angular_vel, const double right_angular_vel);

    /**
     * @brief Integrate Odom measurements in a time range
     * @param start_time     Integration starting time
     * @param end_time       Integration ending time
     * @param num_integrated Number of integrated IMU measurements
     * @return Index of the last integrated IMU frame
     */
    int integrate_odom(double start_time, double end_time, 
        double& left_angle_displacement, double& right_angle_displacement);

    /**
     * @brief Erase Odom data before the given index
     * @param last Last integrated IMU measurement index
     */
    void erase_odom_data(int last);

private:
  std::deque<Eigen::Matrix<double, 3, 1>> odom_queue;

};

}