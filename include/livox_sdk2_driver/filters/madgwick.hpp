// madgwick.hpp
#pragma once
#include <rclcpp/time.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "sensor_msgs/msg/imu.hpp"

class MadgwickFilter {
public:
    MadgwickFilter(double beta = 0.1);

    void update(const sensor_msgs::msg::Imu& imu_msg);
    Eigen::Quaterniond getOrientation() const;

private:
    Eigen::Quaterniond q_;
    rclcpp::Time last_update_;
    double beta_;
};