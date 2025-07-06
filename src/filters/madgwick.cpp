// madgwick.cpp
#include "livox_sdk2_driver/filters/madgwick.hpp"

MadgwickFilter::MadgwickFilter(double beta) : q_(Eigen::Quaterniond::Identity()), beta_(beta) {}

void MadgwickFilter::update(const sensor_msgs::msg::Imu& imu_msg) {
    // Implementation of the Madgwick filter algorithm
    if (last_update_.nanoseconds() == 0) {
        last_update_ = imu_msg.header.stamp;
        return; // Skip the first update
    }

    double dt = (imu_msg.header.stamp - last_update_).seconds();
    last_update_ = imu_msg.header.stamp;

    Eigen::Vector3d gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

    // Normalize accelerometer measurement
    if (acc.norm() > 0.0) {
        acc.normalize();
    } else {
        return; // Skip if accelerometer data is invalid
    }

    // Quaternion elements
    double q0 = q_.w();
    double q1 = q_.x();
    double q2 = q_.y();
    double q3 = q_.z();

    // Reference direction of Earth's gravity
    // ∇f = Jᵗ · f, wobei f = [f0 f1 f2], Jᵗ abgeleitet nach q = [q0 q1 q2 q3]
    double f0 = 2.0*(q1*q3 - q0*q2) - acc.x();
    double f1 = 2.0*(q0*q1 + q2*q3) - acc.y();
    double f2 = 2.0*(0.5 - q1*q1 - q2*q2) - acc.z();

    // Gradient descent algorithm corrective step
    double J_00or13 = 2.0*q2; // Partial derivative of f0 to q0
    double J_01or12 = 2.0*q3; // Partial derivative of f0 to q1
    double J_02or11 = 2.0*q0; // Partial derivative of f0 to q2
    double J_03or10 = 2.0*q1; // Partial derivative of f0 to q3
    double J_21 = 2.0*J_03or10; // Partial derivative of f1 to q1
    double J_22 = 2.0*J_00or13; // Partial derivative of f1 to q2

    //Gradient descent step
    Eigen::Vector4d step;
    step[0] = J_03or10*f1 - J_00or13*f0;
    step[1] = J_01or12*f0 + J_02or11*f1 - J_21*f2;
    step[2] = J_01or12*f1 - J_22*f2 - J_02or11*f0;
    step[3] = J_03or10*f0 + J_00or13*f1;

    // Normalize step
    step.normalize();

    // Compute change of rate for quaternion
    Eigen::Quaterniond dq(
        0,
        0.5 * (-q1 * gyro.x() - q2 * gyro.y() - q3 * gyro.z()),
        0.5 * (q0 * gyro.x() + q2 * gyro.z() - q3 * gyro.y()),
        0.5 * (q0 * gyro.y() - q1 * gyro.z() + q3 * gyro.x())
    );

    dq.w() = 0.5 * (-q1 * gyro.x() - q2 * gyro.y() - q3 * gyro.z());
    dq.x() = 0.5 * (q0 * gyro.x() + q2 * gyro.z() - q3 * gyro.y());
    dq.y() = 0.5 * (q0 * gyro.y() - q1 * gyro.z() + q3 * gyro.x());
    dq.z() = 0.5 * (q0 * gyro.z() + q1 * gyro.y() - q2 * gyro.x());

    // Integrate the quaternion change
    q_.w() += (dq.w() - beta_*step[0]) * dt;
    q_.x() += (dq.x() - beta_*step[1]) * dt;
    q_.y() += (dq.y() - beta_*step[2]) * dt;
    q_.z() += (dq.z() - beta_*step[3]) * dt;

    // Normalize the quaternion
    q_.normalize();
    return;

}

Eigen::Quaterniond MadgwickFilter::getOrientation() const {
  return q_;
}