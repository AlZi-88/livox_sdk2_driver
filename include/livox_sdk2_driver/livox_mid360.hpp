#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
// #include Livox SDK headers
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
// include of Eigen library for PointCloud transformation
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
// include filter algo
#include "livox_sdk2_driver/filters/madgwick.hpp"

class LivoxMid360Node : public rclcpp::Node
{
public:
    LivoxMid360Node();

    ~LivoxMid360Node();

    void PublishImuData();
    void PublishPointCloudData();
    void ConvertToPointCloud2(LivoxLidarEthernetPacket* data);
    void ConvertToIMUData(LivoxLidarEthernetPacket* data);
    bool IsPointCloudDataError() const { return pt_cloud_data_error_; }
    bool IsImuDataError() const { return imu_data_error_; }
    void SetPointCloudDataError(bool error) { pt_cloud_data_error_ = error; }
    void SetImuDataError(bool error) { imu_data_error_ = error; }
  
private:

    bool InitSDK();
    Eigen::Quaterniond InterpolateIMUOrientation(const rclcpp::Time& timestamp);
    uint64_t ParseTimestamp(const uint8_t* timestamp_field);

    std::string pointcloud_topic_;
    std::string imu_topic_;
    std::string config_file_path_;
    double pc_freq_;
    double imu_freq_;
    bool pt_cloud_data_error_;
    bool imu_data_error_;
    std::vector<double> lidar_translation_;
    std::vector<double> lidar_rotation_rpy_deg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ptcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr pc_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    sensor_msgs::msg::PointCloud2 latest_pc_msg_;
    std::shared_ptr<sensor_msgs::msg::Imu> latest_imu_msg_;
    std::mutex data_mutex_; 

    std::deque<sensor_msgs::msg::Imu> imu_buffer_; // Buffer to store IMU data
    std::mutex imu_buffer_mutex_; // Mutex to protect the IMU buffer

    MadgwickFilter madgwick_filter_; // Instance of the Madgwick filter for orientation estimation

    // Transform broadcaster for static transforms
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};