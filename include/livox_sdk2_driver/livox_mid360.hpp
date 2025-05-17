#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include Livox SDK headers
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

class LivoxMid360Node : public rclcpp::Node
{
public:
    LivoxMid360Node();

    ~LivoxMid360Node();

    void PublishImuData();
    void PublishPointCloudData();
    void ConvertToPointCloud2(LivoxLidarEthernetPacket* data);
    void ConvertToIMUData(LivoxLidarEthernetPacket* data);
  
private:

    bool InitSDK();


    std::string pointcloud_topic_;
    std::string imu_topic_;
    std::string config_file_path_;
    double pc_freq_;
    double imu_freq_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ptcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr pc_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_pc_msg_;
    std::shared_ptr<sensor_msgs::msg::Imu> latest_imu_msg_;
    std::mutex data_mutex_; 

  // SDK callbacks and helpers...
};