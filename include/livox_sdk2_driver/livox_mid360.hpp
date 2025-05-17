#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include Livox SDK headers
//#include "livox_lidar_def.h"
//#include "livox_lidar_api.h"

class LivoxMid360Node : public rclcpp::Node
{
public:
    LivoxMid360Node();

    ~LivoxMid360Node();

    //void PublishImuData(LivoxLidarEthernetPacket* data);
    //void PublishPointCloudData(LivoxLidarEthernetPacket* data);
  
private:

    bool InitSDK();


    std::string pointcloud_topic_;
    std::string imu_topic_;
    std::string config_file_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ptcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // SDK callbacks and helpers...
};