#include "livox_sdk2_driver/livox_mid360.hpp"

LivoxMid360Node::LivoxMid360Node(): Node("livox_mid360_node")
{
  //this->declare_parameter<std::string>("sensor_ip", "192.168.1.107");
  //this->declare_parameter<std::string>("host_ip", "192.168.1.5");
  this->declare_parameter<std::string>("configFilePath", "/home/drone/ros2_drone_ws/src/livox_sdk2_driver/config/MID360_config.json");
  this->declare_parameter<std::string>("pointcloud_topic", "livox/pt_cloud");
  this->declare_parameter<std::string>("imu_topic", "livox/imu");

  // Get parameters
  config_file_path_ = this->get_parameter("configFilePath").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();

  ptcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

  // Initialize the Livox-SDK2
  InitSDK();

}

LivoxMid360Node::~LivoxMid360Node()
{
  // Uninitialize the SDK
  LivoxLidarSdkUninit();
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
  if (data == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Received null data.");
    return;
  }
  // Register client
  auto* node = static_cast<LivoxMid360Node*>(client_data);
  if (node == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Client data is not a valid LivoxMid360Node.");
    return;
  }
  node->PublishPointCloudData(data);
}

void LivoxMid360Node::PublishPointCloudData(LivoxLidarEthernetPacket* data)
{  // Process point cloud data
  sensor_msgs::msg::PointCloud2 pcl_msg;

  //Modifier to describe what the fields are.
  sensor_msgs::PointCloud2Modifier modifier(pcl_msg);

  pcl_msg.header.stamp = rclcpp::Clock().now();
  pcl_msg.header.frame_id = "mid360_frame";
  pcl_msg.height = 1; // Point cloud is unorganized
  // Fill in the msg with point cloud data...
  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    pcl_msg.width = data->dot_num;
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::INT32,
      "y", 1, sensor_msgs::msg::PointField::INT32,
      "z", 1, sensor_msgs::msg::PointField::INT32,
      "intensity", 1, sensor_msgs::msg::PointField::UINT8);
    pcl_msg.is_dense = true;
    pcl_msg.point_step = 13;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);

    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");
  
    for (uint32_t i = 0; i < data->dot_num; i++) {
      *iterX = (p_point_data[i].x);
      *iterY = (p_point_data[i].y);
      *iterZ = (p_point_data[i].z);
      *iterIntensity = (p_point_data[i].reflectivity);

      // Increment the iterators
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterIntensity;
    }
  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    pcl_msg.width = data->dot_num;
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::INT16,
      "y", 1, sensor_msgs::msg::PointField::INT16,
      "z", 1, sensor_msgs::msg::PointField::INT16,
      "intensity", 1, sensor_msgs::msg::PointField::UINT8);
    pcl_msg.is_dense = true;
    pcl_msg.point_step = 7;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);
    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");
    for (uint32_t i = 0; i < data->dot_num; i++) {
      *iterX = (p_point_data[i].x);
      *iterY = (p_point_data[i].y);
      *iterZ = (p_point_data[i].z);
      *iterIntensity = (p_point_data[i].reflectivity);

      // Increment the iterators
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterIntensity;
    }
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    pcl_msg.width = data->dot_num;
    modifier.setPointCloud2Fields(4,
      "depth", 1, sensor_msgs::msg::PointField::UINT32,
      "theta", 1, sensor_msgs::msg::PointField::UINT16,
      "phi", 1, sensor_msgs::msg::PointField::UINT16,
      "reflectivity", 1, sensor_msgs::msg::PointField::UINT8);
    pcl_msg.is_dense = true;
    pcl_msg.point_step = 9;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);
    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<uint32_t> iterDepth(pcl_msg, "depth");
    sensor_msgs::PointCloud2Iterator<uint16_t> iterTheta(pcl_msg, "theta");
    sensor_msgs::PointCloud2Iterator<uint16_t> iterPhi(pcl_msg, "phi");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterReflectivity(pcl_msg, "reflectivity");
    for (uint32_t i = 0; i < data->dot_num; i++) {
      *iterDepth = (p_point_data[i].depth);
      *iterTheta = (p_point_data[i].theta);
      *iterPhi = (p_point_data[i].phi);
      *iterReflectivity = (p_point_data[i].reflectivity);

      // Increment the iterators
      ++iterDepth;
      ++iterTheta;
      ++iterPhi;
      ++iterReflectivity;
    }
  }  
  this->ptcloud_pub_->publish(pcl_msg);
  }

void ImuCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data)
{
  if (data == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Received null IMU data.");
    return;
  }
  // Register client
  auto* node = static_cast<LivoxMid360Node*>(client_data);

  if (node == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Client data is not a valid LivoxMid360Node.");
    return;
  }

  node->PublishImuData(data);
}

void LivoxMid360Node::PublishImuData(LivoxLidarEthernetPacket* data)
{
  if (data->data_type == kLivoxLidarImuData) {
    LivoxLidarImuRawPoint* imu_data = (LivoxLidarImuRawPoint*)data->data;
    // Process IMU data
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.header.frame_id = "mid360_imu_frame";
    imu_msg.angular_velocity.x = imu_data->gyro_x;
    imu_msg.angular_velocity.y = imu_data->gyro_y;
    imu_msg.angular_velocity.z = imu_data->gyro_z;
    imu_msg.linear_acceleration.x = imu_data->acc_x;
    imu_msg.linear_acceleration.y = imu_data->acc_y;
    imu_msg.linear_acceleration.z = imu_data->acc_z;

    this->imu_pub_->publish(imu_msg);
  }
}


bool LivoxMid360Node::InitSDK()
{
  // Initialize the SDK and set up the device
  if (!LivoxLidarSdkInit(config_file_path_.c_str())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Livox SDK.");
    LivoxLidarSdkUninit();
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Livox SDK initialized successfully.");

  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);

  // OPTIONAL, to get imu data via 'ImuDataCallback'
  // some lidar types DO NOT contain an imu component
  SetLivoxLidarImuDataCallback(ImuDataCallback, this);
  
  //SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  //SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  return true;
}
