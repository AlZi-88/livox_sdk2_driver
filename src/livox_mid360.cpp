#include "livox_sdk2_driver/livox_mid360.hpp"

LivoxMid360Node::LivoxMid360Node(): Node("livox_mid360_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting LivoxMid360Node setup");
  //this->declare_parameter<std::string>("sensor_ip", "192.168.1.107");
  //this->declare_parameter<std::string>("host_ip", "192.168.1.5");
  this->declare_parameter<std::string>("configFilePath", "/home/drone/ros2_drone_ws/src/livox_sdk2_driver/config/MID360_config.json");
  this->declare_parameter<std::string>("pointcloud_topic", "livox/pt_cloud");
  this->declare_parameter<std::string>("imu_topic", "livox/imu");
  this->declare_parameter("ptcloud_publish_rate", 10.0); //Hz
  this->declare_parameter("imu_publish_rate", 50.0); //Hz


  // Get parameters
  config_file_path_ = this->get_parameter("configFilePath").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  pc_freq_ = this->get_parameter("ptcloud_publish_rate").as_double();
  imu_freq_ = this->get_parameter("imu_publish_rate").as_double();

  pt_cloud_data_error_ = false;
  imu_data_error_ = false;

  ptcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

  pc_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / pc_freq_)),
    std::bind(&LivoxMid360Node::PublishPointCloudData, this));
  imu_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / imu_freq_)),
    std::bind(&LivoxMid360Node::PublishImuData, this));

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
    // Register client
  auto* node = static_cast<LivoxMid360Node*>(client_data);
  if (node == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Client data is not a valid LivoxMid360Node.");
    return;
  }

  if (data == nullptr) {
    if (!node->IsPointCloudDataError()) {
      // Log an error message only once to avoid flooding the logs
      RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Received null data. This may indicate a connection issue or misconfiguration.");
      node->SetPointCloudDataError(true); // Avoid flooding the logs with the same error
    }
    return;
  }
  node->ConvertToPointCloud2(data);
  node->SetPointCloudDataError(false); // Reset error flag if data is valid
}

void LivoxMid360Node::ConvertToPointCloud2(LivoxLidarEthernetPacket* data)
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
    sensor_msgs::PointCloud2Iterator<int32_t> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<int32_t> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<int32_t> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterIntensity(pcl_msg, "intensity");
  
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
    sensor_msgs::PointCloud2Iterator<int16_t> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<int16_t> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<int16_t> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<int8_t> iterIntensity(pcl_msg, "intensity");
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
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_pc_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>(pcl_msg);
  }

void LivoxMid360Node::PublishPointCloudData()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (latest_pc_msg_) {
    ptcloud_pub_->publish(*latest_pc_msg_);
    latest_pc_msg_.reset(); // Reset the pointer after publishing
  }
  else {
    RCLCPP_WARN(this->get_logger(), "No point cloud data available to publish.");
  }
}
void ImuCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data)
{
  // Register client
  auto* node = static_cast<LivoxMid360Node*>(client_data);

  if (node == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Client data is not a valid LivoxMid360Node.");
    return;
  }

  if (data == nullptr) {
    if (!node->IsImuDataError()) {
      // Log an error message only once to avoid flooding the logs
      RCLCPP_ERROR(rclcpp::get_logger("livox_mid360"), "Received null IMU data.");
      node->SetImuDataError(true); // Avoid flooding the logs with the same error
    }
    return;
  }

  node->ConvertToIMUData(data);
  node->SetImuDataError(false); // Reset error flag if data is valid
}

void LivoxMid360Node::ConvertToIMUData(LivoxLidarEthernetPacket* data)
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

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>(imu_msg);
  }
}

void LivoxMid360Node::PublishImuData()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (latest_imu_msg_) {
    imu_pub_->publish(*latest_imu_msg_);
    latest_imu_msg_.reset(); // Reset the pointer after publishing
  }
  else {
    RCLCPP_WARN(this->get_logger(), "No IMU data available to publish.");
  }
}

bool LivoxMid360Node::InitSDK()
{
  RCLCPP_INFO(this->get_logger(), "Initializing Livox SDK...");
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
  SetLivoxLidarImuDataCallback(ImuCallback, this);
  
  //SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  //SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  return true;
}
