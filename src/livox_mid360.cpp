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
  // Removed local pcl_msg, use latest_pc_msg_ directly

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (latest_pc_msg_.data.empty()) {
    latest_pc_msg_.header.frame_id = "mid360_frame";
    latest_pc_msg_.header.stamp = rclcpp::Clock().now();
    latest_pc_msg_.height = 1; // Point cloud is unorganized
  }

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;

    if (latest_pc_msg_.data.empty()) {
      latest_pc_msg_.width = 0;
      sensor_msgs::PointCloud2Modifier modifier(latest_pc_msg_);
      modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::INT32,
        "y", 1, sensor_msgs::msg::PointField::INT32,
        "z", 1, sensor_msgs::msg::PointField::INT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8);
      latest_pc_msg_.is_dense = true;
      latest_pc_msg_.point_step = 13;
    }

    size_t old_size = latest_pc_msg_.data.size();
    latest_pc_msg_.width += data->dot_num;
    latest_pc_msg_.row_step = latest_pc_msg_.width * latest_pc_msg_.point_step;
    latest_pc_msg_.data.resize(latest_pc_msg_.row_step);

    Eigen::Quaterniond orientation = InterpolateIMUOrientation(rclcpp::Time(latest_pc_msg_.header.stamp));

    //Iterators for PointCloud msg, offset by old_size
    sensor_msgs::PointCloud2Iterator<int32_t> iterX(latest_pc_msg_, "x");
    sensor_msgs::PointCloud2Iterator<int32_t> iterY(latest_pc_msg_, "y");
    sensor_msgs::PointCloud2Iterator<int32_t> iterZ(latest_pc_msg_, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterIntensity(latest_pc_msg_, "intensity");

    iterX += old_size / latest_pc_msg_.point_step;
    iterY += old_size / latest_pc_msg_.point_step;
    iterZ += old_size / latest_pc_msg_.point_step;
    iterIntensity += old_size / latest_pc_msg_.point_step;

    for (uint32_t i = 0; i < data->dot_num; i++) {
      Eigen::Vector3d raw_point(
        p_point_data[i].x,
        p_point_data[i].y,
        p_point_data[i].z);
      Eigen::Vector3d transformed_point = orientation.inverse() * raw_point;
      *iterX = static_cast<int32_t>(transformed_point.x()); // in m
      *iterY = static_cast<int32_t>(transformed_point.y()); // in m
      *iterZ = static_cast<int32_t>(transformed_point.z()); // in m
      // Reflectivity is stored as intensity
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

    if (latest_pc_msg_.data.empty()) {
      latest_pc_msg_.width = 0;
      sensor_msgs::PointCloud2Modifier modifier(latest_pc_msg_);
      modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::INT16,
        "y", 1, sensor_msgs::msg::PointField::INT16,
        "z", 1, sensor_msgs::msg::PointField::INT16,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8);
      latest_pc_msg_.is_dense = true;
      latest_pc_msg_.point_step = 7;
    }

    size_t old_size = latest_pc_msg_.data.size();
    latest_pc_msg_.width += data->dot_num;
    latest_pc_msg_.row_step = latest_pc_msg_.width * latest_pc_msg_.point_step;
    latest_pc_msg_.data.resize(latest_pc_msg_.row_step);

    Eigen::Quaterniond orientation = InterpolateIMUOrientation(rclcpp::Time(latest_pc_msg_.header.stamp));
    //Iterators for PointCloud msg, offset by old_size
    sensor_msgs::PointCloud2Iterator<int16_t> iterX(latest_pc_msg_, "x");
    sensor_msgs::PointCloud2Iterator<int16_t> iterY(latest_pc_msg_, "y");
    sensor_msgs::PointCloud2Iterator<int16_t> iterZ(latest_pc_msg_, "z");
    sensor_msgs::PointCloud2Iterator<int8_t> iterIntensity(latest_pc_msg_, "intensity");

    iterX += old_size / latest_pc_msg_.point_step;
    iterY += old_size / latest_pc_msg_.point_step;
    iterZ += old_size / latest_pc_msg_.point_step;
    iterIntensity += old_size / latest_pc_msg_.point_step;

    for (uint32_t i = 0; i < data->dot_num; i++) {
      Eigen::Vector3d raw_point(
        p_point_data[i].x,
        p_point_data[i].y,
        p_point_data[i].z);
      Eigen::Vector3d transformed_point = orientation.inverse() * raw_point;
      *iterX = static_cast<int16_t>(transformed_point.x()); // in m
      *iterY = static_cast<int16_t>(transformed_point.y()); // in m
      *iterZ = static_cast<int16_t>(transformed_point.z()); // in m
      // Reflectivity is stored as intensity
      *iterIntensity = (p_point_data[i].reflectivity);

      // Increment the iterators
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterIntensity;
    }
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;

    if (latest_pc_msg_.data.empty()) {
      latest_pc_msg_.width = 0;
      sensor_msgs::PointCloud2Modifier modifier(latest_pc_msg_);
      modifier.setPointCloud2Fields(4,
        "depth", 1, sensor_msgs::msg::PointField::UINT32,
        "theta", 1, sensor_msgs::msg::PointField::UINT16,
        "phi", 1, sensor_msgs::msg::PointField::UINT16,
        "reflectivity", 1, sensor_msgs::msg::PointField::UINT8);
      latest_pc_msg_.is_dense = true;
      latest_pc_msg_.point_step = 9;
    }

    size_t old_size = latest_pc_msg_.data.size();
    latest_pc_msg_.width += data->dot_num;
    latest_pc_msg_.row_step = latest_pc_msg_.width * latest_pc_msg_.point_step;
    latest_pc_msg_.data.resize(latest_pc_msg_.row_step);

    //Iterators for PointCloud msg, offset by old_size
    sensor_msgs::PointCloud2Iterator<uint32_t> iterDepth(latest_pc_msg_, "depth");
    sensor_msgs::PointCloud2Iterator<uint16_t> iterTheta(latest_pc_msg_, "theta");
    sensor_msgs::PointCloud2Iterator<uint16_t> iterPhi(latest_pc_msg_, "phi");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterReflectivity(latest_pc_msg_, "reflectivity");

    iterDepth += old_size / latest_pc_msg_.point_step;
    iterTheta += old_size / latest_pc_msg_.point_step;
    iterPhi += old_size / latest_pc_msg_.point_step;
    iterReflectivity += old_size / latest_pc_msg_.point_step;

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
}

void LivoxMid360Node::PublishPointCloudData()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_pc_msg_.data.empty()) {
    ptcloud_pub_->publish(latest_pc_msg_);
    latest_pc_msg_.data.clear();
    latest_pc_msg_.width = 0;
    latest_pc_msg_.row_step = 0;
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
    imu_msg.header.stamp = rclcpp::Clock().now(); //No hardware timestamp available, use current time
    imu_msg.header.frame_id = "mid360_imu_frame";
    imu_msg.angular_velocity.x = imu_data->gyro_x;
    imu_msg.angular_velocity.y = imu_data->gyro_y;
    imu_msg.angular_velocity.z = imu_data->gyro_z;
    imu_msg.linear_acceleration.x = imu_data->acc_x;
    imu_msg.linear_acceleration.y = imu_data->acc_y;
    imu_msg.linear_acceleration.z = imu_data->acc_z;

    // Orientation is not provided by the MID360, so we set it to identity for now
    madgwick_filter_.update(imu_msg); // Update the filter with the new IMU data
    Eigen::Quaterniond orientation = madgwick_filter_.getOrientation();
    imu_msg.orientation.x = orientation.x();
    imu_msg.orientation.y = orientation.y();
    imu_msg.orientation.z = orientation.z();
    imu_msg.orientation.w = orientation.w();

    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
    // Store the IMU data in a buffer for later processing
    imu_buffer_.emplace_back(imu_msg);
    while (imu_buffer_.size() > 200) { // Limit buffer size to prevent memory overflow
      imu_buffer_.pop_front();
    }
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    latest_imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>(imu_msg);
  }
}

Eigen::Quaterniond LivoxMid360Node::InterpolateIMUOrientation(const rclcpp::Time& timestamp)
{
  // This function should implement the logic to interpolate the IMU orientation
  // based on the timestamp and the available IMU data in the buffer.
  // For now, we return an identity quaternion as a placeholder.
  std::lock_guard<std::mutex> lock(imu_buffer_mutex_); // Lock the buffer to safely access it

  if (imu_buffer_.size() < 2) {
    return Eigen::Quaterniond::Identity(); // Return identity quaternion if not enough data
  }

  for (size_t i = 0; i < imu_buffer_.size() - 1; ++i) {
    const auto& imu1 = imu_buffer_[i];
    const auto& imu2 = imu_buffer_[i + 1];
    rclcpp::Time t1(imu1.header.stamp.sec, imu1.header.stamp.nanosec, RCL_ROS_TIME);
    rclcpp::Time t2(imu2.header.stamp.sec, imu2.header.stamp.nanosec, RCL_ROS_TIME);
    if (t1 <= timestamp && t2 >= timestamp) {
      // Interpolate between imu1 and imu2
      double ratio = (timestamp - t1).seconds() / (t2 - t1).seconds();
      Eigen::Quaterniond q1(imu1.orientation.w, imu1.orientation.x, imu1.orientation.y, imu1.orientation.z);
      Eigen::Quaterniond q2(imu2.orientation.w, imu2.orientation.x, imu2.orientation.y, imu2.orientation.z);
      return q1.slerp(ratio, q2); // Spherical linear interpolation
    }
  }
  const auto& last_imu = imu_buffer_.back();
  return Eigen::Quaterniond(last_imu.orientation.w, last_imu.orientation.x, last_imu.orientation.y, last_imu.orientation.z);
  // If no suitable interpolation found, return the last known orientation
  // This is a fallback and may not be ideal for all applications.
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
