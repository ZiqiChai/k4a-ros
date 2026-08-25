// Copyright 2026 Ziqi CHAI
// SPDX-License-Identifier: BSD-3-Clause

#include "k4a_ros/k4a_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace k4a_ros
{
namespace
{

constexpr auto kCaptureTimeout = K4A_WAIT_RESULT_TIMEOUT;
constexpr auto kCaptureSucceeded = K4A_WAIT_RESULT_SUCCEEDED;
constexpr auto kCaptureFailed = K4A_WAIT_RESULT_FAILED;

k4a_color_resolution_t parseColorResolution(const std::string & value)
{
  if (value == "720p") {return K4A_COLOR_RESOLUTION_720P;}
  if (value == "1080p") {return K4A_COLOR_RESOLUTION_1080P;}
  if (value == "1440p") {return K4A_COLOR_RESOLUTION_1440P;}
  if (value == "1536p") {return K4A_COLOR_RESOLUTION_1536P;}
  if (value == "2160p") {return K4A_COLOR_RESOLUTION_2160P;}
  if (value == "3072p") {return K4A_COLOR_RESOLUTION_3072P;}
  throw std::invalid_argument("Unsupported color_resolution: " + value);
}

k4a_depth_mode_t parseDepthMode(const std::string & value)
{
  if (value == "nfov_2x2_binned") {return K4A_DEPTH_MODE_NFOV_2X2BINNED;}
  if (value == "nfov_unbinned") {return K4A_DEPTH_MODE_NFOV_UNBINNED;}
  if (value == "wfov_2x2_binned") {return K4A_DEPTH_MODE_WFOV_2X2BINNED;}
  if (value == "wfov_unbinned") {return K4A_DEPTH_MODE_WFOV_UNBINNED;}
  if (value == "passive_ir") {return K4A_DEPTH_MODE_PASSIVE_IR;}
  throw std::invalid_argument("Unsupported depth_mode: " + value);
}

k4a_fps_t parseFps(int value)
{
  if (value == 5) {return K4A_FRAMES_PER_SECOND_5;}
  if (value == 15) {return K4A_FRAMES_PER_SECOND_15;}
  if (value == 30) {return K4A_FRAMES_PER_SECOND_30;}
  throw std::invalid_argument("fps must be one of: 5, 15, 30");
}

geometry_msgs::msg::TransformStamped transformMessage(
  const k4a_calibration_extrinsics_t & extrinsics, const std::string & parent,
  const std::string & child, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  message.transform.translation.x = extrinsics.translation[0] / 1000.0;
  message.transform.translation.y = extrinsics.translation[1] / 1000.0;
  message.transform.translation.z = extrinsics.translation[2] / 1000.0;

  const auto & r = extrinsics.rotation;
  tf2::Matrix3x3 matrix(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
  tf2::Quaternion quaternion;
  matrix.getRotation(quaternion);
  quaternion.normalize();
  message.transform.rotation.x = quaternion.x();
  message.transform.rotation.y = quaternion.y();
  message.transform.rotation.z = quaternion.z();
  message.transform.rotation.w = quaternion.w();
  return message;
}

}  // namespace

K4ADriverNode::K4ADriverNode(const rclcpp::NodeOptions & options)
: Node("k4a_driver", options),
  diagnostic_updater_(this)
{
  declareAndReadParameters();

  const auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  color_publisher_ = image_transport::create_camera_publisher(this, "color/image_raw", sensor_qos);
  depth_publisher_ = image_transport::create_camera_publisher(this, "depth/image_raw", sensor_qos);
  ir_publisher_ = image_transport::create_camera_publisher(this, "ir/image_raw", sensor_qos);
  aligned_depth_publisher_ =
    image_transport::create_camera_publisher(this, "aligned_depth_to_color/image_raw", sensor_qos);
  aligned_color_publisher_ =
    image_transport::create_camera_publisher(this, "aligned_color_to_depth/image_raw", sensor_qos);
  point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS());
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
  temperature_publisher_ = create_publisher<sensor_msgs::msg::Temperature>(
    "imu_temperature", rclcpp::SensorDataQoS());

  diagnostic_updater_.setHardwareID("uninitialized");
  diagnostic_updater_.add("Azure Kinect DK", this, &K4ADriverNode::diagnosticStatus);

  try {
    openDevice();
    configureDevice();
    allocateTransformationImages();
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      publishStaticTransforms();
    }
    startDevice();
  } catch (...) {
    stopDevice();
    throw;
  }
}

K4ADriverNode::~K4ADriverNode()
{
  stopDevice();
}

void K4ADriverNode::declareAndReadParameters()
{
  device_id_ = declare_parameter<int>("device_id", 0);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  color_enabled_ = declare_parameter<bool>("color_enabled", true);
  depth_enabled_ = declare_parameter<bool>("depth_enabled", true);
  ir_enabled_ = declare_parameter<bool>("ir_enabled", true);
  imu_enabled_ = declare_parameter<bool>("imu_enabled", true);
  point_cloud_enabled_ = declare_parameter<bool>("point_cloud_enabled", true);
  aligned_depth_enabled_ = declare_parameter<bool>("aligned_depth_enabled", true);
  aligned_color_enabled_ = declare_parameter<bool>("aligned_color_enabled", true);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  capture_timeout_ms_ = declare_parameter<int>("capture_timeout_ms", 1000);
  color_resolution_name_ = declare_parameter<std::string>("color_resolution", "720p");
  depth_mode_name_ = declare_parameter<std::string>("depth_mode", "nfov_unbinned");
  fps_ = declare_parameter<int>("fps", 30);
  depth_frame_id_ = declare_parameter<std::string>(
    "depth_frame_id", "k4a_depth_camera_optical_frame");
  color_frame_id_ = declare_parameter<std::string>(
    "color_frame_id", "k4a_color_camera_optical_frame");
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "k4a_imu_frame");

  if (device_id_ < 0 || capture_timeout_ms_ <= 0) {
    throw std::invalid_argument("device_id must be >= 0 and capture_timeout_ms must be > 0");
  }
  if (point_cloud_enabled_ && !depth_enabled_) {
    throw std::invalid_argument("point_cloud_enabled requires depth_enabled");
  }
  if ((aligned_depth_enabled_ || aligned_color_enabled_) && !(color_enabled_ && depth_enabled_)) {
    throw std::invalid_argument("aligned images require color_enabled and depth_enabled");
  }
  if (!(color_enabled_ || depth_enabled_ || ir_enabled_ || imu_enabled_)) {
    throw std::invalid_argument("At least one sensor must be enabled");
  }
}

void K4ADriverNode::openDevice()
{
  const uint32_t count = k4a_device_get_installed_count();
  if (count == 0) {
    throw std::runtime_error("No Azure Kinect DK device detected");
  }

  if (!serial_number_.empty()) {
    for (uint32_t index = 0; index < count; ++index) {
      k4a_device_t candidate = nullptr;
      if (k4a_device_open(index, &candidate) != K4A_RESULT_SUCCEEDED) {
        continue;
      }
      if (readSerial(candidate) == serial_number_) {
        device_ = candidate;
        device_id_ = static_cast<int>(index);
        break;
      }
      k4a_device_close(candidate);
    }
    if (device_ == nullptr) {
      throw std::runtime_error("Azure Kinect serial number not found: " + serial_number_);
    }
  } else {
    if (static_cast<uint32_t>(device_id_) >= count ||
      k4a_device_open(static_cast<uint32_t>(device_id_), &device_) != K4A_RESULT_SUCCEEDED)
    {
      throw std::runtime_error(
              "Failed to open Azure Kinect device index " +
              std::to_string(device_id_));
    }
    serial_number_ = readSerial(device_);
  }

  diagnostic_updater_.setHardwareID(serial_number_);
  RCLCPP_INFO(
    get_logger(), "Opened Azure Kinect DK index=%d serial=%s", device_id_, serial_number_.c_str());
}

std::string K4ADriverNode::readSerial(k4a_device_t device) const
{
  size_t size = 0;
  if (k4a_device_get_serialnum(device, nullptr, &size) != K4A_BUFFER_RESULT_TOO_SMALL) {
    return "unknown";
  }
  std::vector<char> serial(size);
  if (k4a_device_get_serialnum(device, serial.data(), &size) != K4A_BUFFER_RESULT_SUCCEEDED) {
    return "unknown";
  }
  return std::string(serial.data());
}

void K4ADriverNode::configureDevice()
{
  calibration_color_resolution_ = parseColorResolution(color_resolution_name_);
  calibration_depth_mode_ = parseDepthMode(depth_mode_name_);

  device_config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  device_config_.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  device_config_.color_resolution =
    color_enabled_ ? calibration_color_resolution_ : K4A_COLOR_RESOLUTION_OFF;
  device_config_.depth_mode =
    (depth_enabled_ || ir_enabled_) ? calibration_depth_mode_ : K4A_DEPTH_MODE_OFF;
  device_config_.camera_fps = parseFps(fps_);
  device_config_.synchronized_images_only = color_enabled_ && depth_enabled_;
  device_config_.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

  if (k4a_device_get_calibration(
      device_, calibration_depth_mode_, calibration_color_resolution_, &calibration_) !=
    K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to read Azure Kinect factory calibration");
  }

  depth_info_ = cameraInfo(calibration_.depth_camera_calibration, depth_frame_id_);
  color_info_ = cameraInfo(calibration_.color_camera_calibration, color_frame_id_);
  std::copy_n(
    calibration_.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_GYRO].rotation,
    accel_to_gyro_rotation_.size(), accel_to_gyro_rotation_.begin());
  transformation_ = k4a_transformation_create(&calibration_);
  if (transformation_ == nullptr) {
    throw std::runtime_error("Failed to create Azure Kinect calibration transformation");
  }
}

void K4ADriverNode::allocateTransformationImages()
{
  const int depth_width = calibration_.depth_camera_calibration.resolution_width;
  const int depth_height = calibration_.depth_camera_calibration.resolution_height;
  const int color_width = calibration_.color_camera_calibration.resolution_width;
  const int color_height = calibration_.color_camera_calibration.resolution_height;
  // A colored cloud must use color-camera visibility. Reprojecting color into
  // depth geometry can copy foreground pixels onto background surfaces at
  // occlusion boundaries because the two cameras have different viewpoints.
  const bool color_point_cloud = point_cloud_enabled_ && color_enabled_;
  const int point_cloud_width = color_point_cloud ? color_width : depth_width;
  const int point_cloud_height = color_point_cloud ? color_height : depth_height;

  if (point_cloud_enabled_ &&
    k4a_image_create(
      K4A_IMAGE_FORMAT_CUSTOM, point_cloud_width, point_cloud_height,
      point_cloud_width * static_cast<int>(3 * sizeof(int16_t)), &point_cloud_image_) !=
    K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to allocate point-cloud transformation image");
  }
  if (aligned_color_enabled_ && color_enabled_ &&
    k4a_image_create(
      K4A_IMAGE_FORMAT_COLOR_BGRA32, depth_width, depth_height, depth_width * 4,
      &color_to_depth_image_) != K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to allocate color-to-depth image");
  }
  if ((aligned_depth_enabled_ || color_point_cloud) &&
    k4a_image_create(
      K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height,
      color_width * static_cast<int>(sizeof(uint16_t)), &depth_to_color_image_) !=
    K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to allocate depth-to-color image");
  }
}

void K4ADriverNode::startDevice()
{
  const bool cameras_enabled = color_enabled_ || depth_enabled_ || ir_enabled_;
  if (cameras_enabled &&
    k4a_device_start_cameras(device_, &device_config_) != K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to start Azure Kinect cameras");
  }
  if (imu_enabled_ && k4a_device_start_imu(device_) != K4A_RESULT_SUCCEEDED) {
    if (cameras_enabled) {k4a_device_stop_cameras(device_);}
    throw std::runtime_error("Failed to start Azure Kinect IMU");
  }

  running_.store(true);
  if (cameras_enabled) {
    capture_thread_ = std::thread(&K4ADriverNode::captureLoop, this);
  }
  if (imu_enabled_) {
    imu_thread_ = std::thread(&K4ADriverNode::imuLoop, this);
  }
  RCLCPP_INFO(
    get_logger(), "Streaming started: color=%s depth=%s ir=%s imu=%s %s/%s@%dHz",
    color_enabled_ ? "on" : "off", depth_enabled_ ? "on" : "off",
    ir_enabled_ ? "on" : "off", imu_enabled_ ? "on" : "off",
    color_resolution_name_.c_str(), depth_mode_name_.c_str(), fps_);
}

void K4ADriverNode::stopDevice() noexcept
{
  running_.store(false);
  if (capture_thread_.joinable()) {capture_thread_.join();}
  if (imu_thread_.joinable()) {imu_thread_.join();}
  if (device_ != nullptr) {
    if (imu_enabled_) {k4a_device_stop_imu(device_);}
    if (color_enabled_ || depth_enabled_ || ir_enabled_) {k4a_device_stop_cameras(device_);}
  }
  if (point_cloud_image_ != nullptr) {
    k4a_image_release(point_cloud_image_);
    point_cloud_image_ = nullptr;
  }
  if (color_to_depth_image_ != nullptr) {
    k4a_image_release(color_to_depth_image_);
    color_to_depth_image_ = nullptr;
  }
  if (depth_to_color_image_ != nullptr) {
    k4a_image_release(depth_to_color_image_);
    depth_to_color_image_ = nullptr;
  }
  if (transformation_ != nullptr) {
    k4a_transformation_destroy(transformation_);
    transformation_ = nullptr;
  }
  if (device_ != nullptr) {
    k4a_device_close(device_);
    device_ = nullptr;
  }
}

void K4ADriverNode::captureLoop()
{
  while (running_.load() && rclcpp::ok()) {
    k4a_capture_t capture = nullptr;
    const auto result = k4a_device_get_capture(device_, &capture, capture_timeout_ms_);
    if (result == kCaptureSucceeded) {
      publishCapture(capture);
      k4a_capture_release(capture);
      ++capture_count_;
    } else if (result == kCaptureTimeout) {
      ++capture_timeout_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Timed out waiting for Azure Kinect capture");
    } else if (result == kCaptureFailed) {
      RCLCPP_ERROR(get_logger(), "Azure Kinect capture failed; capture thread stopped");
      running_.store(false);
    }
  }
}

void K4ADriverNode::publishCapture(k4a_capture_t capture)
{
  k4a_image_t depth = k4a_capture_get_depth_image(capture);
  k4a_image_t color = k4a_capture_get_color_image(capture);
  k4a_image_t ir = k4a_capture_get_ir_image(capture);

  uint64_t timestamp_usec = 0;
  if (depth != nullptr) {
    timestamp_usec = k4a_image_get_device_timestamp_usec(depth);
  } else if (color != nullptr) {
    timestamp_usec = k4a_image_get_device_timestamp_usec(color);
  } else if (ir != nullptr) {timestamp_usec = k4a_image_get_device_timestamp_usec(ir);}
  const rclcpp::Time stamp = deviceTime(timestamp_usec);

  if (depth_enabled_ && depth != nullptr) {publishDepth(depth, stamp);}
  if (color_enabled_ && color != nullptr) {publishColor(color, stamp);}
  if (ir_enabled_ && ir != nullptr) {publishIr(ir, stamp);}
  if (depth != nullptr && color != nullptr) {
    const bool depth_in_color_ready = publishAlignedImages(depth, color, stamp);
    if (point_cloud_enabled_) {publishPointCloud(depth, color, depth_in_color_ready, stamp);}
  } else if (depth != nullptr && point_cloud_enabled_) {
    publishPointCloud(depth, nullptr, false, stamp);
  }

  if (depth != nullptr) {k4a_image_release(depth);}
  if (color != nullptr) {k4a_image_release(color);}
  if (ir != nullptr) {k4a_image_release(ir);}
}

sensor_msgs::msg::Image K4ADriverNode::imageMessage(
  k4a_image_t image, const std::string & encoding, const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::Image message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.height = static_cast<uint32_t>(k4a_image_get_height_pixels(image));
  message.width = static_cast<uint32_t>(k4a_image_get_width_pixels(image));
  message.encoding = encoding;
  message.is_bigendian = false;
  message.step = static_cast<uint32_t>(k4a_image_get_stride_bytes(image));
  const size_t size = static_cast<size_t>(message.step) * message.height;
  const uint8_t * buffer = k4a_image_get_buffer(image);
  message.data.assign(buffer, buffer + size);
  return message;
}

void K4ADriverNode::publishColor(k4a_image_t image, const rclcpp::Time & stamp)
{
  auto message = imageMessage(image, sensor_msgs::image_encodings::BGRA8, color_frame_id_, stamp);
  auto info = color_info_;
  info.header.stamp = stamp;
  color_publisher_.publish(message, info);
}

void K4ADriverNode::publishDepth(k4a_image_t image, const rclcpp::Time & stamp)
{
  auto message = imageMessage(
    image, sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_id_,
    stamp);
  auto info = depth_info_;
  info.header.stamp = stamp;
  depth_publisher_.publish(message, info);
}

void K4ADriverNode::publishIr(k4a_image_t image, const rclcpp::Time & stamp)
{
  auto message = imageMessage(image, sensor_msgs::image_encodings::MONO16, depth_frame_id_, stamp);
  auto info = depth_info_;
  info.header.stamp = stamp;
  ir_publisher_.publish(message, info);
}

bool K4ADriverNode::publishAlignedImages(
  k4a_image_t depth_image, k4a_image_t color_image, const rclcpp::Time & stamp)
{
  if (aligned_color_enabled_ && color_to_depth_image_ != nullptr &&
    k4a_transformation_color_image_to_depth_camera(
      transformation_, depth_image, color_image, color_to_depth_image_) == K4A_RESULT_SUCCEEDED)
  {
    auto message = imageMessage(
      color_to_depth_image_, sensor_msgs::image_encodings::BGRA8, depth_frame_id_, stamp);
    auto info = depth_info_;
    info.header.stamp = stamp;
    aligned_color_publisher_.publish(message, info);
  }

  bool depth_in_color_ready = false;
  if (depth_to_color_image_ != nullptr) {
    depth_in_color_ready = k4a_transformation_depth_image_to_color_camera(
      transformation_, depth_image, depth_to_color_image_) == K4A_RESULT_SUCCEEDED;
    if (aligned_depth_enabled_ && depth_in_color_ready) {
      auto message = imageMessage(
        depth_to_color_image_, sensor_msgs::image_encodings::TYPE_16UC1, color_frame_id_, stamp);
      auto info = color_info_;
      info.header.stamp = stamp;
      aligned_depth_publisher_.publish(message, info);
    }
  }
  return depth_in_color_ready;
}

void K4ADriverNode::publishPointCloud(
  k4a_image_t depth_image, k4a_image_t color_image, bool depth_in_color_ready,
  const rclcpp::Time & stamp)
{
  const bool color_geometry = color_enabled_;
  if (color_geometry &&
    (color_image == nullptr || depth_to_color_image_ == nullptr || !depth_in_color_ready))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Color-space point cloud skipped because depth-to-color conversion failed");
    return;
  }

  // Transform depth into the color camera first, then pair each resulting XYZ
  // sample with the raw color pixel at the same coordinate.
  const k4a_image_t point_cloud_depth =
    color_geometry ? depth_to_color_image_ : depth_image;
  const k4a_calibration_type_t point_cloud_camera =
    color_geometry ? K4A_CALIBRATION_TYPE_COLOR : K4A_CALIBRATION_TYPE_DEPTH;
  if (k4a_transformation_depth_image_to_point_cloud(
      transformation_, point_cloud_depth, point_cloud_camera, point_cloud_image_) !=
    K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Point-cloud conversion failed");
    return;
  }

  const int width = k4a_image_get_width_pixels(point_cloud_depth);
  const int height = k4a_image_get_height_pixels(point_cloud_depth);
  const auto * xyz = reinterpret_cast<const int16_t *>(k4a_image_get_buffer(point_cloud_image_));
  const int xyz_stride = k4a_image_get_stride_bytes(point_cloud_image_) /
    static_cast<int>(sizeof(int16_t));
  const auto * bgra = color_geometry ? k4a_image_get_buffer(color_image) : nullptr;
  const int bgra_stride = color_geometry ? k4a_image_get_stride_bytes(color_image) : 0;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = color_geometry ? color_frame_id_ : depth_frame_id_;
  cloud.height = static_cast<uint32_t>(height);
  cloud.width = static_cast<uint32_t>(width);
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> r_it(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> g_it(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> b_it(cloud, "b");
  const float nan = std::numeric_limits<float>::quiet_NaN();

  for (int row = 0; row < height; ++row) {
    const int16_t * xyz_row = xyz + row * xyz_stride;
    const uint8_t * bgra_row = bgra == nullptr ? nullptr : bgra + row * bgra_stride;
    for (int column = 0; column < width;
      ++column, ++x_it, ++y_it, ++z_it, ++r_it, ++g_it, ++b_it)
    {
      const int16_t * point = xyz_row + column * 3;
      const int16_t z_mm = point[2];
      if (z_mm == 0) {
        *x_it = nan;
        *y_it = nan;
        *z_it = nan;
      } else {
        *x_it = point[0] / 1000.0F;
        *y_it = point[1] / 1000.0F;
        *z_it = z_mm / 1000.0F;
      }
      if (bgra_row != nullptr) {
        const uint8_t * color = bgra_row + column * 4;
        *b_it = color[0];
        *g_it = color[1];
        *r_it = color[2];
      } else {
        *r_it = *g_it = *b_it = 0;
      }
    }
  }
  point_cloud_publisher_->publish(std::move(cloud));
}

void K4ADriverNode::imuLoop()
{
  while (running_.load() && rclcpp::ok()) {
    k4a_imu_sample_t sample{};
    const auto result = k4a_device_get_imu_sample(device_, &sample, 100);
    if (result == kCaptureSucceeded) {
      const rclcpp::Time stamp = deviceTime(sample.gyro_timestamp_usec);
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = imu_frame_id_;
      imu.orientation_covariance[0] = -1.0;
      imu.angular_velocity.x = sample.gyro_sample.xyz.x;
      imu.angular_velocity.y = sample.gyro_sample.xyz.y;
      imu.angular_velocity.z = sample.gyro_sample.xyz.z;
      const auto & rotation = accel_to_gyro_rotation_;
      const auto & acceleration = sample.acc_sample.xyz;
      imu.linear_acceleration.x =
        rotation[0] * acceleration.x + rotation[1] * acceleration.y +
        rotation[2] * acceleration.z;
      imu.linear_acceleration.y =
        rotation[3] * acceleration.x + rotation[4] * acceleration.y +
        rotation[5] * acceleration.z;
      imu.linear_acceleration.z =
        rotation[6] * acceleration.x + rotation[7] * acceleration.y +
        rotation[8] * acceleration.z;
      imu_publisher_->publish(std::move(imu));

      sensor_msgs::msg::Temperature temperature;
      temperature.header.stamp = stamp;
      temperature.header.frame_id = imu_frame_id_;
      temperature.temperature = sample.temperature;
      temperature.variance = 0.0;
      temperature_publisher_->publish(temperature);
      temperature_c_.store(sample.temperature);
      ++imu_count_;
    } else if (result == kCaptureFailed) {
      RCLCPP_ERROR(get_logger(), "Azure Kinect IMU read failed; IMU thread stopped");
      running_.store(false);
    }
  }
}

sensor_msgs::msg::CameraInfo K4ADriverNode::cameraInfo(
  const k4a_calibration_camera_t & camera, const std::string & frame_id) const
{
  const auto & p = camera.intrinsics.parameters.param;
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(camera.resolution_width);
  info.height = static_cast<uint32_t>(camera.resolution_height);
  info.distortion_model = "rational_polynomial";
  info.d = {p.k1, p.k2, p.p1, p.p2, p.k3, p.k4, p.k5, p.k6};
  info.k = {p.fx, 0.0, p.cx, 0.0, p.fy, p.cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {p.fx, 0.0, p.cx, 0.0, 0.0, p.fy, p.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

void K4ADriverNode::publishStaticTransforms()
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  const auto stamp = now();
  transforms.push_back(
    transformMessage(
      calibration_.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR],
      depth_frame_id_, color_frame_id_, stamp));
  transforms.push_back(
    transformMessage(
      calibration_.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_GYRO],
      depth_frame_id_, imu_frame_id_, stamp));
  tf_broadcaster_->sendTransform(transforms);
}

rclcpp::Time K4ADriverNode::deviceTime(uint64_t device_timestamp_usec)
{
  std::lock_guard<std::mutex> lock(time_mutex_);
  if (!time_initialized_) {
    first_device_timestamp_usec_ = device_timestamp_usec;
    first_ros_time_ = now();
    time_initialized_ = true;
  }
  const int64_t delta_usec = static_cast<int64_t>(device_timestamp_usec) -
    static_cast<int64_t>(first_device_timestamp_usec_);
  return rclcpp::Time(
    first_ros_time_.nanoseconds() + delta_usec * 1000LL, first_ros_time_.get_clock_type());
}

void K4ADriverNode::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  if (device_ == nullptr) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device is closed");
  } else if (!running_.load()) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Streaming stopped unexpectedly");
  } else if (capture_timeout_count_.load() > 0 && capture_count_.load() == 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No camera capture received");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
  }
  status.add("serial_number", serial_number_);
  status.add("device_id", device_id_);
  status.add("captures", capture_count_.load());
  status.add("capture_timeouts", capture_timeout_count_.load());
  status.add("imu_samples", imu_count_.load());
  status.add("temperature_c", temperature_c_.load());
  status.add("sdk_version", "1.4.2");
}

}  // namespace k4a_ros

RCLCPP_COMPONENTS_REGISTER_NODE(k4a_ros::K4ADriverNode)
