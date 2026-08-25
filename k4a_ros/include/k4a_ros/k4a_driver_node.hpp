// Copyright 2026 Ziqi CHAI
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <k4a/k4a.h>

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/camera_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

namespace k4a_ros
{

class K4ADriverNode final : public rclcpp::Node
{
public:
  explicit K4ADriverNode(const rclcpp::NodeOptions & options);
  ~K4ADriverNode() override;

  K4ADriverNode(const K4ADriverNode &) = delete;
  K4ADriverNode & operator=(const K4ADriverNode &) = delete;

private:
  void declareAndReadParameters();
  void openDevice();
  void configureDevice();
  void allocateTransformationImages();
  void startDevice();
  void stopDevice() noexcept;

  void captureLoop();
  void imuLoop();
  void publishCapture(k4a_capture_t capture);
  void publishColor(k4a_image_t image, const rclcpp::Time & stamp);
  void publishDepth(k4a_image_t image, const rclcpp::Time & stamp);
  void publishIr(k4a_image_t image, const rclcpp::Time & stamp);
  void publishAlignedImages(
    k4a_image_t depth_image, k4a_image_t color_image, const rclcpp::Time & stamp);
  void publishPointCloud(
    k4a_image_t depth_image, k4a_image_t color_image, const rclcpp::Time & stamp);

  sensor_msgs::msg::CameraInfo cameraInfo(
    const k4a_calibration_camera_t & camera, const std::string & frame_id) const;
  sensor_msgs::msg::Image imageMessage(
    k4a_image_t image, const std::string & encoding, const std::string & frame_id,
    const rclcpp::Time & stamp) const;
  void publishStaticTransforms();
  rclcpp::Time deviceTime(uint64_t device_timestamp_usec);
  std::string readSerial(k4a_device_t device) const;
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper & status);

  int device_id_{0};
  std::string serial_number_;
  bool color_enabled_{true};
  bool depth_enabled_{true};
  bool ir_enabled_{true};
  bool imu_enabled_{true};
  bool point_cloud_enabled_{true};
  bool aligned_depth_enabled_{true};
  bool aligned_color_enabled_{true};
  bool publish_tf_{true};
  int capture_timeout_ms_{1000};
  std::string color_resolution_name_{"720p"};
  std::string depth_mode_name_{"nfov_unbinned"};
  int fps_{30};

  std::string depth_frame_id_{"k4a_depth_camera_optical_frame"};
  std::string color_frame_id_{"k4a_color_camera_optical_frame"};
  std::string imu_frame_id_{"k4a_imu_frame"};

  k4a_device_t device_{nullptr};
  k4a_transformation_t transformation_{nullptr};
  k4a_image_t point_cloud_image_{nullptr};
  k4a_image_t color_to_depth_image_{nullptr};
  k4a_image_t depth_to_color_image_{nullptr};
  k4a_device_configuration_t device_config_{K4A_DEVICE_CONFIG_INIT_DISABLE_ALL};
  k4a_calibration_t calibration_{};
  std::array<float, 9> accel_to_gyro_rotation_{};
  k4a_color_resolution_t calibration_color_resolution_{K4A_COLOR_RESOLUTION_720P};
  k4a_depth_mode_t calibration_depth_mode_{K4A_DEPTH_MODE_NFOV_UNBINNED};
  sensor_msgs::msg::CameraInfo color_info_;
  sensor_msgs::msg::CameraInfo depth_info_;

  image_transport::CameraPublisher color_publisher_;
  image_transport::CameraPublisher depth_publisher_;
  image_transport::CameraPublisher ir_publisher_;
  image_transport::CameraPublisher aligned_depth_publisher_;
  image_transport::CameraPublisher aligned_color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  diagnostic_updater::Updater diagnostic_updater_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::thread imu_thread_;
  std::atomic<uint64_t> capture_count_{0};
  std::atomic<uint64_t> capture_timeout_count_{0};
  std::atomic<uint64_t> imu_count_{0};
  std::atomic<double> temperature_c_{0.0};

  std::mutex time_mutex_;
  bool time_initialized_{false};
  uint64_t first_device_timestamp_usec_{0};
  rclcpp::Time first_ros_time_{0, 0, RCL_SYSTEM_TIME};
};

}  // namespace k4a_ros
