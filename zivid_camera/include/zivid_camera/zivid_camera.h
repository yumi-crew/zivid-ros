#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_transport/image_transport.h>

#include <zivid_camera/visibility_control.h>

#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>
#include <zivid_interfaces/srv/capture.hpp>
#include <zivid_interfaces/srv/capture2_d.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <zivid_interfaces/srv/is_connected.hpp>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Image.h>

namespace Zivid
{
class Settings;
}

namespace zivid_camera
{
enum class CameraStatus
{
  Idle,
  Connected,
  Disconnected
};

class ZividCamera : public rclcpp_lifecycle::LifecycleNode
{
public:
  ZIVID_CAMERA_PUBLIC
  explicit ZividCamera(const rclcpp::NodeOptions& options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state);

private:
  void publishFrame(Zivid::Frame&& frame);

  std_msgs::msg::Header makeHeader();
  sensor_msgs::msg::PointCloud2::UniquePtr makePointCloud2(const std_msgs::msg::Header& header,
                                                           const Zivid::PointCloud& point_cloud);
  sensor_msgs::msg::Image::ConstSharedPtr makeColorImage(const std_msgs::msg::Header& header,
                                                         const Zivid::PointCloud& point_cloud);
  sensor_msgs::msg::Image::ConstSharedPtr makeColorImage(const std_msgs::msg::Header& header,
                                                         const Zivid::Image<Zivid::RGBA8>& image);
  sensor_msgs::msg::Image::ConstSharedPtr makeDepthImage(const std_msgs::msg::Header& header,
                                                         const Zivid::PointCloud& point_cloud);

  sensor_msgs::msg::CameraInfo::ConstSharedPtr makeCameraInfo(const std_msgs::msg::Header& header, std::size_t width,
                                                              std::size_t height,
                                                              const Zivid::CameraIntrinsics& intrinsics);

  CameraStatus camera_status_;

  rclcpp::Service<zivid_interfaces::srv::CameraInfoSerialNumber>::SharedPtr camera_info_serial_number_service_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoModelName>::SharedPtr camera_info_model_name_service_;
  rclcpp::Service<zivid_interfaces::srv::Capture>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::Capture2D>::SharedPtr capture_2d_service_;
  rclcpp::Service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>::SharedPtr
      capture_assistant_suggest_settings_service_;
  rclcpp::Service<zivid_interfaces::srv::IsConnected>::SharedPtr is_connected_service_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  image_transport::CameraPublisher color_image_publisher_;
  image_transport::CameraPublisher depth_image_publisher_;
  // image_transport::ImageTransport image_transport_;
  rclcpp::Node::SharedPtr image_transport_node_;

  Zivid::Application zivid_;
  Zivid::Camera camera_;

  std::string frame_id_;
};

}  // namespace zivid_camera