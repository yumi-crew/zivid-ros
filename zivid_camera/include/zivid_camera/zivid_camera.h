#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>

#include <zivid_camera/visibility_control.h>

#include <zivid_msgs/srv/capture.hpp>

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

  // std_msgs::Header makeHeader();
  // sensor_msgs::PointCloud2ConstPtr makePointCloud2(const std_msgs::Header& header,
  //                                                  const Zivid::PointCloud& point_cloud);

  rclcpp::Service<zivid_msgs::srv::Capture>::SharedPtr capture_service_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;

  Zivid::Application zivid_;
  Zivid::Camera camera_;
};

}  // namespace zivid_camera