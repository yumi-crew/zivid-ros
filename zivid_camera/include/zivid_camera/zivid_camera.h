#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>

#include <zivid_camera/visibility_control.h>

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

private:
  Zivid::Application zivid_;
  Zivid::Camera camera_;
};

}  // namespace zivid_camera