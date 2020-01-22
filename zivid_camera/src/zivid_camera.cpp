#include <zivid_camera/zivid_camera.h>

namespace
{
sensor_msgs::msg::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool big_endian()
{
  return false;
}

template <class T>
void fillCommonMsgFields(T& msg, const std_msgs::msg::Header& header, std::size_t width, std::size_t height)
{
  msg.header = header;
  msg.height = static_cast<uint32_t>(height);
  msg.width = static_cast<uint32_t>(width);
  msg.is_bigendian = big_endian();
}

std::string toString(zivid_camera::CameraStatus camera_status)
{
  switch (camera_status)
  {
    case zivid_camera::CameraStatus::Connected:
      return "Connected";
    case zivid_camera::CameraStatus::Disconnected:
      return "Disconnected";
    case zivid_camera::CameraStatus::Idle:
      return "Idle";
  }
  return "N/A";
}

}  // namespace

namespace zivid_camera
{
ZividCamera::ZividCamera(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("zivid_camera", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_configure(const rclcpp_lifecycle::State& state)
{
  camera_ = zivid_.createFileCamera("/home/lars/Downloads/MiscObjects.zdf");

  auto handle_capture = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<zivid_msgs::srv::Capture::Request> request,
                               std::shared_ptr<zivid_msgs::srv::Capture::Response> response) -> void {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Capture");

    publishFrame(camera_.capture());
  };
  capture_service_ = create_service<zivid_msgs::srv::Capture>("capture", handle_capture);

  auto qos = rclcpp::SystemDefaultsQoS();
  points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", qos);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_activate(const rclcpp_lifecycle::State& state)
{
  points_publisher_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_deactivate(const rclcpp_lifecycle::State& state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_cleanup(const rclcpp_lifecycle::State& state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_shutdown(const rclcpp_lifecycle::State& state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std_msgs::msg::Header ZividCamera::makeHeader()
{
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = frame_id_;
  return header;
}

void ZividCamera::publishFrame(Zivid::Frame&& frame)
{
  const auto header = makeHeader();
  const auto point_cloud = frame.getPointCloud();

  points_publisher_->publish(std::move(makePointCloud2(header, point_cloud)));
}

sensor_msgs::msg::PointCloud2::UniquePtr ZividCamera::makePointCloud2(const std_msgs::msg::Header& header,
                                                                      const Zivid::PointCloud& point_cloud)
{
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->point_step = sizeof(Zivid::Point);
  msg->row_step = msg->point_step * msg->width;
  msg->is_dense = false;

  msg->fields.reserve(5);
  msg->fields.push_back(createPointField("x", 0, 7, 1));
  msg->fields.push_back(createPointField("y", 4, 7, 1));
  msg->fields.push_back(createPointField("z", 8, 7, 1));
  msg->fields.push_back(createPointField("c", 12, 7, 1));
  msg->fields.push_back(createPointField("rgb", 16, 7, 1));

  msg->data = std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(point_cloud.dataPtr()),
                                   reinterpret_cast<const uint8_t*>(point_cloud.dataPtr() + point_cloud.size()));

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    uint8_t* point_ptr = &(msg->data[i * sizeof(Zivid::Point)]);
    float* x_ptr = reinterpret_cast<float*>(&(point_ptr[0]));
    float* y_ptr = reinterpret_cast<float*>(&(point_ptr[4]));
    float* z_ptr = reinterpret_cast<float*>(&(point_ptr[8]));

    // Convert from mm to m
    *x_ptr *= 0.001f;
    *y_ptr *= 0.001f;
    *z_ptr *= 0.001f;
  }
  return msg;
}

}  // namespace zivid_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera::ZividCamera)