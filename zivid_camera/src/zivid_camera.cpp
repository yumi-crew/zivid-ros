// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <zivid_camera/zivid_camera.h>

#include <Zivid/HDR.h>
#include <Zivid/Firmware.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>
#include <Zivid/CaptureAssistant.h>

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
ZividCamera::ZividCamera(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("zivid_camera", options)
  , camera_status_(CameraStatus::Idle)
  , image_transport_node_(rclcpp::Node::make_shared("image_transport_node"))
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Built towards Zivid API version " << ZIVID_VERSION);
  RCLCPP_INFO_STREAM(this->get_logger(), "Running with Zivid API version " << Zivid::Version::libraryVersion());
  if (Zivid::Version::libraryVersion() != ZIVID_VERSION)
  {
    throw std::runtime_error("Zivid library mismatch! The running Zivid Core version does not match the "
                             "version this ROS driver was built towards. Hint: Try to clean and re-build your project "
                             "from scratch.");
  }

  this->declare_parameter<std::string>("serial_number", "");
  this->declare_parameter<int>("num_capture_frames", 10);
  this->declare_parameter<std::string>("frame_id", "zivid_optical_frame");
  this->declare_parameter<std::string>("file_camera_path", "");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_configure(const rclcpp_lifecycle::State& state)
{
  std::string serial_number = this->get_parameter("serial_number").as_string();
  int num_capture_frames = this->get_parameter("num_capture_frames").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();

  std::string file_camera_path = this->get_parameter("file_camera_path").as_string();
  file_camera_mode_ = !file_camera_path.empty();

  if (file_camera_mode_)
  {
    RCLCPP_INFO(this->get_logger(), "Creating file camera from file '%s'", file_camera_path.c_str());
    camera_ = zivid_.createFileCamera(file_camera_path);
  }
  else
  {
    auto cameras = zivid_.cameras();
    RCLCPP_INFO_STREAM(this->get_logger(), cameras.size() << " camera(s) found");
    if (cameras.empty())
    {
      throw std::runtime_error("No cameras found. Ensure that the camera is connected to the USB3 port on your PC.");
    }
    else if (serial_number.empty())
    {
      camera_ = [&]() {
        RCLCPP_INFO(this->get_logger(), "Selecting first available camera");
        for (auto& c : cameras)
        {
          if (c.state().isAvailable())
            return c;
        }
        throw std::runtime_error("No available cameras found. Is the camera in use by another process?");
      }();
    }
    else
    {
      if (serial_number.find(":") == 0)
      {
        serial_number = serial_number.substr(1);
      }
      camera_ = [&]() {
        RCLCPP_INFO(this->get_logger(), "Searching for camera with serial number '%s' ...", serial_number.c_str());
        for (auto& c : cameras)
        {
          if (c.serialNumber() == Zivid::SerialNumber(serial_number))
            return c;
        }
        throw std::runtime_error("No camera found with serial number '" + serial_number + "'");
      }();
    }

    if (!Zivid::Firmware::isUpToDate(camera_))
    {
      RCLCPP_INFO(this->get_logger(), "The camera firmware is not up-to-date, starting update");
      Zivid::Firmware::update(camera_, [this](double progress, const std::string& state) {
        RCLCPP_INFO(this->get_logger(), "  [%.0f%%] %s", progress, state.c_str());
      });
      RCLCPP_INFO(this->get_logger(), "Firmware update completed");
    }
  }

  camera_info_serial_number_service_ = this->create_service<zivid_interfaces::srv::CameraInfoSerialNumber>(
      "camera_info/serial_number",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request> request,
             std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response) -> void {
        (void)request_header;
        if (!enabled_)
        {
          RCLCPP_WARN(this->get_logger(), "Trying to call the 'camera_info/serial_number' service, but the service is "
                                          "not activated");
          return;
        }
        response->serial_number = camera_.serialNumber().toString();
      });

  camera_info_model_name_service_ = this->create_service<zivid_interfaces::srv::CameraInfoModelName>(
      "camera_info/model_name",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request> request,
             std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response) -> void {
        (void)request_header;
        if (!enabled_)
        {
          RCLCPP_WARN(this->get_logger(), "Trying to call the 'camera_model/model_name' service, but the service is "
                                          "not activated");
          return;
        }
        response->model_name = camera_.modelName();
      });

  capture_service_ = create_service<zivid_interfaces::srv::Capture>(
      "capture",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<zivid_interfaces::srv::Capture::Request> request,
             std::shared_ptr<zivid_interfaces::srv::Capture::Response> response) -> void {
        (void)request_header;
        if (!enabled_)
        {
          RCLCPP_WARN(this->get_logger(), "Trying to call the 'capture' service, but the service is not activated");
          return;
        }
        publishFrame(camera_.capture());
      });

  capture_2d_service_ = create_service<zivid_interfaces::srv::Capture2D>(
      "capture_2d",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<zivid_interfaces::srv::Capture2D::Request> request,
             std::shared_ptr<zivid_interfaces::srv::Capture2D::Response> response) -> void {
        (void)request_header;

        if (!enabled_)
        {
          RCLCPP_WARN(this->get_logger(), "Trying to call the 'capture_2d' service, but the service is not activated");
          return;
        }

        Zivid::Settings2D settings2D;
        auto frame2D = camera_.capture2D(settings2D);
        const auto header = makeHeader();
        auto image = frame2D.image<Zivid::RGBA8>();
        const auto camera_info = makeCameraInfo(header, image.width(), image.height(), camera_.intrinsics());
        color_image_publisher_.publish(makeColorImage(header, image), camera_info);
      });

  capture_assistant_suggest_settings_service_ = create_service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
      "capture_assistant/suggest_settings",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
             std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response) -> void {
        (void)request_header;

        if (!enabled_)
        {
          RCLCPP_WARN(this->get_logger(), "Trying to call the 'capture_assistant/suggest_settings' service, but the "
                                          "service is not activated");
          return;
        }
      });

  auto qos = rclcpp::SystemDefaultsQoS();

  color_image_publisher_ = image_transport::create_camera_publisher(image_transport_node_.get(), "color/image_color");
  depth_image_publisher_ = image_transport::create_camera_publisher(image_transport_node_.get(), "depth/image_raw");

  points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", qos);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO_STREAM(this->get_logger(), camera_);
  if (!file_camera_mode_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to camera '" << camera_.serialNumber() << "'");
    camera_.connect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Connected to camera '" << camera_.serialNumber() << "'");

  points_publisher_->on_activate();
  enabled_ = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_deactivate(const rclcpp_lifecycle::State& state)
{
  if (!file_camera_mode_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Disconnecting from camera '" << camera_.serialNumber() << "'");
    camera_.disconnect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Disconnected from camera '" << camera_.serialNumber() << "'");

  points_publisher_->on_deactivate();
  enabled_ = false;

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

sensor_msgs::msg::Image::ConstSharedPtr ZividCamera::makeColorImage(const std_msgs::msg::Header& header,
                                                                    const Zivid::PointCloud& point_cloud)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->encoding = sensor_msgs::image_encodings::RGB8;
  constexpr uint32_t bytes_per_pixel = 3U;
  msg->step = static_cast<uint32_t>(bytes_per_pixel * point_cloud.width());
  msg->data.resize(msg->step * msg->height);

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    const auto point = point_cloud(i);
    msg->data[3 * i] = point.red();
    msg->data[3 * i + 1] = point.green();
    msg->data[3 * i + 2] = point.blue();
  }
  return msg;
}

sensor_msgs::msg::Image::ConstSharedPtr ZividCamera::makeColorImage(const std_msgs::msg::Header& header,
                                                                    const Zivid::Image<Zivid::RGBA8>& image)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*msg, header, image.width(), image.height());
  msg->encoding = sensor_msgs::image_encodings::RGBA8;
  constexpr uint32_t bytes_per_pixel = 4U;
  msg->step = static_cast<uint32_t>(bytes_per_pixel * image.width());
  const auto uint8_data_ptr = reinterpret_cast<const uint8_t*>(image.dataPtr());
  msg->data = std::vector<uint8_t>(uint8_data_ptr, uint8_data_ptr + image.size() * sizeof(Zivid::RGBA8));
  return msg;
}

sensor_msgs::msg::Image::ConstSharedPtr ZividCamera::makeDepthImage(const std_msgs::msg::Header& header,
                                                                    const Zivid::PointCloud& point_cloud)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg->step = static_cast<uint32_t>(4 * point_cloud.width());
  msg->data.resize(msg->step * msg->height);

#pragma omp parallel for
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    float* image_data = reinterpret_cast<float*>(&msg->data[4 * i]);
    // Convert from mm to m
    *image_data = point_cloud(i).z * 0.001f;
  }
  return msg;
}

sensor_msgs::msg::CameraInfo::ConstSharedPtr ZividCamera::makeCameraInfo(const std_msgs::msg::Header& header,
                                                                         std::size_t width, std::size_t height,
                                                                         const Zivid::CameraIntrinsics& intrinsics)
{
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  msg->header = header;
  msg->width = static_cast<uint32_t>(width);
  msg->height = static_cast<uint32_t>(height);
  msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // k1, k2, t1, t2, k3
  const auto distortion = intrinsics.distortion();
  msg->d.resize(5);
  msg->d[0] = distortion.k1().value();
  msg->d[1] = distortion.k2().value();
  msg->d[2] = distortion.p1().value();
  msg->d[3] = distortion.p2().value();
  msg->d[4] = distortion.k3().value();

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  const auto camera_matrix = intrinsics.cameraMatrix();
  msg->k[0] = camera_matrix.fx().value();
  msg->k[2] = camera_matrix.cx().value();
  msg->k[4] = camera_matrix.fy().value();
  msg->k[5] = camera_matrix.cy().value();
  msg->k[8] = 1;

  // R (identity)
  msg->r[0] = 1;
  msg->r[4] = 1;
  msg->r[8] = 1;

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  msg->p[0] = camera_matrix.fx().value();
  msg->p[2] = camera_matrix.cx().value();
  msg->p[5] = camera_matrix.fy().value();
  msg->p[6] = camera_matrix.cy().value();
  msg->p[10] = 1;

  return msg;
}

}  // namespace zivid_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera::ZividCamera)