// Copyright (c) 2020 Norwegian University of Science and Technology
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <chrono>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <zivid_parameter_server/visibility_control.h>

using namespace std::chrono_literals;

namespace zivid_parameter_server
{
class ZividParameterServer
{
public:
  ZIVID_PARAMETER_SERVER_PUBLIC
  explicit ZividParameterServer(const rclcpp::NodeOptions& options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    node_ = rclcpp::Node::make_shared("zivid_parameter_server", options);
    parameters_client_ = rclcpp::SyncParametersClient::make_shared(node_);
    while (!parameters_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }

    auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
      std::stringstream ss;
      ss << "\nParameter event:\n new parameters:";
      for (auto& new_parameter : event->new_parameters)
      {
        ss << "\n  " << new_parameter.name;
      }
      ss << "\n changed parameters:";
      for (auto& changed_parameter : event->changed_parameters)
      {
        ss << "\n  " << changed_parameter.name;
      }
      ss << "\n deleted parameters:";
      for (auto& deleted_parameter : event->deleted_parameters)
      {
        ss << "\n  " << deleted_parameter.name;
      }
      ss << "\n";
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    };

    node_->declare_parameter("foo");

    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return this->node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

}  // namespace zivid_parameter_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zivid_parameter_server::ZividParameterServer)